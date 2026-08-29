// ================================================================================================
//  M5c: the sparse shallow-water solver -- the estuary actually FLOWING.
//
//  State lives in atlas banks as the DEVIATION from the analytic tide plane: eta_bank = eta -
//  tide(t). A NULL tile therefore means "the stateless model is exactly right here", which is
//  the gameplan's thesis expressed as hydrodynamics: the offshore sponge relaxes the deviation
//  to zero, and everything the basin does differently -- the lag behind the ocean, the throat
//  jet, the river's standing slope -- IS the resident state.
//
//  Scheme: the classic virtual-pipe model (Mei et al. 2007): per-cell outflow fluxes to the four
//  neighbours accelerate down the water-surface gradient, are scaled so a cell never ships more
//  volume than it holds (wet/dry handled for free), and the height update is the flux
//  divergence. Damping keeps it unconditionally friendly at dt <= ~0.3 dx / sqrt(g h_max).
//
//  Texture space: +x = east, +y = SOUTH (bathy row 0 is the northern edge). CsSweDerive flips v.
//  Flux channels: x = +x (east), y = +y (south), z = -x (west), w = -y (north).
// ================================================================================================

cbuffer SweCb : register(b0) {
    uint  gNx, gNy;              // bathy grid dims (1701 x 890)
    uint  gEtaTilesX, gFluxTilesX;
    uint  gEtaTileW, gEtaTileH, gFluxTileW, gFluxTileH;
    uint  gListCount;
    float gWorldX0;
    uint  gPad1, gPad2;
    float gDx, gDt, gDamp, gTideNavd;
    float gSpongeX0, gSpongeRate, gRiverDEta, gGravity;   // gRiverDEta = west-boundary target
                                                          // DEVIATION (river tide - ocean tide)
    float4 gRiverBox;            // x = west strip width (texels), y = relax rate per substep,
                                 // z = SOUTH strip start row (>= ny disables), w = south target
                                 // deviation -- Plum Island Sound's Ipswich entrance lies
                                 // outside the window, so its tide enters as data too (M6d)
};

StructuredBuffer<uint> gTileList : register(t0);
Texture2D<float> gBathy : register(t1);          // NAVD88 m; row 0 = north
RWTexture2D<float>  gEta  : register(u0);        // DEVIATION from the tide plane, m
RWTexture2D<float2> gFlux : register(u1);        // SIGNED face fluxes, m^3/s: x = across the
                                                 // EAST face (+east), y = across the SOUTH
                                                 // face (+south). Staggered C-grid.
RWTexture2D<float4> gUv   : register(u2);        // derived: u east, v north, speed, valid

float BedAt(int2 t) {
    if (any(t < 0) || t.x >= (int)gNx || t.y >= (int)gNy) return 100.0f;   // outside = wall
    return gBathy.Load(int3(t, 0));
}

float EtaAt(int2 t) {
    if (any(t < 0) || t.x >= (int)gNx || t.y >= (int)gNy) return 0.0f;
    return gEta[t];   // NULL tiles read 0 = "exactly the tide plane"
}

// Water-surface elevation (absolute, NAVD) if wet, else the bed (dry cells present their ground
// so gradients push water back downhill instead of through it).
float SurfaceAt(int2 t, out float h) {
    const float bed = BedAt(t);
    const float eta = gTideNavd + EtaAt(t);
    h = max(eta - bed, 0.0f);
    return (h > 0.0f) ? eta : bed;
}

// The offshore sponge, RAMPED: 0 inside the estuary and over the entrance bar (live SWE), 1 in
// open water where the stateless ocean is the truth. A hard sponge edge is a reflecting wall --
// the first cut had one, and the deep zone checkerboarded and flooded the throat with slosh.
float SpongeAt(int2 t) {
    const float worldX = gWorldX0 + (t.x + 0.5f) * gDx;
    return smoothstep(gSpongeX0, gSpongeX0 + 700.0f, worldX);
}

uint2 EtaTexel(uint3 id) {
    const uint t = gTileList[id.z];
    return uint2(t % gEtaTilesX, t / gEtaTilesX) * uint2(gEtaTileW, gEtaTileH) + id.xy;
}

uint2 FluxTexel(uint3 id) {
    const uint t = gTileList[id.z];
    return uint2(t % gFluxTilesX, t / gFluxTilesX) * uint2(gFluxTileW, gFluxTileH) + id.xy;
}

// ---- init / reset --------------------------------------------------------------------------

[numthreads(16, 16, 1)]
void CsSweClearEta(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gEtaTileW || id.y >= gEtaTileH) return;
    gEta[EtaTexel(id)] = 0.0f;
}

[numthreads(16, 16, 1)]
void CsSweClearFlux(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gFluxTileW || id.y >= gFluxTileH) return;
    gFlux[FluxTexel(id)] = 0;
}

// Full-surface (non-list) clear of the derived-current texture: texels the derive pass never
// visits (NULL tiles, land) must read valid=0 so the sea falls back to the analytic jet.
[numthreads(16, 16, 1)]
void CsSweUvClear(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gNx || id.y >= gNy) return;
    gUv[id.xy] = float4(0, 0, 0, 0);
}

// ---- one substep ---------------------------------------------------------------------------

// One signed face update. eta/h describe the owner cell; nbr is the cell across the face.
// The first cut used the classic game pipe model (four RECTIFIED outflows, max(0, .)): with
// honest ocean-scale friction, passing gravity waves pump both opposing pipes up and nothing
// drains them -- a ratchet that filled the offshore with ~30 m^3/s phantom fluxes and froze
// eta. Signed staggered faces cannot ratchet.
float FaceUpdate(float q, int2 t, int2 nbr) {
    float h, hn;
    const float eta = SurfaceAt(t, h);
    const float etan = SurfaceAt(nbr, hn);
    const float sill = max(BedAt(t), BedAt(nbr));
    const float hface = max(max(eta, etan) - sill, 0.0f);   // upwind-ish face depth
    if (hface <= 0.0f) return 0.0f;

    // Accelerate down the surface gradient; implicit quadratic bottom drag (Cd = 0.0025);
    // a whisper of linear damping for numerical hygiene.
    q += gDt * gGravity * hface * (eta - etan);   // (g * hface * dEta/dx) * dx face width
    const float uf = q / max(hface * gDx, 1e-3f);
    q *= gDamp / (1.0f + gDt * 0.0025f * abs(uf) / max(hface, 0.3f));

    // Positivity: no face may move more than a quarter of its DONOR cell's volume per step.
    const float Vself = h * gDx * gDx;
    const float Vnbr = hn * gDx * gDx;
    return clamp(q, -0.25f * Vnbr / gDt, 0.25f * Vself / gDt);
}

[numthreads(16, 16, 1)]
void CsSweFlux(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gFluxTileW || id.y >= gFluxTileH) return;
    const int2 t = int2(FluxTexel(id));
    if (t.x >= (int)gNx || t.y >= (int)gNy) return;

    float2 q = gFlux[t];
    q.x = FaceUpdate(q.x, t, t + int2(1, 0));   // east face
    q.y = FaceUpdate(q.y, t, t + int2(0, 1));   // south face (texture +y)
    // Kill reflections off the sponge: waves entering the analytic ocean just fade.
    const float sp = SpongeAt(t);
    if (sp > 0.0f) q *= 1.0f - 0.5f * sp * gSpongeRate;
    gFlux[t] = q;
}

[numthreads(16, 16, 1)]
void CsSweHeight(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gEtaTileW || id.y >= gEtaTileH) return;
    const int2 t = int2(EtaTexel(id));
    if (t.x >= (int)gNx || t.y >= (int)gNy) return;

    // Divergence of the signed face fluxes: volume in = west+north faces, out = my own faces.
    const float2 qc = gFlux[t];
    const float qw = gFlux[int2(t.x - 1, t.y)].x;   // across my west face (+ = into me)
    const float qn = gFlux[int2(t.x, t.y - 1)].y;   // across my north face (+ = into me)
    float dEta = gEta[t] + gDt * ((qw - qc.x) + (qn - qc.y)) / (gDx * gDx);

    // Offshore sponge: the open sea IS the analytic tide; deviations die here. This is also the
    // tidal forcing -- the basin drains into / fills from a boundary pinned to the real clock.
    dEta *= 1.0f - SpongeAt(t) * gSpongeRate;

    // West boundary: the upriver tide from the M1 station fits, interpolated to this edge
    // (Newburyport <-> Salisbury Point). The CUDEM window ends here but the DATA keeps going:
    // relaxing this strip to the real lagged river tide routes the entire upriver prism's
    // demand through the modelled channel. The truncation error becomes a boundary condition,
    // and the boundary condition is NOAA's.
    if (t.x < (int)gRiverBox.x && BedAt(t) < 2.0f) {
        dEta += (gRiverDEta - dEta) * gRiverBox.y;
    }
    // South boundary: the sound's tide (entrance clock, slightly lagged) enters where the
    // window truncates its real mouth -- but ONLY through the deep channel. Pinning the whole
    // marsh edge let the tide short-circuit into the Merrimack basin across box-averaged marsh
    // and halved the throat current (diagnosed by the basin losing its lag entirely).
    if (t.y >= (int)gRiverBox.z && BedAt(t) < -2.0f) {
        dEta += (gRiverBox.w - dEta) * gRiverBox.y;
    }

    gEta[t] = clamp(dEta, -6.0f, 6.0f);
}

// ---- derived surface currents (for rendering, Doppler, and validation probes) ---------------

[numthreads(16, 16, 1)]
void CsSweDerive(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gEtaTileW || id.y >= gEtaTileH) return;
    const int2 t = int2(EtaTexel(id));
    if (t.x >= (int)gNx || t.y >= (int)gNy) return;

    float h;
    SurfaceAt(t, h);
    // Dry cells and the sponged open sea hand the render back to the stateless model (the
    // analytic jet is ACT-calibrated; out there it is the better answer).
    if (h < 0.05f || SpongeAt(t) > 0.5f) {
        gUv[t] = float4(0, 0, 0, 0);
        return;
    }
    // Cell-centred velocity: the mean of the two faces in each axis.
    const float2 qc = gFlux[t];
    const float qw = gFlux[int2(t.x - 1, t.y)].x;
    const float qn = gFlux[int2(t.x, t.y - 1)].y;
    const float u = 0.5f * (qc.x + qw) / (h * gDx);
    const float v = -0.5f * (qc.y + qn) / (h * gDx);   // texture +y is south; world v is north
    gUv[t] = float4(u, v, length(float2(u, v)), 1.0f);
}
