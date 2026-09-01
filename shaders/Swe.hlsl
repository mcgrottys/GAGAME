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
    float gDy;                   // M6r: north-south texel metres (equiangular CUDEM grid:
                                 // dy = 13.65 != dx = 10.08 -- the per-axis metric)
    float gWestUext;             // M6r: Flather's u_ext (m/s, +east): the west TRANSPORT
                                 // (river Q minus the upriver prism demand) over the live
                                 // section area -- radiation alone cannot carry a prism
    float gDx, gDt, gDamp, gTideNavd;
    float gSpongeX0, gSpongeRate, gRiverDEta, gGravity;   // gRiverDEta = west-boundary target
                                                          // DEVIATION (river tide - ocean tide)
    float4 gRiverBox;            // x = west EXTERIOR column width (texels; Flather pin, M6r),
                                 // y = relax rate per substep (south strip only),
                                 // z = SOUTH strip start row (>= ny disables), w = south target
                                 // deviation -- Plum Island Sound's Ipswich entrance lies
                                 // outside the window, so its tide enters as data too (M6d)
    float gTideRate;             // M6r: d(tide plane)/dt, m/s -- the prism source term
    float gPadA, gPadB, gPadC;
};

StructuredBuffer<uint> gTileList : register(t0);
Texture2D<float> gBathy : register(t1);          // NAVD88 m; row 0 = north
RWTexture2D<float>  gEta  : register(u0);        // DEVIATION from the tide plane, m
RWTexture2D<float2> gFlux : register(u1);        // SIGNED face fluxes, m^3/s: x = across the
                                                 // EAST face (+east), y = across the SOUTH
                                                 // face (+south). Staggered C-grid.
RWTexture2D<float4> gUv   : register(u2);        // derived: u east, v north, speed, valid
// M9h: grad(flow) -- the FIRST field in this engine whose residency was decided by the Cayley
// closure rather than by a physics policy. grad is a grade-1 operator, so nabla U is the
// geometric product of two vectors: Cl2ProductSignature(kG1, kG1) = kG0 | kG2. The scalar part
// is the divergence, the bivector part is the vorticity, and NOTHING else can appear -- the
// algebra says so before a single texel is read. Two channels, exactly the two grades.
RWTexture2D<float2> gMv   : register(u3);        // x = div (grade 0), y = curl (grade 2)

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

// M9h: THE DERIVED FIELD. Central differences on the derived current, one dispatch over the
// gradient bank's OWN resident list -- which is the eta bank's list dilated by one tile,
// because this stencil reads its neighbours and a null tile would silently swallow the write
// (GAMEPLAN 4.2, hazard 1 and hazard 3 in the same kernel).
//
// Land and dry cells present zero: a null tile reads zero anyway, so the two agree and the
// consumer never has to ask which it got.
[numthreads(16, 16, 1)]
void CsSweVelGrad(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gEtaTileW || id.y >= gEtaTileH) return;
    const uint2 t = EtaTexel(id);
    if (t.x >= gNx || t.y >= gNy) return;
    const float4 c = gUv[t];
    if (c.w < 0.5f) { gMv[t] = float2(0, 0); return; }

    const uint2 xm = uint2(max(int(t.x) - 1, 0), t.y);
    const uint2 xp = uint2(min(t.x + 1u, gNx - 1u), t.y);
    const uint2 ym = uint2(t.x, max(int(t.y) - 1, 0));
    const uint2 yp = uint2(t.x, min(t.y + 1u, gNy - 1u));
    float4 a = gUv[xm], b = gUv[xp], d = gUv[ym], e = gUv[yp];
    if (a.w < 0.5f) a = c;
    if (b.w < 0.5f) b = c;
    if (d.w < 0.5f) d = c;
    if (e.w < 0.5f) e = c;

    const float dudx = (b.x - a.x) / (2.0f * gDx);
    const float dvdx = (b.y - a.y) / (2.0f * gDx);
    const float dudy = (e.x - d.x) / (2.0f * gDy);
    const float dvdy = (e.y - d.y) / (2.0f * gDy);
    // grade 0 = div, grade 2 = curl. The wedge is the bivector coefficient in e1^e2.
    gMv[t] = float2(dudx + dvdy, dvdx - dudy);
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
// M6r, the per-axis metric: the grid is equiangular, so a face is faceLen metres LONG and its
// gradient acts over span metres -- (faceLen, span) = (dy, dx) for east faces, (dx, dy) for
// south faces. The old isotropic gDx overdrove every north-south term by dy/dx = 1.35.
float FaceUpdate(float q, int2 t, int2 nbr, float faceLen, float span) {
    float h, hn;
    const float eta = SurfaceAt(t, h);
    const float etan = SurfaceAt(nbr, hn);
    const float sill = max(BedAt(t), BedAt(nbr));
    const float hface = max(max(eta, etan) - sill, 0.0f);   // upwind-ish face depth
    if (hface <= 0.0f) return 0.0f;

    // Accelerate down the surface gradient; implicit quadratic bottom drag (Cd = 0.0025);
    // a whisper of linear damping for numerical hygiene.
    q += gDt * gGravity * hface * (eta - etan) * faceLen / span;   // g*hface*(dEta/span)*faceLen
    const float uf = q / max(hface * faceLen, 1e-3f);
    // Cd 0.0015: sandy-estuary range. 0.0025 was fine over the 5 km mouth window; across the
    // M6d wide window's 15 km reach the extra friction accumulated ~40 min of phase lag.
    q *= gDamp / (1.0f + gDt * 0.0015f * abs(uf) / max(hface, 0.3f));

    // Positivity: no face may move more than a quarter of its DONOR cell's volume per step.
    const float Vself = h * gDx * gDy;
    const float Vnbr = hn * gDx * gDy;
    return clamp(q, -0.25f * Vnbr / gDt, 0.25f * Vself / gDt);
}

[numthreads(16, 16, 1)]
void CsSweFlux(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gFluxTileW || id.y >= gFluxTileH) return;
    const int2 t = int2(FluxTexel(id));
    if (t.x >= (int)gNx || t.y >= (int)gNy) return;

    float2 q = gFlux[t];
    q.x = FaceUpdate(q.x, t, t + int2(1, 0), gDy, gDx);   // east face (dy long, dx span)
    q.y = FaceUpdate(q.y, t, t + int2(0, 1), gDx, gDy);   // south face (texture +y)

    // M6r: FLATHER west boundary -- the open boundary RADIATES. The exterior column rides the
    // station river tide as pinned data; the flux across its east face is the Flather
    // condition  u_b = u_ext + sqrt(g/h) * (eta_ext - eta_int),  so an interior surplus flows
    // OUT at the gravity-wave speed instead of reflecting off a relaxed strip (the Dirichlet
    // strip was a soft wall: transients bounced, and the basin's phase carried the echo).
    // u_ext is the PRESCRIBED transport (USGS river Q minus the upriver prism demand) over
    // the live section: radiation alone cannot carry a prism -- without u_ext the through-
    // flow throttles behind the standing dEta it needs to sustain itself (measured: the gap
    // current lost a third of its amplitude).
    if (t.x < (int)gRiverBox.x && BedAt(t) < 2.0f) {
        float hI;
        const float etaI = SurfaceAt(t + int2(1, 0), hI);
        const float etaX = gTideNavd + gRiverDEta;          // exterior = the river-tide data
        const float sill = max(BedAt(t), BedAt(t + int2(1, 0)));
        const float hf = max(max(etaX, etaI) - sill, 0.0f);
        q.x = 0.0f;
        if (hf > 0.05f) {
            const float ub = gWestUext + sqrt(gGravity / hf) * (etaX - etaI);
            const float V = gDx * gDy / gDt;                // positivity, as everywhere
            q.x = clamp(hf * gDy * ub, -0.25f * hI * V, 0.25f * hf * V);
        }
    }

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
    float dEta = gEta[t] + gDt * ((qw - qc.x) + (qn - qc.y)) / (gDx * gDy);

    // M6r, THE PRISM TERM. The eta bank stores deviation from a MOVING plane: when the plane
    // rises the volume must still ARRIVE, so every resident cell books the rise as debt. The
    // debt's gradient against the free boundaries (the sponge = the analytic ocean, the pinned
    // west column = the river data) IS the flood current; the ebb is the debt paid back. This
    // also makes wet/dry hydrodynamic: a flat's surface no longer rides the plane for free --
    // it waits for the water. (Measured before this term: 43 m^3/s of storage flux across the
    // whole harbor at peak flood, ~5% of the real prism; the x5 render gain was compensation.)
    dEta -= gTideRate * gDt;

    // Offshore sponge: the open sea IS the analytic tide; deviations die here. This is also the
    // tidal forcing -- the basin drains into / fills from a boundary pinned to the real clock.
    dEta *= 1.0f - SpongeAt(t) * gSpongeRate;

    // M6r west boundary, the Flather pair's other half: the EXTERIOR column is the upriver
    // tide from the M1 station fits, interpolated to this edge, held as data (hard pin -- the
    // radiation lives in the face flux, not in a relax rate). The CUDEM window ends here but
    // the DATA keeps going: the truncation error becomes a boundary condition, and the
    // boundary condition is NOAA's.
    if (t.x < (int)gRiverBox.x && BedAt(t) < 2.0f) {
        dEta = gRiverDEta;
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
    // Cell-centred velocity: the mean of the two faces in each axis, each over its OWN face
    // area (M6r: east faces are dy long, south faces dx -- the isotropic gDx overstated v
    // by 35%).
    const float2 qc = gFlux[t];
    const float qw = gFlux[int2(t.x - 1, t.y)].x;
    const float qn = gFlux[int2(t.x, t.y - 1)].y;
    const float u = 0.5f * (qc.x + qw) / (h * gDy);
    const float v = -0.5f * (qc.y + qn) / (h * gDx);   // texture +y is south; world v is north
    gUv[t] = float4(u, v, length(float2(u, v)), 1.0f);
}
