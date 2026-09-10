// ================================================================================================
//  M2 ocean synthesis, all compute. Four kernels, run per cascade:
//
//    CsInitSpectrum  once per forecast tick: h0(k) from the partition spectra (JONSWAP wind sea +
//                    Gaussian swells, cos^2s spreading) with deterministic per-texel Gaussians.
//    CsModulate      per frame: THE ROTOR. h(k,t) = h0(k) e^{-iwt} + conj(h0(-k)) e^{+iwt} --
//                    each complex multiply is a Cl(2)+ rotor advancing the bin's phase
//                    (GAMEPLAN.md section 5, application 3). Then the eight linear spectral
//                    fields (h, Dx, Dz, hx, hz, Jxx, Jzz, Jxz) packed two-real-per-complex into
//                    two float4 textures: IFFT of (f_hat + i g_hat) lands f in .re and g in .im.
//    CsFft           radix-2 DIT in groupshared memory, one 256-row/column per group,
//                    bit-reversed load (reversebits), e^{+i} synthesis sign, NO normalisation --
//                    the un-normalised inverse transform IS the plain sum of amplitudes, which is
//                    exactly how the spectrum amplitudes were scaled.
//    CsAssemble      real-space fields -> displacement texture (Dx, h, Dz) and derivative
//                    texture (hx, hz, Jacobian, foam).
//
//  Everything reads/writes UAVs so no resource ever changes state inside the loop; only the two
//  output textures transition to pixel-shader-readable for the draw.
// ================================================================================================

cbuffer PartsCb : register(b0) {
    uint4  gPartCount;   // x = number of partitions (<= 4)
    float4 gPart[8];     // two float4s per partition:
                         //   [2i+0] = fp, specScale, sigF, gamma (0 => Gaussian swell)
                         //   [2i+1] = dirToX, dirToZ, spreadS, dirNorm
};

cbuffer FftCb : register(b1) {
    uint  uN;            // 256
    uint  uDir;          // CsFft: 0 = rows, 1 = columns
    uint  uCascade;
    uint  uSeed;
    float fKLo;          // cascade band, rad/m
    float fKHi;
    float fL;            // patch size, metres
    float fTime;         // seconds since the forecast cycle
    float fLambda;       // choppy displacement scale
    float fFoamScale;
    float fFoamBias;     // Jacobian threshold
    uint  uPad0;
    float4 fPad;
};

RWTexture2D<float4> gU0 : register(u0);
RWTexture2D<float4> gU1 : register(u1);
RWTexture2D<float4> gU2 : register(u2);
RWTexture2D<float4> gU3 : register(u3);

static const float PI = 3.14159265f;
static const float G = 9.81f;

float2 cmul(float2 a, float2 b) { return float2(a.x * b.x - a.y * b.y, a.x * b.y + a.y * b.x); }

// -------------------------------------------------- deterministic per-texel Gaussians
uint2 pcg2d(uint2 v) {
    v = v * 1664525u + 1013904223u;
    v.x += v.y * 1664525u;  v.y += v.x * 1664525u;
    v ^= v >> 16u;
    v.x += v.y * 1664525u;  v.y += v.x * 1664525u;
    v ^= v >> 16u;
    return v;
}

float2 GaussianPair(uint2 texel) {
    const uint2 h = pcg2d(texel * uint2(3u, 5u) + uint2(uSeed + uCascade * 197u, uSeed * 13u));
    const float u1 = max((h.x & 0xFFFFFFu) / 16777216.0f, 1e-6f);
    const float u2 = (h.y & 0xFFFFFFu) / 16777216.0f;
    const float r = sqrt(-2.0f * log(u1));
    return r * float2(cos(2.0f * PI * u2), sin(2.0f * PI * u2));
}

// -------------------------------------------------- the spectrum (shapes mirror SeaState.cpp)
float ShapeJonswap(float f, float fp, float gamma) {
    if (f <= 1e-4f) return 0.0f;
    const float r = fp / f;
    const float sigma = (f <= fp) ? 0.07f : 0.09f;
    const float d = (f - fp) / (sigma * fp);
    return pow(f, -5.0f) * exp(-1.25f * r * r * r * r) * pow(gamma, exp(-0.5f * d * d));
}

// Abramowitz & Stegun 7.1.26; enough accuracy for energy bookkeeping.
float Erf(float x) {
    const float s = sign(x);
    x = abs(x);
    const float t = 1.0f / (1.0f + 0.3275911f * x);
    const float y = 1.0f - (((((1.061405429f * t - 1.453152027f) * t) + 1.421413741f) * t
                             - 0.284496736f) * t + 0.254829592f) * t * exp(-x * x);
    return s * y;
}

// Directional variance for one k bin.
//
// ** THE NARROW-SWELL TRAP. ** A long swell's Gaussian peak (sigma ~ 4 mHz) is narrower than one
// k-bin's frequency footprint on a 756 m patch (~11 mHz). Point-sampling S(f) at bin centres
// loses most of the energy between bins -- the first storm render came out a third of its Hs.
// So the radial direction is INTEGRATED over each bin's frequency extent: closed-form via erf
// for the Gaussian swell, point * width for the broad JONSWAP where sampling is fine. m0 is then
// conserved regardless of how the spectrum and the grid line up.
float BinVariance(float2 k) {
    const float kLen = length(k);
    if (kLen < 1e-5f || kLen < fKLo || kLen >= fKHi) return 0.0f;
    const float w = sqrt(G * kLen);
    const float f = w / (2.0f * PI);
    const float dfdk = G / (4.0f * PI * w);
    const float dk = 2.0f * PI / fL;

    float acc = 0.0f;
    const uint n = gPartCount.x;
    [loop] for (uint i = 0; i < n; ++i) {
        const float4 a = gPart[2 * i + 0];
        const float4 b = gPart[2 * i + 1];
        float Ef;   // integral of S(f) over this bin's radial frequency extent
        if (a.w > 0.0f) {
            Ef = a.y * ShapeJonswap(f, a.x, a.w) * dfdk * dk;
        } else {
            const float hw = 0.5f * dfdk * dk;
            const float d0 = (f - hw - a.x) / a.z;
            const float d1 = (f + hw - a.x) / a.z;
            // sqrt(2*pi)/2 = 1.2533141; erf argument scale 1/sqrt(2)
            Ef = a.y * a.z * 1.2533141f
                 * (Erf(d1 * 0.70710678f) - Erf(d0 * 0.70710678f));
        }
        // Angular: same trap, same cure. The cos^2s lobe is ~exp(-s d^2 / 4), a Gaussian of
        // sigma = sqrt(2/s) -- at s=60 that is ~10 degrees, narrower than an angular bin at the
        // swell peak. Integrate the Gaussian over this bin's angular cell; the erf difference is
        // the cell's exact energy FRACTION, so no normalisation constant survives at all.
        const float cosD = (k.x * b.x + k.y * b.y) / kLen;
        const float crossD = (k.x * b.y - k.y * b.x) / kLen;
        const float dTh = atan2(crossD, cosD);
        const float sigTh = sqrt(2.0f / b.z);
        const float hTh = 0.5f * dk / kLen;
        const float frac = 0.5f * (Erf((dTh + hTh) / (sigTh * 1.41421356f))
                                   - Erf((dTh - hTh) / (sigTh * 1.41421356f)));
        acc += Ef * frac;
    }
    return acc;
}

float2 WaveK(uint2 id) {
    const float sx = (id.x < uN / 2) ? (float)id.x : (float)id.x - (float)uN;
    const float sy = (id.y < uN / 2) ? (float)id.y : (float)id.y - (float)uN;
    return 2.0f * PI / fL * float2(sx, sy);
}

// u0 = h0 texture: (h0(k).re, .im, h0(-k).re, .im)
[numthreads(8, 8, 1)]
void CsInitSpectrum(uint3 id : SV_DispatchThreadID) {
    if (id.x >= uN || id.y >= uN) return;
    const uint2 mid = uint2((uN - id.x) % uN, (uN - id.y) % uN);   // the -k texel

    const float2 gp = GaussianPair(id.xy);
    const float2 gm = GaussianPair(mid);
    const float ap = 0.5f * sqrt(BinVariance(WaveK(id.xy)));
    const float am = 0.5f * sqrt(BinVariance(-WaveK(id.xy)));      // -k evaluated consistently

    gU0[id.xy] = float4(ap * gp, am * gm);
}

// u0 = h0 (read), u1 = packed A (h + iDx, Dz + i hx), u2 = packed B (hz + iJxx, Jzz + iJxz)
[numthreads(8, 8, 1)]
void CsModulate(uint3 id : SV_DispatchThreadID) {
    if (id.x >= uN || id.y >= uN) return;
    const float2 k = WaveK(id.xy);
    const float kLen = length(k);
    const float4 h0 = gU0[id.xy];

    // The rotor advances every bin's phase analytically. Stateless in time.
    //
    // THE SIGN IS -w, AND IT IS LOAD-BEARING. CsFft synthesises with e^{+ik.x} (its twiddle is
    // +2pi(i%hl)/len), so pairing that with e^{+iwt} would make each bin's phase fronts satisfy
    // k.x + wt = const -- fronts that run along -k. Since BinVariance puts a partition's energy
    // at k parallel to dirTo, the whole sea then ran OPPOSITE to the direction it was declared
    // with: a swell built "toward the east" measured 19.6 m/s westward. Tessendorf's equations
    // carry the same pairing, but his |k^.w^|^2 spectrum is symmetric in k, so both directions
    // get equal energy and the error cannot show; the cos^2s lobe here is NOT symmetric (s = 8
    // wind sea, s = 60 swell), so it showed as soon as anything asked which way the sea ran.
    // With -w the bins go as e^{i(k.x - wt)} and travel along +k, which is dirTo.
    //
    // The conjugate half needs no separate change: it reads (rot.x, -rot.y), so it flips with
    // this line and the field stays exactly real (hk(-k) = conj(hk(k)) either way).
    const float w = sqrt(G * kLen);
    const float wt = -w * fTime;
    const float2 rot = float2(cos(wt), sin(wt));
    const float2 hk = cmul(h0.xy, rot) + cmul(float2(h0.z, -h0.w), float2(rot.x, -rot.y));

    float2 dx = 0, dz = 0, hx = 0, hz = 0, jxx = 0, jzz = 0, jxz = 0;
    if (kLen > 1e-5f) {
        const float2 kn = k / kLen;
        dx = float2(hk.y, -hk.x) * kn.x;       // -i (kx/k) h
        dz = float2(hk.y, -hk.x) * kn.y;
        hx = float2(-hk.y, hk.x) * k.x;        //  i kx h
        hz = float2(-hk.y, hk.x) * k.y;
        jxx = hk * (k.x * k.x / kLen);         //  (kx^2/k) h
        jzz = hk * (k.y * k.y / kLen);
        jxz = hk * (k.x * k.y / kLen);
    }

    // Pack: C = f_hat + i g_hat  =>  C.re = f.re - g.im, C.im = f.im + g.re
    const float2 c0 = float2(hk.x - dx.y, hk.y + dx.x);
    const float2 c1 = float2(dz.x - hx.y, dz.y + hx.x);
    const float2 c2 = float2(hz.x - jxx.y, hz.y + jxx.x);
    const float2 c3 = float2(jzz.x - jxz.y, jzz.y + jxz.x);
    gU1[id.xy] = float4(c0, c1);
    gU2[id.xy] = float4(c2, c3);
}

// u0 = srcA, u1 = dstA, u2 = srcB, u3 = dstB. One 256-line per group; uDir picks rows/columns.
groupshared float4 gsA[256];
groupshared float4 gsB[256];

[numthreads(256, 1, 1)]
void CsFft(uint3 gtid : SV_GroupThreadID, uint3 gid : SV_GroupID) {
    const uint i = gtid.x;
    const uint2 coord = (uDir == 0) ? uint2(i, gid.x) : uint2(gid.x, i);
    const uint rev = reversebits(i) >> 24;   // 8-bit reversal (N = 256)
    gsA[rev] = gU0[coord];
    gsB[rev] = gU2[coord];

    [unroll] for (uint len = 2; len <= 256; len <<= 1) {
        GroupMemoryBarrierWithGroupSync();
        const uint hl = len >> 1;
        if (i < 128) {
            const uint j = (i / hl) * len + (i % hl);
            const float ang = 2.0f * PI * (float)(i % hl) / (float)len;   // +i: synthesis
            const float2 tw = float2(cos(ang), sin(ang));
            const float4 a = gsA[j];
            const float4 b = gsA[j + hl];
            const float2 t0 = cmul(tw, b.xy), t1 = cmul(tw, b.zw);
            gsA[j] = float4(a.xy + t0, a.zw + t1);
            gsA[j + hl] = float4(a.xy - t0, a.zw - t1);
            const float4 c = gsB[j];
            const float4 d = gsB[j + hl];
            const float2 t2 = cmul(tw, d.xy), t3 = cmul(tw, d.zw);
            gsB[j] = float4(c.xy + t2, c.zw + t3);
            gsB[j + hl] = float4(c.xy - t2, c.zw - t3);
        }
    }
    GroupMemoryBarrierWithGroupSync();
    gU1[coord] = gsA[i];
    gU3[coord] = gsB[i];
}

// u0 = displacement out (Dx, h, Dz, 0), u1 = derivatives out (hx, hz, J, foam),
// u2 = real-space A, u3 = real-space B.
[numthreads(8, 8, 1)]
void CsAssemble(uint3 id : SV_DispatchThreadID) {
    if (id.x >= uN || id.y >= uN) return;
    const float4 a = gU2[id.xy];
    const float4 b = gU3[id.xy];
    const float h = a.x, dx = a.y, dz = a.z, hx = a.w;
    const float hz = b.x, jxx = b.y, jzz = b.z, jxz = b.w;

    const float J = (1.0f + fLambda * jxx) * (1.0f + fLambda * jzz)
                    - fLambda * fLambda * jxz * jxz;
    const float foam = saturate((fFoamBias - J) * fFoamScale);

    gU0[id.xy] = float4(fLambda * dx, h, fLambda * dz, 0.0f);
    gU1[id.xy] = float4(hx, hz, J, foam);
}
