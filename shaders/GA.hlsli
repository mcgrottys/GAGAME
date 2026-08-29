// ================================================================================================
//  GEOMETRIC ALGEBRA ON TEXTURE CHANNELS -- the shared toolkit.
//
//  Extracted from Common.hlsli in M3 so COMPUTE shaders (which have their own root signatures
//  and must not drag in the scene's bindings) can use the algebra too. This is the block vqview
//  wrote as scaffolding with zero call sites; as of M3 the velocity-gradient pass calls it.
//
//  A multivector in G(n) has exactly 2^n components, which is always a power of two, which is
//  exactly the shape a texture format wants:
//
//      G2 multivector   1 scalar + 2 vector + 1 bivector   = 4  -> one RGBA texel
//      G3 rotor         1 scalar + 3 bivector              = 4  -> one RGBA texel (a quaternion)
//      G3 multivector   1 + 3 + 3 + 1                      = 8  -> two RGBA texels
//
//  WHERE IT EARNS ITS KEEP
//    * grad applied to a CURRENT field gives divergence as grade 0 and vorticity as grade 2 from
//      one pass into one texel. Okubo-Weiss (strain^2 - vorticity^2) is then a norm on that
//      multivector, and its negative regions are vortex-dominated water.
//    * Refraction of a wavenumber vector by a current is a ROTATION. As a rotor, k' = R k R~
//      carries the rotation plane and sense intrinsically, so the "which way does this normal
//      point" class of bug cannot be expressed.
// ================================================================================================
#ifndef GA_GA_HLSLI
#define GA_GA_HLSLI

// G2 multivector: s + v.x e1 + v.y e2 + b e12
struct Mv2 {
    float s;
    float2 v;
    float b;
};

Mv2 MakeMv2(float4 packed) {
    Mv2 m;
    m.s = packed.x;
    m.v = packed.yz;
    m.b = packed.w;
    return m;
}
float4 PackMv2(Mv2 m) { return float4(m.s, m.v, m.b); }

// Full geometric product in G2. e1e1 = e2e2 = 1, e12e12 = -1, e1e12 = e2, e2e12 = -e1.
Mv2 GeometricProduct(Mv2 a, Mv2 b) {
    Mv2 r;
    r.s   = a.s * b.s + dot(a.v, b.v) - a.b * b.b;
    r.v.x = a.s * b.v.x + a.v.x * b.s - a.v.y * b.b + a.b * b.v.y;
    r.v.y = a.s * b.v.y + a.v.y * b.s + a.v.x * b.b - a.b * b.v.x;
    r.b   = a.s * b.b + a.b * b.s + a.v.x * b.v.y - a.v.y * b.v.x;
    return r;
}

float ScalarPart(Mv2 m)   { return m.s; }
float BivectorPart(Mv2 m) { return m.b; }

// Okubo-Weiss from the gradient of a 2D flow: strain^2 - vorticity^2.
// Negative => rotation-dominated (an eddy); positive => strain-dominated (a shear line).
float OkuboWeiss(float dudx, float dudy, float dvdx, float dvdy) {
    const float normalStrain = dudx - dvdy;
    const float shearStrain  = dvdx + dudy;
    const float vorticity    = dvdx - dudy;
    return normalStrain * normalStrain + shearStrain * shearStrain - vorticity * vorticity;
}

// A G3 rotor is a quaternion: (s, e23, e31, e12). Applying it is the sandwich product R v R~.
float3 RotorApply(float4 R, float3 v) {
    const float3 t = 2.0f * cross(R.yzw, v);
    return v + R.x * t + cross(R.yzw, t);
}

// ** TRAP. ** Hardware bilinear filtering interpolates CHANNELS, not geometric objects. A
// componentwise lerp of two rotors is not a rotor. Rotor fields must be point-sampled and
// renormalised, or slerped by hand.
float4 NlerpRotor(float4 a, float4 b, float t) {
    if (dot(a, b) < 0.0f) b = -b;      // shortest arc: q and -q are the same rotation
    return normalize(lerp(a, b, t));
}

#endif  // GA_GA_HLSLI
