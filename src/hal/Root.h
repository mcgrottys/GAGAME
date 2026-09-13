// ================================================================================================
//  Root.h - M12 step 3d: THE ROOT LAYOUT. Nine hand-written root signatures (a range array, a
//  parameter array, a versioned desc, a serialize, a create -- thirty lines each) become one
//  builder that says what a kernel binds, in the order it binds it.
//
//  THE HOUSE LAYOUT: root CBVs (b0 constants, b1 per-draw), root SRVs (t0, a list in the frame
//  arena), 32-bit constants where a kernel wants four numbers without an upload, and
//  descriptor tables of SRV and UAV ranges into the one shader-visible heap. A range is
//  DESCRIPTORS_VOLATILE always -- every one of the nine sites said so, because a table's slots
//  are rewritten after the signature is built (the churn and solver tables; the bindless heap
//  itself) -- and its offset in the table is the running sum of the ranges before it. An
//  UNBOUNDED range (kUnbounded) covers the heap from where it starts and adds nothing to that
//  sum, so every unbounded range of a table starts at offset 0: the shared graphics layout
//  views the whole heap six times (Texture2D, Texture3D, TextureCube, Texture2D<uint>,
//  Texture2DArray, TextureCubeArray -- spaces 1..6) and the water bank's kernel twice, all
//  from the table's start. That is the bindless law: one heap, one index, every
//  dimensionality.
//
//  Root descriptors carry no flags and shader visibility ALL unless said (no site set either).
//  A static sampler is SamplerFields -- the nine fields the sites set, the D3D desc made only
//  here (step 3f); StaticSampler() is the house preset (the whole mip range, ComparisonFunc
//  NEVER, every stage) that nine of the eleven in the tree are, and the water bank's two spell
//  their own (MaxLOD 0, no comparison function at all: dead, since its shaders declare no
//  sampler -- step 3d's finding, kept as found). Version 1.1 always.
//
//  THE GATE: every site serialized its hand-written desc and the builder's, and the two blobs
//  were compared byte for byte in step 3d's probe before the hand-written block was deleted.
//  Eight of nine were EQUAL as version 1.1; the ocean's mip reduce was the one version-1.0
//  signature in the tree, EQUAL when the builder's parameters were serialized the old way,
//  and it now ships as 1.1 with the volatile flag that 1.0 implied.
//
//  DX12-first: the parameter kinds are D3D's, the samplers D3D's desc, the result the
//  ID3D12RootSignature a pipeline binds. Build() throws on failure the way every site's
//  GA_CHECK did, after logging the serializer's own message under the tag.
// ================================================================================================
#pragma once

#include "hal/Gpu.h"

#include <climits>
#include <cstdint>
#include <initializer_list>
#include <string>
#include <vector>

namespace ga::hal {

constexpr uint32_t kUnbounded = UINT_MAX;

struct Range {
    D3D12_DESCRIPTOR_RANGE_TYPE type;
    uint32_t reg, count, space;
};
inline Range SrvRange(uint32_t reg, uint32_t count, uint32_t space = 0) {
    return {D3D12_DESCRIPTOR_RANGE_TYPE_SRV, reg, count, space};
}
inline Range UavRange(uint32_t reg, uint32_t count, uint32_t space = 0) {
    return {D3D12_DESCRIPTOR_RANGE_TYPE_UAV, reg, count, space};
}

// A static sampler, field by field: the fields the sites set and nothing else -- the desc's
// other fields (MipLODBias, BorderColor, MinLOD, RegisterSpace) stay its zeros, as every site
// left them. The defaults are the house sampler's; a site that differs says so in the field.
struct SamplerFields {
    uint32_t reg = 0;
    D3D12_FILTER filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
    D3D12_TEXTURE_ADDRESS_MODE addrU = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    D3D12_TEXTURE_ADDRESS_MODE addrV = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    D3D12_TEXTURE_ADDRESS_MODE addrW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    float maxLod = D3D12_FLOAT32_MAX;
    D3D12_COMPARISON_FUNC comparison = D3D12_COMPARISON_FUNC_NEVER;
    D3D12_SHADER_VISIBILITY visibility = D3D12_SHADER_VISIBILITY_ALL;
    uint32_t maxAnisotropy = 0;

    // The desc law: the nine writes the house sampler made into a zeroed desc, in its order.
    D3D12_STATIC_SAMPLER_DESC ToDesc() const {
        D3D12_STATIC_SAMPLER_DESC s{};
        s.Filter = filter;
        s.AddressU = addrU;
        s.AddressV = addrV;
        s.AddressW = addrW;
        s.MaxAnisotropy = maxAnisotropy;
        s.MaxLOD = maxLod;
        s.ShaderRegister = reg;
        s.ShaderVisibility = visibility;
        s.ComparisonFunc = comparison;
        return s;
    }
};
// The house sampler: `filter` with one address mode on all three axes, the whole mip chain
// (MaxLOD FLOAT32_MAX), no comparison (NEVER), visible to every stage; `maxAnisotropy` only
// matters under an anisotropic filter (the shared layout's s3 says 8).
inline SamplerFields StaticSampler(uint32_t reg, D3D12_FILTER filter,
                                   D3D12_TEXTURE_ADDRESS_MODE addr, uint32_t maxAnisotropy = 0) {
    return SamplerFields{reg, filter, addr, addr, addr, D3D12_FLOAT32_MAX,
                         D3D12_COMPARISON_FUNC_NEVER, D3D12_SHADER_VISIBILITY_ALL, maxAnisotropy};
}

class RootLayout {
public:
    RootLayout& Cbv(uint32_t reg, uint32_t space = 0,
                    D3D12_SHADER_VISIBILITY vis = D3D12_SHADER_VISIBILITY_ALL) {
        return Descriptor(D3D12_ROOT_PARAMETER_TYPE_CBV, reg, space, vis);
    }
    RootLayout& Srv(uint32_t reg, uint32_t space = 0,
                    D3D12_SHADER_VISIBILITY vis = D3D12_SHADER_VISIBILITY_ALL) {
        return Descriptor(D3D12_ROOT_PARAMETER_TYPE_SRV, reg, space, vis);
    }
    RootLayout& Constants(uint32_t reg, uint32_t count, uint32_t space = 0,
                          D3D12_SHADER_VISIBILITY vis = D3D12_SHADER_VISIBILITY_ALL) {
        Param p;
        p.p.ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
        p.p.Constants.ShaderRegister = reg;
        p.p.Constants.RegisterSpace = space;
        p.p.Constants.Num32BitValues = count;
        p.p.ShaderVisibility = vis;
        m_params.push_back(std::move(p));
        return *this;
    }
    RootLayout& Table(std::initializer_list<Range> ranges,
                      D3D12_SHADER_VISIBILITY vis = D3D12_SHADER_VISIBILITY_ALL) {
        Param p;
        p.p.ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
        p.p.ShaderVisibility = vis;
        uint32_t offset = 0;
        for (const Range& r : ranges) {
            D3D12_DESCRIPTOR_RANGE1 d{};
            d.RangeType = r.type;
            d.NumDescriptors = r.count;
            d.BaseShaderRegister = r.reg;
            d.RegisterSpace = r.space;
            d.Flags = D3D12_DESCRIPTOR_RANGE_FLAG_DESCRIPTORS_VOLATILE;
            d.OffsetInDescriptorsFromTableStart = offset;
            if (r.count != kUnbounded) offset += r.count;
            p.ranges.push_back(d);
        }
        m_params.push_back(std::move(p));
        return *this;
    }
    RootLayout& Sampler(const SamplerFields& s) {
        m_samplers.push_back(s.ToDesc());
        return *this;
    }
    RootLayout& Flags(D3D12_ROOT_SIGNATURE_FLAGS f) {
        m_flags = f;
        return *this;
    }

    // The serialized form (Build's, and the gate's). Throws on a malformed layout, after
    // logging the serializer's own message under the tag -- the ocean and the renderer did.
    Com<ID3DBlob> Serialize(const char* tag) const {
        std::vector<D3D12_ROOT_PARAMETER1> ps;
        D3D12_VERSIONED_ROOT_SIGNATURE_DESC vd{};
        Assemble(ps, vd);
        Com<ID3DBlob> blob, err;
        const HRESULT hr = D3D12SerializeVersionedRootSignature(&vd, &blob, &err);
        if (FAILED(hr)) {
            if (err) {
                Log("[%s] root signature: %s", tag,
                    static_cast<const char*>(err->GetBufferPointer()));
            }
            GA_CHECK(hr);
        }
        return blob;
    }
    Com<ID3D12RootSignature> Build(Gpu& gpu, const char* tag) const {
        const Com<ID3DBlob> blob = Serialize(tag);
        Com<ID3D12RootSignature> rs;
        GA_CHECK(gpu.Device()->CreateRootSignature(0, blob->GetBufferPointer(),
                                                  blob->GetBufferSize(), IID_PPV_ARGS(&rs)));
        return rs;
    }

private:
    struct Param {
        D3D12_ROOT_PARAMETER1 p{};
        std::vector<D3D12_DESCRIPTOR_RANGE1> ranges;
    };
    // The D3D desc over this layout's storage: `ps` must outlive the use of `vd`.
    void Assemble(std::vector<D3D12_ROOT_PARAMETER1>& ps,
                  D3D12_VERSIONED_ROOT_SIGNATURE_DESC& vd) const {
        ps.clear();
        ps.reserve(m_params.size());
        for (const Param& p : m_params) {
            D3D12_ROOT_PARAMETER1 d = p.p;
            if (d.ParameterType == D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE) {
                d.DescriptorTable.NumDescriptorRanges = static_cast<UINT>(p.ranges.size());
                d.DescriptorTable.pDescriptorRanges = p.ranges.data();
            }
            ps.push_back(d);
        }
        vd = {};
        vd.Version = D3D_ROOT_SIGNATURE_VERSION_1_1;
        vd.Desc_1_1.NumParameters = static_cast<UINT>(ps.size());
        vd.Desc_1_1.pParameters = ps.empty() ? nullptr : ps.data();
        vd.Desc_1_1.NumStaticSamplers = static_cast<UINT>(m_samplers.size());
        vd.Desc_1_1.pStaticSamplers = m_samplers.empty() ? nullptr : m_samplers.data();
        vd.Desc_1_1.Flags = m_flags;
    }
    RootLayout& Descriptor(D3D12_ROOT_PARAMETER_TYPE type, uint32_t reg, uint32_t space,
                           D3D12_SHADER_VISIBILITY vis) {
        Param p;
        p.p.ParameterType = type;
        p.p.Descriptor.ShaderRegister = reg;
        p.p.Descriptor.RegisterSpace = space;
        p.p.ShaderVisibility = vis;
        m_params.push_back(std::move(p));
        return *this;
    }

    std::vector<Param> m_params;
    std::vector<D3D12_STATIC_SAMPLER_DESC> m_samplers;
    D3D12_ROOT_SIGNATURE_FLAGS m_flags = D3D12_ROOT_SIGNATURE_FLAG_NONE;
};

}  // namespace ga::hal
