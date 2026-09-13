// ================================================================================================
//  DxTest - the DX12 CONTRACT gate (M7v). The class of bug this repo keeps meeting is a
//  contract broken between two languages: a CB row added on one side only, a compute kernel
//  sampling the bindless heap through a static sampler (silent zeros on this driver), an AST
//  edge whose code anchor rotted. This gate makes each of those LOUD the moment a new data
//  type, row, or kernel is added:
//
//    1. CB PARITY -- every registered C++ CB struct is parsed from its header (bytes) and
//       compared against DXC REFLECTION of the shader's cbuffer (the compiler's own truth,
//       packing included). Add a row on one side only -> FAIL naming both sizes.
//    2. THE SAMPLER LAW -- every [numthreads] compute entry in shaders/ is DISCOVERED,
//       compiled, and reflected; an entry whose post-DCE resource set uses a sampler
//       together with an UNBOUNDED bindless array trips the proven-trap alarm. New kernels
//       are covered the day they are written; no table to maintain.
//    3. AST ANCHORS -- every registered edge's code anchor must name a file that exists.
//
//  Pure CPU + dxcompiler; no device, no GPU. Runs in --selftest as its own gate.
// ================================================================================================
#pragma once

namespace ga {
bool RunDxSelfTest();
}
