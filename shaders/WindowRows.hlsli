// ================================================================================================
//  WindowRows.hlsli - PHASE B1 (out/integration/plan_phase_b.md): THE ROWS A KERNEL CARRIES FOR ITS
//  CHAIN. A kernel (the bank, the churn, the solver) has no surface rows: it carries its own copy of
//  its level's window rows, appended at the END of its cbuffer (priors 22), and HeightPages.hlsli's
//  HP_WINDOW_ROWS turns Window.hlsli's chain onto them. Rank k + 1's planes U, V, W about the
//  kernel's frame (PageTexelUv's), the box's origin less the anchor two ranks a row, the slices
//  (ranks 1..4, then rank 5 and K) -- SurfaceFrame::KernelRows fills them from the one RowsOf.
//  Included before the kernel's cbuffer, which names HP_WINDOW_ROWS_DECL as its last rows.
// ================================================================================================
#ifndef GA_WINDOW_ROWS_HLSLI
#define GA_WINDOW_ROWS_HLSLI
#define HP_WINDOW_ROWS_DECL \
    float4 gHwU[5]; float4 gHwV[5]; float4 gHwW[5]; float4 gHwO[3]; uint4 gHwS[2];
#endif
