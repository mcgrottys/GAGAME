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
// PHASE C1: THE SOLVER'S CHART (SweDomain::KernelRows) about the reader's own frame: the planes U, V,
// W of the domain's cells (a ratio of planes about its anchor, HIERARCHY 4.4) and (nx, ny, 1 = a
// solver stands, 0). A point's cell is SolverCell(p) (cell i spans [i, i + 1)); the point is the
// solver's where SolverDen(p) > 0 and the cell lies in [0, nx) x [0, ny). The LAST rows.
#define HP_SOLVER_ROWS_DECL float4 gSvU; float4 gSvV; float4 gSvW; float4 gSvO;
#define SolverDen(p) (dot((p), gSvW.xyz) + gSvW.w)
#define SolverCell(p) (float2(dot((p), gSvU.xyz) + gSvU.w, dot((p), gSvV.xyz) + gSvV.w) / SolverDen(p))
#endif
