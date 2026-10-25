# HD INSERTS — many fine places on one planet, one law (proposal, 2026-10-24)

Mark, 2026-10-24: "make it so that whatever we are doing for hardcoding in the Merrimack is moved to
the scene file. I want to be able to add multiple HD inserts across the globe but have them allow the
same math, and interact if they do end up being neighbors or overlapping."

An **insert** is a place's high-detail water: a shallow-water solver box (SWE) and a solved wave window
(the eikonal BVP, `sim/WaveField`). Everything else the water reads is already global and by
measurement: the bed, the tide atlas, the sea grid, the currents and the cascades on the lattice's
planes (`sim/WaveChart.h`). This note says what still stops two inserts from being the same thing, and
the order to take it apart in.

## Where it stands (after 3ce3dc7 and 57c946b)

| | wave window | solver |
|---|---|---|
| how many | one per scene (`water.wavefield`) | one drawn (`water.swe`), more ridden (`solvers`, dormant) |
| declared by | the scene (since 57c946b; was the code's Merrimack default) | the scene |
| its frame | the ROOT's flat frame (`WaveFieldSource::Align` over `surface.flat`) | its own lat/lon box and chart rows (PHASE C1) |
| place laws in its math | Snell against one bar normal (`barNormalDeg`, `WaveField.cpp` ~603); a current proxy aimed ebb 105 / flood 285, x3.0 from ACT0816, decaying past x = 800 m (`BuildCurrentPlanes`, ~276) | none |
| the bank reads | one window | one solver (`WaterBankLayer::m_swe`) |
| the hull reads | one window (`RootOf`) | the first active window holding the point (`WaterManager::WindowAt`) |
| overlap | n/a | first match wins on the hull, nothing on the bank |

Two consequences beyond the Merrimack:
- A dormant solver is ridden but not drawn. This is the same class of mismatch as today's drift fix. No
  shipped scene declares one yet.
- The current proxy's `x <= 800` is measured in root metres since the root moved onto the cloud
  (2026-10-11). At the Merrimack its tip now stands about 2 km east of the jetties. It is only read
  when no solved current exists.

## The law (what "the same math" means)

1. **No place constant inside a solve.**
   - Refraction comes from the eikonal's own direction field: action is transported along
     k̂ = ∇φ/|∇φ|, ∇·(A² c_g k̂ / σ_r) = 0. The phase is already this eikonal (M9bw).
   - That removes `barNormalDeg` and gives focusing as well as spreading. The fixed frame's Kr ≤ 1
     was an accepted liberty.
   - The current comes from the manager, per cell: the solver where one holds water, else the
     measured field. The proxy goes.
2. **Every insert in its own chart.**
   - A window Aligns over the cloud origin nearest it, not over the root. A window 100 km from the root
     is otherwise a tangent plane 100 km long.
   - The bank addresses it by rows about the eye, as it already addresses the solver
     (`SweDomain::KernelRows`). The hull addresses it through its place.
3. **One phase gauge.**
   - Each window's deep-water seed is the plane wave on the cascades' own chart planes:
     k_∞ d̂ · `WaveChart::UOf`.
   - Two windows in one chart cell then agree to the bit in deep water. The 190-400 km cells make
     that almost every pair.
4. **Overlap or contact is one solve, not a blend.**
   - The windows already live on one global lattice: Align snaps every window to z16 page texels.
   - Inserts whose texel boxes overlap or touch form a **cluster**, solved as one eikonal over the
     union of their texels (the bounding grid, masked).
   - One value per texel, and the interaction is exact: a bar in one shades the other. There is
     nothing to cross-fade between solved fields.
   - Disjoint clusters solve apart. The bank tile carries the short list of clusters that touch it.
     The hull loops the same list.
5. **Solvers compose by a partition of unity; they interact through their open sea.**
   - Over the solvers holding a point, the weights are w_i = f_i / max(1, Σf) (f the domain weight).
     The deviation blends linearly: the solvers' fields are deterministic estimates of one water, so
     the variance-preserving operator of `WaveChart.h` is wrong for them.
   - Each solver's Flather faces take the manager's composed level and current there, excluding
     itself, in place of the atlas alone. Neighbours and nests then couple, a step apart (an
     overlapping Schwarz coupling; the radiating faces keep it stable).
   - The cascades against the solved field stay what they are: independent realizations. Their
     handover is where the variance-preserving operator belongs.
6. **The frame's waterline** is the atlas level under the eye, not the scene solver's station. It is
   only the datum the bank's departures are stored from (a NULL tile reads it).

## Declaration

One list replaces `water.swe`, `solvers` and `water.wavefield`. A place's scene (or its manifest in
`data/places`) says its own inserts. A scene that composes places lists them:

```json
"inserts": [
  {"name": "merrimack",
   "solver": {"box": [-70.9907, 42.7001, -70.77, 42.845], "cellM": 13.728},
   "waves":  {"box": [-70.8271, 42.8111, -70.7860, 42.8293], "cellM": 2}},
  {"name": "plum-island-sound", "solver": {"box": [...]}}
]
```

Boxes are lat/lon: an insert stands at its place, not about any anchor. Every other number (spectrum,
closures, buckets) is the world's and stays in `world.json` and `data/wave_scene.json`.

## Order (each step gated before and after, at home and past the mouth's cell)

1. **W1** refraction by transport; **W2** the manager's current. **Done 2026-10-24.**
   - No place constant is left in the solve. `barNormalDeg` is gone from the code and the scenes.
   - W1 carries the action along the eikonal, with Mase's diffraction marched along the ray
     (ALGEBRA.md `wavefield`).
   - W2 is the field current with the solver's over it, by the solver's domain weight.
   - The python twin (`proofs/wave_field_check.py`) was already stale: a cumsum gauge, a 16-component
     table. The engine's own `[wavetest]` limits replace it for refraction.
   - Still open: `WaterManager::SolverRefine` replaces the field current with the solver's inside the
     domain's one-cell edge band, because the hull's current is twinned with the bank's and the bank
     holds no field current. Blending both is S2's.
2. **D** the `inserts` list read into today's single window and solver, behaviour unchanged. merrimack,
   mavericks and mavericks_beach are rewritten to it.
3. **W3** each window in its own chart, with the WaveChart phase gauge.
4. **S1** the bank draws every active solver (a table); the hull composes the same set.
5. **W4** clusters: overlap is one solve, with a per-tile cluster list on the bank and the hull.
6. **S2/S3** the solvers' partition of unity and their coupling through the open sea.

What this does not touch: the bed, imagery and buildings are already global datasets by measurement.
The harvesters that still cut per-place boxes (`harvest/.../Tides.cs`, `Water.cs`, `Waves.cs`,
`Currents.cs`, `HqBathy.cs`, `Route.cs`, `Aton.cs`, `Survey.cs`) are their own task.
