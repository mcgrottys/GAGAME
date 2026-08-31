# GAGAME — Gameplan for a Real-Time, Data-Driven Global Ocean Simulator

*Drafted 2026-08-28. Successor to `vqview-inlet`. All station IDs, endpoints, S3 buckets, D3D12 semantics, and citations below were verified by live requests on that date; the few unverified items are marked UNVERIFIED.*

---

## 1. Thesis

**NOAA carries the state. The GPU carries the phase.**

The forecast models (GFS-Wave, RTOFS, GoMOFS, HRRR) already integrate the ocean's slow state on supercomputers and publish it 1–4× per day. Tides are even better: they are literally a sum of cosines with published coefficients. So the runtime never integrates the ocean — it *evaluates* it:

```
surface(x, t) = fold( tide(x,t), surge(x,t), swell(x,t), wind_sea(x,t), current(x,t), … )
```

where every layer is an analytic function of position, time, and cached forecast data. This is the same trick `vqview-inlet` already plays (baked `a, k, cos φ, sin φ` + one rotor `e^{-σt}` per component per frame) and the same trick Tessendorf FFT oceans play (`h(k,t) = h₀(k)e^{iω(k)t}`) — generalized to a stack of real-world layers.

**State becomes a sparse exception, not the default.** Some physics genuinely needs memory: the estuary's river–tide interaction, boat wakes, foam decay. Those live only in *resident tiles* of sparse GPU resources. Everywhere else the tile is NULL and — by D3D12 Tier-2 guarantee — reads as zero, meaning "no local deviation from the analytic ocean." That's the unifying idea: **statelessness as the base field, statefulness as sparse residency.** Your tiled-GA pattern is exactly the mechanism that makes this composable (§4).

Two free superpowers fall out of statelessness:

- **Time travel.** `t` is just a parameter — scrub tides forward a week, replay yesterday's storm, fast-forward the ebb. Nothing to re-simulate.
- **Deterministic multiplayer.** Two machines pinned to the same forecast cycle + clock render the identical ocean with zero sync traffic; only vessels need networking.

---

## 2. Starting point: what `vqview-inlet` gives us

The dossier verdict: ~7,200 LOC, unusually high quality, already a stateless Merrimack simulator. **Carry forward:**

| Asset | Why |
|---|---|
| `FieldSet` (bindless field table, per-channel scale/bias) | Its own header anticipates this project: *"the moment field data lives in a tiled/reserved resource, the mapping from world position to texel stops being a single affine transform."* Add a residency layer underneath; shaders keep working. |
| `Layer` contract + `b1`/`b2` constant-buffer publication | Proven extension seam — the vessel layer touched no other layer. |
| `wave_model.py` | Current-modified dispersion `(σ+kU)² = gk·tanh(kh)` via bracket+bisect (Newton diverges near blocking), shoaling, refraction, total-based `Hs ≤ 0.60h` breaking. Hard-won, correct, keep. |
| `Common.hlsli` GA block | `Mv2` (Cl(2) multivector), full geometric product, Okubo-Weiss, `RotorApply`, `NlerpRotor` — **complete and currently dead code**. This project is where it finally gets call sites. |
| Tessellation block | Crack-free `EdgeFactor` (shared-corner trick), screen-space density invariant. |
| Kelvin wake solver | Stationary-phase quadratic; the 19.47° wedge emerges from the discriminant. Stateless wakes for free. |
| `Gpu.cpp` / `Shader.cpp` | Minimal D3D12 + runtime DXC with safe hot reload (failed compile keeps last PSO). |

**Rewrite:** the bake-to-PNG pipeline (8-bit quantized coefficients, one frozen scenario, 40 MB per 2.5 km crop — does not survive a globe), `potential_flow.py` (its own docstring lists why: zero vorticity, ebb/flood mirror symmetry), all site-specific hardcoding, the 45-flag `main.cpp`, and the flat-earth `float2 worldXZ` assumption.

Notable alignment: vqview's `FieldSet.h` already plans exactly the multivector texel this project needs — *"grad(U) gives divergence as grade 0 and vorticity as grade 2 in one RGBA texel."*

---

## 3. The layer stack ("chain and fold")

Each layer is a pure field `f(x, t) → multivector`, refreshed from cache on its own cadence. The per-frame **fold** composes them:

| # | Layer | Math | Data source | Refresh |
|---|---|---|---|---|
| 1 | **Tide height** | Σ Aᵢ cos(ωᵢt + φᵢ), 37 harmonic constituents/station, interpolated along-channel | CO-OPS `harcon.json` — stations §6.2 | fetch **once**, keep forever |
| 2 | **Tidal current** | sinusoid fit between slack/max-flood/max-ebb tables | CO-OPS `currents_predictions` (ACT0816 etc.) | days–weeks |
| 3 | **Circulation / SSH / SST / salinity** | trilinear sample of forecast grids, time-interpolated between forecast hours | RTOFS global 1/12° → GoMOFS 700 m regional | daily / 4×day |
| 4 | **Wind** | grid sample | GFS 0.25° global → HRRR 3 km near-field | 4×day / hourly |
| 5 | **Wind sea + 3 swell partitions** | Horvath TMA/JONSWAP spectra parameterized by (4)+(5) partitions → FFT cascades, phase advanced analytically | GFS-Wave 0.25° (Hs, Tp, dir per partition) | 4×day |
| 6 | **Wave–current interaction** | Doppler-shifted dispersion ω′ = ω₀(k) + k·U(x) at synthesis time; refraction/shoaling from bathy | layers 2+3 folded into 5 | per frame (analytic) |
| 7 | **River** | discharge boundary condition + along-channel travel lag | USGS 01100000 (Lowell), 01100500 (Lawrence) | 15 min |
| 8 | **Estuary SWE** *(stateful, sparse)* | shallow-water solver, tide-forced at mouth, discharge-forced upstream | layers 1+7 as boundaries, CUDEM bathy | sim step |
| 9 | **Wakes/foam** *(stateful, sparse)* | Kelvin analytic + injected decay fields | vessels; Jacobian whitecap criterion | sim step |

Fold rules: displacements **add** (superposition); currents **Doppler** wave phase (6); wind **modulates** spectra (5); tide+river **force boundaries** of the one real solver (8). The fold graph is a DAG evaluated per frame; only rows 8–9 own state, and only inside resident tiles.

---

## 4. The tiled multivector atlas (your abstract pattern, formalized)

### 4.1 The idea

A multivector field over the ocean, stored **grade-banked**:

- **Grade 0 (+ grade 3):** scalars — SSH, temperature, salinity, foam energy, phase. Dense 1-D buffers / low-res textures. *No tiling needed* (as you predicted).
- **Grade 1 (vectors):** currents, wind, wave momentum — 2-D tiled textures for surface fields, 3-D tiled for depth-resolved fields.
- **Grade 2 (bivectors):** vorticity, wave orbital planes, EM `F` — 2-D/3-D tiled. In 2-D this is one channel (e₁₂); in 3-D three (e₂₃, e₃₁, e₁₂).

Per tile, per grade bank: an entry either points at resident memory **or is NULL, meaning "this grade is identically zero here."** Geometric-product kernels consult a per-tile **grade-signature nibble** (bit per grade); the product of two signatures is a 16×16 lookup through the Cayley table of the algebra, so **sparsity propagates through the geometric product structurally** — output tiles are only allocated where a contributing grade pair is resident.

Two lovely packing accidents:

- A **Cl(2) surface multivector is exactly one RGBA16F texel** (s, v.x, v.y, b) — which is precisely vqview's `Mv2` and its planned div/curl texel.
- A **Cl(3) multivector splits even/odd into two RGBA texels**: even part (s + 3 bivectors) *is a quaternion/rotor bank*; odd part (vector + pseudoscalar) is the flow bank. The even/odd split is also exactly the symplectic decomposition that turns hypercomplex FFTs into pairs of ordinary complex FFTs (§5.3) — storage layout and transform algorithm agree.

### 4.2 Verified platform facts (D3D12)

- **Tier 2 tiled resources guarantee (verbatim from Microsoft docs): reads from NULL-mapped tiles return zero; writes to NULL-mapped tiles are discarded.** And *"adapters that support feature level 12_0 all support TIER_2 or greater"* — so on the RTX 5060 the null-tile-reads-zero semantics your pattern needs are hardware-guaranteed, branch-free.
- Tiles are 64 KB (`64KB_UNDEFINED_SWIZZLE`), created via `CreateReservedResource`, mapped with queue-side `UpdateTileMappings`; all mappings start NULL; tiles of one resource may come from multiple heaps. 64 KB = 128×128 texels at 32 bpp, 128×64 at 64 bpp (RGBA16F).
- **Tier 3 adds 3-D (volume) tiled resources** — needed for your 3-D banks; query `D3D12_TILED_RESOURCES_TIER` at startup, fall back to 2-D texture-array banks below Tier 3.
- Sampler feedback (DX12 Ultimate) exists for streaming decisions, but sim-tile residency is already known CPU-side — use feedback only for *rendering* texture LOD. DirectStorage moves 64 KB tiles (exactly one tile per request) NVMe→GPU with GDeflate decompression; Intel's SamplerFeedbackStreaming sample demonstrates the full feedback → load → `UpdateTileMappings` → residency-map loop to copy.

**Hazards to design around (verified):**
1. Null-tile **writes are silently discarded** — never dispatch simulation writes into unmapped tiles; dispatch from the occupancy list only, and map tiles *ahead* of moving features (wakes) before they arrive.
2. `UpdateTileMappings` cost scales with region count — batch per frame, budget mappings/frame, recycle from a round-robin heap pool.
3. Cross-tile stencils (SWE) need a one-tile apron in the occupancy list or explicit ghost-cell exchange.

### 4.3 Residency architecture

CPU occupancy quadtree per bank → per-frame batched `UpdateTileMappings` → tiny residency/indirection texture (megatexture pattern, van Waveren) sampled by shaders that need to *know* (dispatch pruning, LOD clamp) — while plain sampling needs no branch at all thanks to Tier 2 zeros. Compute dispatches walk compacted occupied-tile lists via `ExecuteIndirect`, one specialized kernel per active Cayley block. Prior art proving sparse *simulation* (not just texturing) on tiled resources works: Alex Dunn's GDC 2015 sparse Eulerian fluid and NVIDIA Flow.

### 4.4 Is it novel?

The literature agent's verdict after sweeping arXiv, IEEE, Springer AACA, SIGGRAPH/GDC material, and the GA library ecosystem: **every ingredient exists separately — nobody has combined them.** Tier-2 null tiles (Dunn/Flow, scalar fields only), per-grade multivector storage (Garamon, CPU objects), grade-sparse product codegen (kingdon, symbolic), Clifford convolution on flows (Ebling–Scheuermann), Clifford-FFT neural PDE layers (Brandstetter et al.) — but no publication or library stores multivector *fields* in GPU tiled resources with per-grade residency and Cayley-driven kernel skipping. **The tiled multivector atlas appears novel as a system.** A short writeup would land well at AGACSE, a SIGGRAPH Talk, or bivector.net.

---

## 5. Applications of the pattern (the answer to "what could this be used for")

Ranked by how directly they serve this game; ★ = appears genuinely novel, rest are adaptations of published math to this storage.

1. **Eddy/vortex/front detection via Clifford convolution.** Ebling–Scheuermann multivector filter masks (geometric-product convolution) match vortices/sources/saddles rotation-independently. Vorticity `∇∧v` lives in the bivector bank and is null except along the ebb jet, storm tracks, and Gulf Stream edges — precisely tile-sparse; a null bivector tile skips the whole matching kernel. Gameplay: eddies = fishing spots, whirlpool hazards, race gates.
2. **Velocity-gradient atlas.** `∇v` decomposes by grade: divergence (scalar bank — upwelling/downwelling), vorticity (bivector bank — Okubo-Weiss / eddy cores, activating vqview's dormant `OkuboWeiss()`), strain (symmetric part). Foam/whitecap heuristics and eddy classification become grade-local reads; open ocean is null in all but grade 0. ★ as a GA framing — the research found this restatement is textbook-derivable but unpublished ("small paper-shaped hole").
3. **Spectral ocean as a rotor field.** ★ Tessendorf's `e^{iω(k)t}` *is* a rotor in Cl(2)⁺; Gerstner orbital motion *is* a depth-attenuated rotor applied to an offset vector — vqview's `Water.hlsl` already derives this in comments. Store `h₀(k)` as (scalar + e₁e₂) banks; time advance is pure rotor application; calm tiles are NULL per frequency band. Confirmed gap: no published paper states the GA–ocean-wave connection. You could be first.
4. **Hypercomplex FFT on stock FFT infrastructure.** Ell–Sangwine/Pei: a quaternion/Cl(2) Fourier transform computes as **two ordinary complex FFTs** via symplectic decomposition — which aligns exactly with the even/odd grade banks. The FFT-ocean compute shader doubles as a Clifford-FFT engine for analyzing forecast fields (current+wind+pressure as one multivector signal).
5. **EM sandbox on the same kernels.** In GA, the whole EM field is one bivector field `F = E + IB`, Maxwell is `∇F = J`, vacuum tiles are NULL. The Marmanis metafluid analogy (Lamb vector ↔ E, vorticity ↔ B) means water-vorticity and EM genuinely share code paths. Lightning, radio propagation, auroras — one grade-2 banked field over the same atlas. ★ as shared-infrastructure design.
6. **Clifford neural surrogates hosted on the atlas.** ★ CliffordLayers / Clifford-steerable CNNs / Fengbo (ICLR 2025) run multivector-field convolutions for Navier–Stokes, shallow-water, and Maxwell — today dense in PyTorch, with 2025 papers fighting exactly the perf problem your null-tile grade skipping answers. Long-term toy: train a small Clifford surrogate on GoMOFS history, run inference on the tile atlas — "the forecast lives in a texture."
7. **Grade-shedding LOD.** ★ Distance-based residency *per grade*: far tiles keep only grade-0 height; mid-range maps grade-1 currents; near-camera maps bivector detail. Virtual-texturing economics (Far Cry 4 adaptive VT) generalized so that **algebraic grade is an LOD axis**. No precedent found.
8. **PGA motor layer for floating bodies.** Boats/buoys/debris as motors (Lengyel's conventions, Terathon library); the local wave rotor (app 3) composes directly with the body motor for buoyancy attitude — one algebraic species from ocean surface to rigid body, à la De Keninck's matrix-free PBR renderer ("Look, Ma, No Matrices!", SIGGRAPH 2024).

---

## 6. Data pipeline

### 6.1 Products (all endpoints verified 2026-08-28)

| Product | Use | Res / cadence (measured lag) | Get it from | Cache |
|---|---|---|---|---|
| CO-OPS harmonic constituents | tide layer | per station / ~static | `api.tidesandcurrents.noaa.gov/mdapi/prod/webapi/stations/{id}/harcon.json` | **forever** (annual refresh) |
| CO-OPS current predictions | tidal current layer | per station | `datagetter?product=currents_predictions&interval=MAX_SLACK` | per date window |
| **GFS-Wave** 0.25° (multi_1 retired 2020) | wave spectra: Hs/Tp/dir, wind-sea + **3 swell partitions** | 4×/day (+3h35m) | `s3://noaa-gfs-bdp-pds/gfs.YYYYMMDD/CC/wave/gridded/` (GRIB2+.idx) | cycle-immutable → forever |
| **GFS** 0.25° | global wind | 4×/day (+3h36m) | `s3://noaa-gfs-bdp-pds/.../atmos/` | forever |
| **HRRR** 3 km | Merrimack wind | hourly (+52m) | `s3://noaa-hrrr-bdp-pds`, zarr: `s3://hrrrzarr` | forever |
| **RTOFS** 1/12° | global currents/SST/SSS/SSH | daily, 8-day fcst (~13:00Z) | `s3://noaa-nws-rtofs-pds` (diag/ice verified; `_prog.nc` on S3 UNVERIFIED — fall back to NOMADS https) | forever |
| **GoMOFS** ROMS ~700 m | Gulf of Maine ring: level/current/T/S | 4×/day, 72 h fcst (+2h03m) | `s3://noaa-nos-ofs-pds/gomofs/netcdf/YYYY/MM/DD/`; OPeNDAP subsets via CO-OPS THREDDS | forever |
| NDBC realtime2 | validation buoys | 10–60 min obs | `ndbc.noaa.gov/data/realtime2/{ID}.txt` / `.swdir` | 10–30 min, If-Modified-Since |
| USGS IV | river discharge/stage | 15 min | `waterservices.usgs.gov/nwis/iv` — **decommissioned Q1 2027; migrate to `api.waterdata.usgs.gov/ogcapi/v0/`** | 15–60 min |
| MUR SST (`jplMURSST41`) | pretty global SST without decoding RTOFS | 0.01° daily | CoastWatch ERDDAP griddap | 24 h |
| GEBCO_2026 15″ / ETOPO 2022 | global bathy | static | gebco.net / NCEI | once |
| CUDEM 1/9″ (~3 m) topobathy | Merrimack bathy | static | `coast.noaa.gov/htdata/raster2/elevation/NCEI_ninth_Topobathy_2014_8483/MA_NH_ME/` — 4 tiles around the mouth, ~145 MB each (verify coverage with gdalinfo) | once |
| NECOFS (FVCOM GOM3/MASSBAY) | upgrade path for estuary-scale forecast | daily | SMAST OPeNDAP (http :8080); NOS operational ~FY27 | evaluate later |

### 6.2 Merrimack stations (IDs verified by live API calls)

- **Tide (harmonic prediction stations, 37 constituents each — the river has real phase progression upriver):** `8440452` Plum Island/Merrimack Entrance (M2 = 3.84 ft @ 115.7°), `8440466` Newburyport (M2 = 3.86 @ 122.0° — ≈13 min later), `8440273` Salisbury Point, `8440369` Merrimacport, `8440889` Riverside. Reference/live obs: `8443970` Boston (active NWLON, 6-min water level).
- **Tidal currents (subordinate, max/slack tables):** `ACT0816` Merrimack River entrance (flood 1.84 kn @ 285°, ebb @ 105°), `ACT0821` Newburyport, `ACT0826` Plum Island Sound, `ACT0831` Annisquam.
- **Wave buoys:** `44013` Boston (directional spectra ✓), `44098` Jeffrey's Ledge (directional ✓), `44029` NERACOOS A01 Mass Bay (closest; ADCP currents; non-directional), `44030` B01.
- **USGS:** `01100000` Merrimack @ Lowell (live-verified: 998 cfs), `01100500` @ Lawrence, `01101000` Parker R @ Byfield. No mainstem gauge below Lawrence → apply ~½–1 day freshwater travel lag to Newburyport.

### 6.3 The Harvester (cache daemon) + politeness playbook

A small tool (Python first; the vqview bakery culture already exists) that maintains `cache/` and emits engine-ready fp16 tiles + a manifest; the engine file-watches and hot-reloads, same philosophy as the DXC shader reload.

1. **Mirrors first:** all gridded data from AWS NODD S3 (anonymous, unthrottled, built for bulk). NOMADS only for grib-filter subsets S3 can't do cheaply. **NOMADS hard limit: 120 hits/min/IP gets you blocked — stay 10× under.**
2. **`.idx` byte-range reads:** fetch the tiny GRIB index, then HTTP-Range only the records needed (HRRR 10 m U/V = 2 ranged reads, not 143 MB).
3. **Cycle files are immutable → cache forever**, keyed `{model}/{YYYYMMDD}/{cycle}/{fhr}/{subset-hash}`. Only "newest complete cycle?" ever polls.
4. **Schedule around measured lags** (GFS/Wave +3.6 h, HRRR +1 h, RTOFS +13 h, GoMOFS +2 h), capped exponential backoff + jitter; serve previous cycle on failure.
5. **Tides never touch the network after day one** — constituents fetched once, evaluated locally forever. CO-OPS: always send `application=GAGAME`; USGS: gzip + `modifiedSince`; NDBC/ERDDAP: conditional GETs, exact subsets.
6. **Identify yourself:** User-Agent with project name + contact email on every fetcher.
7. **Offline mode is a feature, not a fallback:** tides + bathymetry + last cached cycles = fully playable with zero network. This is also the dev loop — hammer the cache, never the agencies.

---

## 7. Runtime architecture

### 7.1 Three LOD rings

| Ring | Coverage | Data | Technique |
|---|---|---|---|
| **L0 Global** | planet | GFS-Wave 0.25°, RTOFS 1/12°, MUR SST, GEBCO | quad-sphere CDLOD; spectrum tails → slope-variance BRDF roughness (Bruneton 2010); prefiltered Jacobian whitecaps (Dupuy–Bruneton 2012); camera-relative ECEF, doubles on CPU |
| **L1 Gulf of Maine** | ~500 km | GoMOFS 700 m, HRRR wind, GFS-Wave partitions | full FFT cascade ocean + current Doppler fold |
| **L2 Merrimack** | ~10 km | CUDEM 3 m, USGS discharge, tide stations, ACT currents | everything of L1 **+** sparse SWE estuary solver + wave-amplitude transport (shoaling/refraction/blocking) |

Meshing: viewer-centered clipmap/CDLOD rings for both geometry and data cascades (the shipped Crest pattern — "CDClipmaps", 256² cascade textures, wavelength-matched FFT injection). **Not** a projected grid (wobble, world-space effect pain). vqview's screen-space-density tessellation carries into the near field.

### 7.2 Stateless synthesis (per frame)

3–4 FFT cascades, 256²–512², non-integer patch ratios (~15 m→1 km) to hide tiling; own Stockham compute-shader FFT (verified: FidelityFX has **no** FFT; MIT references: GodotOceanWaves, gasgiant/FFT-Ocean); fp16 storage, fp32 butterflies; outputs displacement + Jacobian/foam + slope variance. Spectra rebuilt per region when forecast data ticks: Horvath empirical TMA/JONSWAP + directional spreading, one component set per GFS-Wave partition (wind sea + 3 swells) — buoy-validatable. Currents fold in via Doppler `ω′ = ω₀(k) + k·U(x)`; tide height adds as DC; Kelvin wakes evaluate analytically.

### 7.3 Sparse stateful patches (the atlas earning its keep)

Reserved-resource banks (§4) for: SWE state (η, hu — pinned tiles over the estuary), wake displacement/velocity (tiles allocated ahead of vessels, expired by decay), persistent foam. The composite shader adds patch contributions *unconditionally* — null tiles contribute exactly zero, no branches. SWE solver: virtual-pipe first (Mei et al. 2007 — 4 fetches/cell, stable), upgrade to the Jeschke–Wojtan 2023 SWE + dispersive-Airy split (the first heightfield method doing deep+shallow interaction — wakes refracting onto the flats) when the pipe model's limits show. Tide (M1 layer) forces the seaward boundary; USGS discharge forces upstream; the ebb-jet wave steepening at the mouth then *emerges* instead of being painted.

### 7.4 GA toolchain

- **Shader side: generate, don't link.** No GA library targets HLSL — and that's fine, because grade-banked SoA wants ~a dozen specialized Cayley-block kernels, not a runtime type system. Use **kingdon** (Python, symbolic GA with input-sparsity support) to expand products with declared operand grades → tiny emitter → flat HLSL functions. HLSL 2021 operator overloading is allowed in leaf code (`Rotor2` struct), but hot kernels stay flat SoA.
- **Host side: Terathon Math Library** (MIT, active, matches *PGA Illuminated*) for motors/flectors/rigid transforms. klein is archived (2024) — read for SSE tricks, don't depend. Garamon if we want generated per-grade C++ mirroring the GPU banks; ganja.js for whiteboard prototyping.

---

## 8. Milestones

Each milestone ends with something *fun on screen* plus a validation gate.

- **M0 — Skeleton & null-tile proof** *(~a weekend)*: New CMake app from vqview's `Gpu/Shader/FieldSet/Layer`; query `D3D12_TILED_RESOURCES_TIER` (expect ≥2, hope 3 for volume banks on the RTX 5060); PIX markers from the first triangle. **Gate:** unit test proving null-tile reads = 0, writes discarded, residency map round-trips; PIX capture showing the tile mappings.
- **M1 — The metronome (tides)**: Harvester fetches `harcon.json` for the 5 river stations + Boston, once; analytic tide evaluator (CPU+GPU) with along-channel phase interpolation; time-scrub UI. **Gate:** overlay matches CO-OPS predictions; scrubbing a week feels instant. *Zero recurring network.*
- **M2 — Forecast-driven sea**: GFS-Wave + GFS via S3 `.idx` ranged reads; Horvath spectra per partition → 3–4 Stockham-FFT cascades; clipmap ocean with displacement/Jacobian/foam. **Gate:** rendered Hs at buoy 44013/44098 locations tracks reported Hs; side-by-side measured-vs-synthesized spectrum plot.
- **M3 — Currents & the first live GA**: RTOFS + GoMOFS ingestion; Doppler fold into cascades; tidal-current sinusoids from ACT stations; **velocity-gradient decomposition pass writing div (scalar bank) + vorticity (bivector bank), Okubo-Weiss eddy overlay — vqview's dead GA code goes live.** **Gate:** waves visibly shorten/steepen against the ebb at the mouth; eddies highlighted in the Gulf.
- **M4 — The tiled multivector atlas**: reserved-resource grade banks, occupancy quadtree, batched `UpdateTileMappings`, apron rule, `ExecuteIndirect` over occupied tiles, grade-signature Cayley skipping; port M3 fields in. **Gate:** PIX timing shows null regions cost ~nothing; toggling residency visualizer; memory budget report.
- **M5 — Living estuary**: CUDEM bathy; sparse SWE over the Merrimack (pipe model first), tide + discharge forced; wave-amplitude transport for shoaling/refraction replacing baked coefficients. **Gate:** ebb vs flood asymmetry exists (the thing vqview explicitly couldn't do); river plume visible; bore-ish front on big spring ebb + storm swell. *(DONE 2026-08-28 across M5/M5b/M5c. The pipe model was tried first per plan and RETIRED with cause — its rectified outflows ratchet under honest ocean friction; the shipped scheme is signed-staggered faces + implicit quadratic drag on the same atlas banks, eta stored as deviation-from-the-analytic-tide so NULL = "the stateless model is right". Boundaries are DATA: offshore sponge to the analytic tide, west edge riding the M1 station-interpolated river tide (the truncated upriver prism arrives through NOAA instead of being lost). Gate results: flood/ebb asymmetry emerges with the right dominance sense (1.30 vs ACT's 1.61), gap current r = 0.93 vs ACT0816, slack on the predicted hour; the ×3.2 prism-truncation current gain is the honest cost of the small window — retires when M6 widens the domain. Wave transport shipped as finite-depth dispersion + Green's-law shoaling + breaking clamp (M5b) + the graded line-of-sight swell-shadow mask (M5c); the full wavelets-style directional transport stays on the M6+ wishlist. Bore hunt deferred to a spring-tide + storm replay session.)*
- **M6 — The globe**: quad-sphere CDLOD earth, GEBCO relief, RTOFS/MUR global fields, BRDF-LOD far ocean; seamless zoom orbit → Newburyport with grade-shedding residency per ring. **Gate:** the zoom is one unbroken shot at 60+ fps.
- **M7 — Things that float**: Terathon motors for vessels; buoyancy from analytic surface + gradient; Kelvin wakes + sparse wake tiles (Atlas-style injection); AIS replay from vqview's routes. **Gate:** a boat surfs the ebb jet believably; wake tiles allocate ahead / expire behind.
- **M8 — Sandbox toys (stretch)**: Clifford-convolution eddy hunt minigame; EM layer demo (`F = E + IB` aurora over the ocean, same kernels); time-travel replay of a real storm (cached cycles); Clifford neural surrogate experiment; the AGACSE/SIGGRAPH-Talk writeup of the atlas.

**Perf budget** (1440p, RTX 5060): cascades+synthesis ≤ 1 ms, fold/composite ≤ 0.5 ms, sparse SWE ≤ 1 ms, shading ≤ 1.5 ms → whole ocean ≤ 4 ms, comfortable 120 fps headroom.

---

## 9. PIX workflow (installed: 2603.25)

- `WinPixEventRuntime`: `PIXBeginEvent/PIXEndEvent` per layer/pass with the layer names from §3; `SetName` on every resource ("G2.vorticity.L1" style) so captures read like the plan.
- **GPU captures** to inspect reserved resources: PIX shows tile mappings/residency — the M0 gate and the M4 debugging loop.
- **Timing captures** for the fold and FFT kernels against the §8 budget; occupancy view to check the Cayley-block kernels aren't tiny-dispatch-bound.
- **Programmatic capture** (`PIXGpuCaptureNextFrames` / `pixtool.exe` at `C:\Program Files\Microsoft PIX\2603.25\`) wired to a hotkey and to the Harvester's "new cycle" event, so a capture exists for the exact frame a new forecast folds in.

---

## 10. Risks & open questions

1. **GoMOFS↔estuary blend seam** — 700 m model meets 3 m local solver; needs a relaxation zone at the mouth. (NECOFS MASSBAY mesh density near 42.815 N/−70.81 W: UNVERIFIED — inspect before adopting FVCOM readers.)
2. **RTOFS `_prog.nc` (currents) on S3 unconfirmed** — verified on NOMADS https; confirm bucket layout or budget the NOMADS fallback into the politeness rules.
3. **CUDEM tile coverage** ambiguity — `gdalinfo` the four candidate tiles on download.
4. **USGS API migration** before Q1 2027 (brownouts possible late 2026) — build the Harvester's USGS adapter against the OGC API from day one.
5. **Tier 3 volume tiles** on the RTX 5060 — expected but unqueried; 2-D array fallback specced (§4.2).
6. **Geodesy** — camera-relative ECEF with local tangent frames; vqview is flat-earth throughout, so this is new code, and the FieldSet world→texel comment was written in anticipation of exactly this.
7. **Wave-model honesty** — GFS-Wave partitions are point summaries; reconstructing spectra via Horvath is an approximation. The buoy-validation gates (M2) keep us honest.

---

## 11. Bibliography (all URLs verified 2026-08-28)

**Ocean:** Tessendorf, *Simulating Ocean Water* (course notes) — jtessen.people.clemson.edu/reports/papers_files/coursenotes2004.pdf · Horvath, *Empirical Directional Wave Spectra for CG*, DigiPro '15 (impl: github.com/blackencino/EncinoWaves) · Bruneton et al., *Ocean Lighting / BRDF LOD*, CGF 2010 — maverick.inria.fr/Publications/2010/BNH10 · Dupuy & Bruneton, *Whitecaps*, SA 2012 Briefs (github.com/jdupuy/whitecaps) · Jeschke & Wojtan: *Water Wave Packets* '17, *Water Surface Wavelets* '18, *Wave Curves* '20, *Dispersive SWE* '23 — visualcomputing.ist.ac.at/publications · Chentanez & Müller, SCA 2010 — matthias-research.github.io/pages/publications/hfFluid.pdf · Mei et al., pipe-model SWE, PG 2007 · Johanson projected grid (Lund 2004) · Losasso & Hoppe clipmaps (hhoppe.com/proj/geomclipmap) · Strugar CDLOD (github.com/fstrugar/CDLOD) · Crest (github.com/wave-harmonic/crest) · Sea of Thieves SIGGRAPH '18 (history.siggraph.org) · Malan, *Water in Horizon Forbidden West*, Advances 2022 · Tcheblokov, *War Thunder ocean*, CGDC 2015 · Mihelich & Tcheblokov, *Atlas wakes*, GDC 2019.

**GA:** Hestenes, *Space-Time Algebra*; Oersted Lecture 2003 · Doran & Lasenby, *GA for Physicists* · Cibura & Hildenbrand, *GA Fluid Dynamics*, AGACSE 2008 (gaalop.de) · Panakkal et al., Phys. Fluids 2020 · Marmanis, Phys. Fluids 1998 (metafluid) · Dressel et al., *STA for EM*, Phys. Reports 2015 (arxiv.org/abs/1411.5002) · Ebling & Scheuermann, Clifford convolution (Vis '03) & CFT (TVCG '05) · Hitzer, *QFT & CFT* (CRC 2021); Phil. Trans. A 384(2326) Aug 2026 GA theme issue · Ell & Sangwine, IEEE TIP 2007; Pei et al., IEEE TSP 2001 (QFT via 2 complex FFTs) · Brandstetter et al., *Clifford Neural Layers for PDE Modeling*, ICLR 2023 (github.com/microsoft/cliffordlayers) · Ruhe et al., GCAN ICML '23; CGENN NeurIPS '23 · Zhdanov et al., *Clifford-Steerable CNNs*, ICML '24 · Pepe et al., *Fengbo*, ICLR '25 · Brehmer et al., GATr, NeurIPS '23 · GAALOP (gaalop.de) · kingdon (github.com/tBuLi/kingdon, arxiv 2503.10451) · Garamon, AACA 2019 (per-grade storage) · Lengyel, *PGA Illuminated* 2024 + Terathon-Math-Library (MIT) · De Keninck, *Look, Ma, No Matrices!*, SIGGRAPH 2024 Talks · bivector.net.

**Sparse GPU:** Microsoft D3D12 docs: `D3D12_TILED_RESOURCES_TIER` (Tier-2 null semantics), `UpdateTileMappings`, texture layouts, residency, Sampler Feedback spec, DirectStorage (github.com/microsoft/DirectStorage) · Dunn, *Sparse Fluid Simulation in DirectX*, GDC 2015 (NVIDIA) + NVIDIA Flow · Setaluri et al., SPGrid, SIGGRAPH Asia 2014 · Museth, VDB TOG 2013; NanoVDB · van Waveren, *Software Virtual Textures* 2012 · Obert et al., VT course SIGGRAPH 2012 · Chen, *Adaptive VT in Far Cry 4*, GDC 2015 · Intel SamplerFeedbackStreaming (github.com/GameTechDev/SamplerFeedbackStreaming) · GodotOceanWaves (github.com/2Retr0/GodotOceanWaves) · gasgiant/FFT-Ocean.

**Data:** CO-OPS API + harcon (api.tidesandcurrents.noaa.gov) · NOMADS (nomads.ncep.noaa.gov; 120 hits/min limit) · AWS NODD: noaa-gfs-bdp-pds, noaa-hrrr-bdp-pds, hrrrzarr, noaa-nws-rtofs-pds, noaa-nos-ofs-pds · GoMOFS (tidesandcurrents.noaa.gov/ofs/gomofs) · NECOFS (fvcom.smast.umassd.edu/necofs) · NDBC realtime2 (44013/44098/44029) · USGS waterservices → api.waterdata.usgs.gov migration · GEBCO_2026 · ETOPO 2022 · CUDEM MA_NH_ME tiles (coast.noaa.gov/htdata/raster2/elevation/NCEI_ninth_Topobathy_2014_8483).

---

## 12. Air, light, and the one-substrate idea (added M6c, 2026-08-28)

M6c put the first tenant into a **3-D reserved-resource bank** (TileAtlas3D: GFS isobaric cloud
fraction → density volume; NULL tile = clear air), which makes the air/water question concrete:
the managers ARE already shared. What §4's grade-signature machinery adds next:

- **The air is a Cl(3) field like the water is a Cl(2) one.** The M3 velocity-gradient pass
  generalizes verbatim: 3-D wind (GFS U/V/W on isobars) → div (scalar bank) + strain +
  **vorticity bivector volume bank**; helicity ⟨u∧ω⟩ is a pseudoscalar density and is literally
  the supercell-hunting quantity (GFS ships updraft helicity as a 2-D proxy — we can VOLUME it).
  One Cayley-signature system, 2-D and 3-D banks, water and air: the unification is residency
  policy + grade bookkeeping, not new physics code.

- **Novel-GA candidate for atmospherics: polarized light transport as rotors.** Rayleigh
  scattering strongly polarizes the sky; the Stokes formalism that tracks this lives on the
  Poincaré sphere, and the non-depolarizing Mueller matrices form the Lorentz group — i.e.
  polarized radiance is a paravector in the algebra of physical space (APS, Baylis) and
  scattering events are ROTORS acting on it. A real-time sky that carries S = (I, Q, U, V) as a
  multivector and composes scattering as rotor products would render polarization-correct skies
  (and a polarizing-filter camera toy over the water — glare that rotates away). We have found
  no real-time renderer that does this natively: it is an honest AGACSE/SIGGRAPH-Talk-sized
  claim, and it shares blades with the M8 `F = E + IB` layer.

- **The one-substrate / graph-of-projections idea** (the "rhetoric" — it is useful, and it has
  algebra): one underlying model — fields in grade banks over manifolds, plus entities — and
  every VIEW is an edge labelled by a versor chain plus a grade projection plus a residency
  policy. Concretely, already in the tree: the globe↔estuary handoff is a rigid versor (tangent
  frame); the camera rails are motor interpolation; the quad-sphere face maps and equirect are
  (piecewise) conformal maps, and CGA (Cl(4,1)) is the algebra where sphere↔plane projections
  are native versors (stereographic projection is a rotor there — the natural home for "project
  the globe as a chart / a map / a windshield"); choosing ⟨·⟩₀ vs ⟨·⟩₁ vs ⟨·⟩₂ of one field IS
  choosing the layer (density map / current map / eddy map); and the atlas's signature bits are
  the sparse adjacency of that graph. So: sky, land, water, unit overlays, map projections,
  residency queries — one substrate, many (algebra, grade, versor) projections. M8's writeup
  should present the atlas this way.

- **RTX note (user question, answered with measurements):** the cloud march is already a ray
  cast through the sparse volume — tiled-resource page tables make NULL samples nearly free,
  and the whole globe frame with the 14-step march is ~0.3 ms, so RT cores have nothing to save
  yet. The genuine DXR fit arrives with NEAR-FIELD volumes (the estuary sky: sub-km tiles, 64+
  steps, long mostly-empty rays): build a BLAS of axis-aligned boxes FROM THE RESIDENT-TILE
  LIST and let DXR 1.1 inline `RayQuery` (usable in the existing pixel shaders, no RT pipeline)
  hand back cloud intervals to march — hardware empty-space skipping where **the residency
  manager and the acceleration structure are the same object**. Slated as an M7 experiment with
  a PIX A/B.

**Additional refs:** Baylis, *Electrodynamics: A Modern Geometric Approach* (APS, polarization
as paravectors) · Han/Kim/Noz, the Lorentz group in polarization optics (J. Opt. Soc. Am.) ·
Dorst/Fontijne/Mann, *GA for Computer Science* (CGA versors, stereographic maps) · NanoVDB +
OptiX sparse-volume ray tracing (NVIDIA) as prior art for tile-BVH volume traversal.

## 13. The layer compositor (added M6i, 2026-08-29) — "the logical evolution of the mars sample"

The problem it retired: every data source used to reach the renderer through its OWN path — the
global Google cube had one shader block, the Mercator detail window another, ETOPO a third, the
NE 15s ring a fourth — each with its own uv math, feathering and blend weights evaluated per
pixel per frame, and two layers (globe, terrain) re-deriving that math independently. That is
how they came to disagree about what the Earth looks like (the M6h glitch report), and why a
frame-handedness bug could hide for three milestones (see the postmortem below).

**The architecture (user-specified, implemented in `src/compose/`):**

1. **Schema registry** (`SourceInfo`): every source declares its name, structure
   ("mercator-tile-tree jpeg 256px", "equirect-grid int16", "geotiff-window float32"), its
   native **CRS** (the alignment contract — GeoTIFFs carry theirs, Google is EPSG:3857, CUDEM
   is a local-tangent frame anchored at ACT0816), its honest cm/px, and its coverage box.
   Logged as a table at startup. Every `Sample()` resolves its CRS to WGS84 lat/lon — the
   exchange frame — so two sources can only disagree by being wrong, not by speaking different
   coordinates.

2. **Channels = ordered layer stacks, per planet, per meaning.** `earth.color` = [google],
   `earth.height` = [etopo ← ne15s ← cudem], `mars.height` = [mola]. Upper layers paint over
   lower with per-texel WEIGHTS (0..1, edge-feathered at paint time — "bathymetry overwrites
   imagery softly"). Compositing is per PIXEL, never per tile: a texel a source does not cover
   keeps what is beneath it, and the realization's alpha rides into the shader so even a
   transient paint hole shows the layer below instead of black.

3. **Realizations = composed quadtrees.** A channel is realized as tile pyramids: a 16k cube
   for the planet (color RGBA8 128² tiles, height R16F 256×128 tiles), plus Mercator-aligned
   z14 WINDOWS where a region needs depth the cube cannot carry — and color and height windows
   share ONE window frame, so their texels describe the same ground by construction. The
   provider fn handed to the residency manager IS the paint: it runs on the worker threads,
   walks the stack per texel, and caches every finished 64KB tile to
   `cache/composed/<channel>/<realization>_v<stackhash>/` — raw tiles laid out for CopyTiles,
   i.e. the DirectStorage-ready folder. Paint once per machine per stack version; a second run
   streams composed tiles off disk with zero HTTP, zero reprojection (measured: first warm
   1367 paints / 0 fetches from the raw-tile cache; second warm 0 paints / 1367 cache hits).
   Changing a stack changes its hash and simply starts a fresh folder — stale tiles cannot be
   served.

4. **One render path** (`shaders/Compose.hlsli` + `GA_COMPOSED_CB_ROWS`): the renderer knows
   CHANNELS — earth color is earth color, earth height is earth height. Globe, terrain, and
   sea embed the same 9 CB rows, filled by `FillComposedCb` alone, and call the same
   `ComposedColor` / `ComposedHeight` / `ComposedIsLand` functions — the class of bug where two
   layers disagree about the planet's surface is structurally gone. The LOD is CONTINUOUS and
   allowed negative: the cube clamps at its mip 0 and the window realization picks up exactly
   where the cube runs out (lod −6 = z14 texels) — the climb-the-quadtree rule, in the sampler,
   with no branch. What this deleted from render time: NeWeight/ReliefBlended (the equirect/NE
   blend), the inline detail-window block, the committed 89 MB equirect relief + NE textures,
   the Mars-vs-Earth relief branch (both planets now displace from composed height cubes), and
   the M6b beacon.

5. **Classification by survey** (user call: "get a stencil/shp of ocean water as the default
   fallback"): GSHHG full-res shorelines + WDBII rivers (one 113 MB cached download) are the
   land/sea AUTHORITY. The harvester parity-fills their closed polygons into land masks (window
   4096² in the shared Mercator frame + global equirect); `ComposedIsLand` consults the mask
   first and lets fine height data refine only the intertidal margin; the sea's `BedAt` uses
   the composed height channel outside the surveyed CUDEM window (the "pretend 30 m ocean" is
   gone — waves feel the real shelf) and discards per pixel where the survey says land.

6. **Vector-first GIS** (user call): the polylines stay resident as the authority; the masks
   are merely their raster realization. `GisLayer`/`GisVec.hlsl` renders the SAME polylines as
   line geometry — raw lon/lat pairs projected in-shader through the shared rows — crisp at
   every zoom (--stencil). The GA thesis fits here: spherical polygons are chains of great
   arcs; point-in-polygon is winding/incidence (meets and joins); CGA versors are the natural
   home for projection chains. Vector/mesh GIS and raster GIS as two realizations of one
   source is exactly §12's graph-of-projections, made concrete.

**The debug stencil doctrine:** `--stencil` overlays surveyed truth on rendered claims — green
survey coast + magenta rivers (vectors), red composed-height zero contour (fwidth-thin), blue
window frame, white graticule. Where green disagrees with the photo the color channel is
misregistered; where red disagrees with green the height stack is; the survey settles the
argument, so two of our own layers can never again vouch for each other.

**Postmortem — the M6g reflection (found by the stencil, 2026-08-29):** the one-world frame
computed `north0 = up × east`. In OUR planet frame (x at 0N0E, **y at the pole**, z at 90E — an
odd permutation of ECEF) that identity flips sign: the result was SOUTH, the planet→tangent map
had determinant −1, and every lat/lon-registered layer (imagery, GIS, composed heights, the
foundation-sink window, MOLA vs the sample's diffuse on Mars) rendered N/S-MIRRORED about the
anchor latitude relative to every world-metre layer (terrain, sea, SWE). Invisible while those
families never shared a pixel; the moment the terrain wore imagery, the user's screenshots
showed rivers mirroring themselves. Fix: `north0 = east × up`, plus a determinant guard at
startup so a reflection can never impersonate a rotation again. Lesson recorded: **when a frame
is built from crosses, assert its determinant** — a versor sanity check PGA would have given us
for free had the basis been constructed as a motor.

**Deferred / next (see §14 for what M6j closed):**
- **DirectStorage** for the composed-tile folder (the layout is already right).
- **water / air channels** through the same registry (SWE eta and the cloud bank become
  channel realizations; sudden weather updates = a new stack version = a fresh cache folder).
- **Octrees**: the 3-D banks join the compositor the way the quadtrees did.
- **Shoreline upgrade path**: GSHHG f (~100 m fidelity) → NOAA CUSP where finer survey truth
  is wanted; the registry makes that a one-line stack swap.

## 14. M6j (2026-08-29): trust, utility, plugins — and the unified mesh-shader surface

The user's directive: clear the backlogs FIRST (skipping prep states has cost debugging loops
before), make the manager trustworthy AND useful enough to serve as the data interface — for
exports, for physics, for plugins — then build the unification on it.

**Backlog triage (every item accounted for):**
- DONE **compositor selftest gate** (`RunComposeSelfTest`, in --selftest beside pga/tile/atlas):
  paint order, per-pixel weights, alpha, transient-never-cached, cache byte-identity, cube AND
  window addressing vs closed forms, stack-hash isolation.
- DONE **--export**: any composed channel, pulled through the exact provider path the renderer
  streams, as .png / .raw / .obj (`--export earth.height.window:4 out.obj` hands you the same
  tiles the mesh shaders eat as a 1 M-vert inspectable mesh; a 4096² window PNG of the whole
  NH-coast-to-Boston footprint exports offline from a warmed cache). Seed of the manager-as-MCP
  idea.
- DONE **the color "grading" question — the user was right to challenge it.** Measured with
  ffmpeg straight off the cached JPEGs (no engine code in the path): same-footprint luma across
  z12→z14 differs only ~2.5%. The loud banding had been OUR bugs (terrain lighting fork, VS
  classification smear — both fixed in M6i). The real conversion sin found instead: sRGB JPEG
  bytes stored in linear UNORM + a hand-tuned img*img*1.2 curve hack. Now: *_SRGB formats
  decode in HARDWARE (Google RGBA8 + Mars BC1), the curve hack is deleted, one linear exposure
  constant remains. Grade normalization survives only as a clamped per-tile trim against an
  absolute z10 reference (BeginTile/PaintCtx — sources stay stateless across worker threads).
- DONE Antarctica mask levels (5=ice front→land, 6=grounding→dropped).
- DONE Mars face-orientation calibration (was the M6g reflection), committed-relief retirement,
  anisotropic residency-map fix (all landed M6i).
- DEFERRED, explicitly: **Flather west boundary + per-axis metric** (top SOLVER leg — physics
  formulation, orthogonal to the manager; unchanged design in §... the M6d diagnosis),
  z15/16 imagery rings (budget policy), WinPixEventRuntime, ephemeris sun, DirectStorage,
  CUSP, octrees, wavelets/bore/M7. M6f's 594-vs-334 fetch question is OBSOLETE (that code
  path no longer exists; the composed cache changed the flow).

**The Exchange (`src/compose/Exchange.h`) — the plugin bus.** Named, versioned GPU buffer
channels with DECLARED layouts: stride + a grade-signature byte (the atlas's Cl bits) + a
human semantic. Producers Publish, consumers Query BY NAME — the buffer sibling of "earth
color is earth color". Two plugins prove it end to end:
- the GIS vectors publish as `gis.coast.ne` / `gis.rivers.ne` / `gis.coast.global`
  (lonlat line-lists) and GisLayer renders whatever the channels hold;
- `markers.stations`: the CPU builds one **PGA motor** per tide station (Pga.h — placement =
  data organization, the CPU's job), publishes {motor dq8, scale, color} as a GA product
  buffer, and Markers.hlsl applies the SAME sandwich on the GPU (GA.hlsli MotorPoint/MotorDir,
  line-for-line translations of the selftest-pinned CPU formulas). CPU-GA organizes → Exchange
  carries → GPU-GA renders: exactly the socket the user's physics plugins will drive with
  mesh/vertex buffers inside larger GA product buffers.

**The unified mesh-shader surface (`GlobeMesh.hlsl`, tier confirmed).** One pipeline, orbit to
helm, both planets; TerrainLayer keeps the physics heightfield but stops rendering. The CPU
CDLOD walk (still the residency feedback) deepens to level 16 (4.75 m vertices) and emits one
record per 8×8-cell meshlet; DispatchMesh amplifies them from the composed channels at
CONTINUOUS lod. **How the float wall fell:** fine meshlets (arc ≤ 650 m) carry a double-
precision camera-relative anchor + the position Jacobian; vertices reconstruct as
anchor + J·Δuv with Δuv from small integer cell offsets — no 6.4e6-magnitude float subtraction,
millimetre-stable at walking height (linearization error at 650 m span ≈ 3 cm). Coarse
meshlets keep the classic exact-float path (their ≥2 km viewing distance hides the ~0.6 m dir
jitter). The close-up material model (wet sand at the LIVE waterline, riprap by slope) moved
into the planet shader, fed by `waterNavd` per frame.

**Two hard-won rules from the bring-up (stencil-diagnosed, baseline-compared):**
1. *Water-classified geometry rides ~2 m BELOW the live waterline, never at the geoid.* At low
   tide the geoid stands PROUD of the real sea; without the (now unnecessary) foundation sink
   the globe's water plane buried the FFT surface. Classification itself follows
   `ComposedIsLand(dir, h, waterLevel)`: survey mask far afield, fine heights vs the LIVE tide
   inside the window (flats emerge and drown with the actual tide — no static polygon knows
   that), height sign on Mars.
2. *Vertex heights near the camera sample the FINEST RESIDENT data — the same the per-pixel
   classifier reads* — or geometry and classification disagree into vertex-land/pixel-water
   plates; and the CDLOD morph must blend the height SOURCE along with the grid.
   (Also learned: compare against the --no-ms baseline before chasing "artifacts" — the glassy
   sheet over the bar is the sea's honest dry-guard, present in both paths.)

**Perf:** helm 5.5 ms (vs 1.8 ms classic — no meshlet culling yet), 8 km 2.7 ms, orbit 1.2 ms,
Mars 0.2 ms. Next optimization: an amplification-shader pass for per-meshlet frustum/backface
culling, and meshlet count tuning. Then: physics reading the height WINDOW the renderer
streams (concurrent read/write), Flather, and the water/air channels.

## 15. M6k (2026-08-29): the presentation purge — the user's three instincts, all confirmed

The zoom video showed (a) the whole Earth REORIENTING in one frame at 0:02, and (b) a washed-
out look at low altitude. The user asked whether the atmospherics were logically needed or
"rhetoric you've trained on," said the insets should behave "like layering images in paint,"
and told us to take the water out of the texture loop. Every one of those calls was right.

**The 0:02 reorientation (fixed).** `ViewRelative`'s gravity-up had a CLIFF: looking straight
at the planet centre, upHint is parallel to the view axis and a 0.999 fallback picked a world-
up roll; the first frame the plunge drifted off-centre it SNAPPED to gravity-roll. And the sky
shell + the scene-constant rays were rebuilt from `Camera::Right()` — the HORIZONTAL right —
so every ray-reconstructing pass was roll-blind and the atmosphere detached from the surface
under fast motion. Fix: ONE `Camera::ViewBasis()` (used by the view matrix, the scene
constants, and the shell — they cannot disagree now), with the degeneracy BLENDED smoothly
toward flat-north projected ⊥ view. Roll transitions now spread across the descent.

**The wash (fixed, three layers deep — the user called every one).**
1. The rim/limb tint multiplied into every oblique view INSIDE the atmosphere → now fades in
   above 60 km (a from-space effect; near haze belongs to AerialPerspective alone).
2. The 1.35 "exposure compensation" for the sRGB switch matched the old curve at mid-tones but
   pushed bright land cover over the shoulder → REMOVED, pixels ship as decoded.
3. **ACES itself** — the real chalk-maker. Narkowicz-ACES boosts mid-tones (0.39→0.52 linear)
   and desaturates; applied to imagery Google already tone-mapped, it bleached the planet.
   Replaced with identity-below-the-knee (0.85) + a smooth exponential shoulder: DATA passes
   untouched; only genuine HDR (sun glint, the disc) rolls off. The cross-zoom "grade gain"
   experiment was also removed the same hour (v3 cache tag): raw tiles proved z11–z14 captures
   agree over the estuary; the gain was bleaching seasonal land cover toward the z10 capture.
   Along the way the M6h bilinear residency-map ramp was found UNSOUND (it interpolates below
   the locally-resident mip → Tier-2 null reads dilute the window's alpha) and replaced with
   conservative gather-max reads everywhere.

**The texture-work lens (`--albedo`).** Raw composed color at every altitude — no lighting, no
atmosphere, no materials, no clouds, no SEA (per the user), stencil available. This is the
loop texture streaming is judged in: one code path, no altitude behavior, literally layered
images. The remaining altitude-dependent presentation flips are inventoried and gated OUT of
this lens: sea <60 km, sky dome ↔ limb shell at 9 km, materials <2.7 km, rim >60 km. The
composed-channel path itself has NO altitude flips — one sampler, continuous mips.

**Opened by this purge (queued):** the LIT look must be retuned under the honest tonemap — the
sea/sky constants were tuned against ACES's compression and the helm sea currently reads
milky. That is a lighting-constant pass (sea reflectance, SkyRadiance levels, sun scale), not
a texture problem; the water stays out of the texture loop until it lands. Also: two runs
exited 255 with garbage readbacks (uninitialized dump = the copy never landed) that did not
reproduce under the debug layer or in repeats — watch item, suspected teardown/TDR race.

## 16a. M6n (2026-08-29): the flats speckle — three layers deep, all named

The ragged land/water patches over the tidal flats turned out to be THREE stacked causes, and
each fix is a principle worth keeping:

1. **A binary classifier asking an analog question.** Flats sit within centimetres of the
   tide line; `hp > water + 0.05` flipped a bit per residency change and cut tile-shaped
   patches. Now `ComposedLandness` returns a 0..1 through a ~40 cm shore band (15 cm below
   the waterline to 25 cm above) inside the window: shading mixes (albedo, glint dies as the
   flat emerges, the land normal takes over), the near-material weight rides it, and the MESH
   displacement lerps between the drowned plane and true height — shorelines slide, never
   pop. Outside the window the survey mask stays binary (a polygon IS a bit); the sea's
   discard keeps the boolean view.
2. **A residency-dependent classifier input.** Neighbouring tiles streaming at different
   height-data levels disagreed by more than the band and cut seams ALONG TILE EDGES. The
   land/water QUESTION now reads a fixed ~38 m level (window mip 2, fully warmed) —
   `ComposedHeight(up, max(lod, -4))` — while fine data keeps driving shading detail.
3. **Presentation painting where physics said "calm".** What remained was not classification
   at all: the sea's shallow sheet showed an ANALYTIC sand bed and a depth-triggered 40%
   shore-foam wash. Now thin water goes transparent to the COMPOSED IMAGERY bed
   (`ComposedColor` at the bed point — flooded marsh shows brown marsh through the tide), and
   the depth foam term is gated by BREAKING ENERGY (`i.brk`): foam is surf, and sheltered
   creeks stay glassy. The helm's bar now carries one energy-driven breaking patch instead of
   a painted band.

Diagnosis pattern that worked (twice now): render the SAME view repeatedly and vary one input
— if the artifact does not move, it is not that input. The pale mid-tide marsh that remains
is honest: Google's own photo of a flooded sound is the same sky-grey.

## 15a. THE DOCTRINE (2026-08-29, the fork before the water work): see ATLAS.md

The texture-handling writeup graduated into its own document — **[ATLAS.md](ATLAS.md)** —
and it is doctrine, not description: rasters are PLANES (frame + metric + lattice + value
fiber); the lingua franca is three tiers (the WGS84 exchange frame, the multivector value
algebra, the resolution ladder), never one master grid; the rung rule answers granularity
(a 10 cm and a 7 cm source realize at 5 cm — the first rung strictly finer than the finest
source, so undersampling can only happen in the coarsening direction where box means make
it a measurement); conservative vs perceptual resampling is law; and the monastery pattern
is the architecture vocabulary (monasteries = channel families that never import each
other's internals; the bible = the shared algebra; the Vatican = any client where they
convene; cardinals = the kernels that paint). Water, air, and Maxwell enter through the
same five steps (ATLAS §9). Decoupling is staged toward AtlasD, the background sim service
the renderer peeks into (§8). The Scriptorium (tools/Scriptorium, .NET 10 MCP + SQLite)
keeps the growing repo navigable: symbols, harvester registry, provenance, monastery
registry — registered in .mcp.json.

## 16. M6l (2026-08-29): the painting verified — a plane-flown ortho through the registry

The user's test: bring in an INDEPENDENT high-res aerial of the Merrimack inlet and see if the
painting aligns "accounting for meters per pixel and projections." Source found: **MassGIS
2023 statewide orthos** — 15 cm, leaf-off, plane-flown, 1500 m USNG-named tiles, lossless JP2,
**EPSG:6348 (NAD83(2011)/UTM 19N)** straight from the GeoJP2 header. Two tiles cover the mouth
(19TCH510415/510400; MassGIS stops at the shoreline column, so the jetty TIPS lie beyond
coverage). `harvest_aerial.py` decodes via Pillow/OpenJPEG (ffmpeg's j2k chokes on the TLM
markers), builds an 8-level box-filtered mip chain per tile, and writes raw RGB + a
georeferencing json (the index shapefile's LCC bboxes located the tiles; the .prj is
EPSG:26986 — parsed by hand, stdlib only).

The contract, in code: `src/compose/Projections.h` — EXACT Snyder transverse-Mercator and
Lambert-conformal-conic forwards (GRS80), **pinned in --selftest against independently
computed references at the ACT0816 anchor** (UTM 352034.1/4742229.8; LCC 256429.4/952196.8,
±0.5 m). `AerialOrthoSource` declares its native CRS in the registry, memory-maps the mip
chains, resolves WGS84→UTM per sample, and picks the mip whose GROUND m/px (Mercator-
equatorial × cos lat) matches the paint footprint; 25 m feather against the coverage UNION
(interior tile seams stay seamless — one flight). NAD83↔WGS84 (~1 m) left uncorrected and
DOCUMENTED — it is part of what the comparison measures.

Verification: `--export earth.color.inlet:2` (a z19 export-only realization, ~22 cm ground)
painted the ortho over Google — the north jetty, roads and shoreline run CONTINUOUSLY across
the feathered boundary at 0.9 m/px (subpixel registration), the footprint's slight rotation is
honest UTM grid convergence vs Mercator north, and the tone step is the spring-2023 flight vs
Google's summer mosaic (a capture difference, deliberately NOT color-matched after the M6k
lesson). The live path streams the same layer through the z14 window (--albedo at 2.5 km).
Next for this layer: a streamed z18/z19 window realization so the 15 cm detail reaches the
renderer, not just the exporter; and the user's opt-in per-layer color-adjust ops.

## 17. M6q (2026-08-29): seed the law from surveys — and KML round-trips

The user caught me hand-digitizing jetty polygons off my own renders (twice wrong — the
second attempt confidently traced the ebb-shoal classification smear as the "south jetty";
their red-line annotation was the correction) and asked the right question: "are you making
up your own GIS? Why not just overlay a coastal one?" One cached Overpass fetch
(`data/gis/osm_structures.json`, ODbL) brought every charted structure at the mouth as
surveyed `man_made=breakwater|groyne` footprint OUTLINES — North/South Jetty, spur, Old
South Jetty, a dozen groins, the Low-crested Dike. `edits.geojson` is seeded from those
true footprints; `structures.kml` exports the inventory for Google Earth; and the harvester
now INGESTS any KML dropped into `data/gis/` or `cache/vectors/` (Placemark walk → vpack
layer with wedge importance), so hand-drawn overlays enter the survey order like any
shapefile. The structures draw amber on the GIS vector overlay — lines live on their OWN
layer (user rule), never painted into surface textures; masks are the one sanctioned raster
use. The three survey-law files are the one versioned exception to the data/ gitignore.

## 18. M6r (2026-08-29): the water speaks — the prism term, Flather, and the per-axis metric

Back to the water, at the top deferred solver item — and the validation harness turned a
boundary-condition chore into the biggest physics fix since the signed-staggered rewrite.
Method: pin a baseline (14 h `--swe-cycle` vs the ACT0816 prediction, scored for r, phase,
amplitude, dominance), change one thing, re-run.

**The per-axis metric (measured, not assumed).** The CUDEM grid is EQUIANGULAR: dlon = dlat
= 0.44″, so texels are 10.08 m east × 13.65 m north — and the solver ran both axes on dx.
Every north-south face length, cell volume, and derived v carried a 26–35% metric error.
Now (faceLen, span) per axis, volumes dx·dy, CFL on the min axis.

**Flather west boundary.** The 24-texel Dirichlet relax strip was a soft wall — transients
reflected, and the basin's phase carried the echo. Now: ONE exterior column pinned to the
station river tide (data), its east face radiating at the gravity-wave speed
(u_b = u_ext + √(g/h)(η_ext − η_int)). First cut with u_ext = 0 LOST amplitude — radiation
alone cannot carry a prescribed prism; it throttles behind the standing Δη it needs. u_ext
is the prescribed transport (USGS river Q minus the upriver prism demand, A_up·dη/dt with
A_up ≈ 3.9 km² for the reach to the head of tide; NHD-integrated area is the named
refinement) over the LIVE wet section (bed profile cached at init).

**THE PRISM TERM — the real find.** A continuity audit (net east transport summed down full
N-S sections, now printed by `--swe-uv`) showed ~520 m³/s flat along the entire channel at
peak flood with only 43 m³/s of storage flux across the whole harbor: the basin was filling
BY CONSTRUCTION. The eta bank stores deviation from a MOVING plane — when gTideNavd rose,
every cell's absolute surface rose with it, no transport required; the gap only ever carried
the deviation dynamics, and the ×5 render gain had been compensating for missing physics.
The fix is one line with a section of comment: `dEta -= gTideRate·gDt` — every resident
cell books the plane's rise as DEBT, and the debt's gradient against the free boundaries
(sponge = the analytic ocean, pinned west column = the river data) IS the flood current;
the ebb is the debt paid back. Wet/dry becomes hydrodynamic for free: a flat's surface no
longer rides the plane — it waits for the water to arrive.

**The survey law reaches the solver.** `BathyModel::ApplyMaskEdits` rasterizes the
mask=land edit polygons (the OSM jetty footprints, M6q) into the physics bathymetry as
+2.5 m NAVD riprap walls (254 cells) — the 3 m CUDEM knows the jetties but 13.7 m box means
smear them into leaky sills, and the jet crossed the crest line instead of concentrating.
CPU-side masking, per the vector doctrine.

**Validation, baseline → final** (14 h vs ACT0816, same clock, same scorer):
r 0.72 → **0.95** (best-lag 0.92 @ −88 min → **0.96 @ −24 min**), transect-mean amplitude
ratio 0.22 → **0.60** (throat CORE 0.93 vs ACT 1.06 — ACT predicts the jet core), flood
dominance 0.72 (wrong side) → **1.06** (ACT 1.48), sponge residual ≤ 0.2 cm, basin lag 0,
attenuation 1.00. Continuity now reads like an estuary: −400 m³/s entering at the west
boundary growing section-by-section to −4600 at the gap. **The ×5 sweGain is RETIRED**
(default 1.0): the physics carries the magnitude. Spinup default 0.25 → 1 h (the debt field
needs the history; ~1 s).

Named next for the water: flood dominance (ours 1.06 vs 1.48 — likely bar/ebb-shoal
geometry + the missing overtide steepening), subgrid channel conveyance (the M6d tracked
item, still real upriver), NHD-integrated upriver prism area, the sound's real Ipswich
entrance, and eta-aware shoreline classification (the wet line now truly lags the analytic
tide on the flats — the renderer should read the solved eta, which is the water/air-channel
Exchange step of ATLAS §7).

## 19. M6t (2026-08-29): one water, every altitude — grade shedding with conservation

The user watched the flood-ride video and called the piecewise water: "shouldn't it be seen
from space like the textures should if we are bleeding through LODs correctly? … I was
hoping we could find something that works well from either high or low altitudes using
Geometric Algebra." Diagnosis confirmed: THREE disagreements at the seam. The sea layer
faded each FFT cascade by hand-tuned camera DISTANCE (fadeD 2600/520 m) and the energy went
NOWHERE — waves just vanished; the globe carried the whole spectrum as Cox-Munk slope
variance; and the sea's only specular was a mirror-Fresnel sky (no sun lobe at all — the
"glossiness"). Two halves of one BRDF that never talked, swapping at the mode handoff.

**The GA statement (GAMEPLAN §107.7, now real): grade shedding with conservation.** Each
cascade band is a rotor bank in Cl(2)+ — Tessendorf's e^{iωt} IS the rotor. When a pixel's
ground FOOTPRINT (dist × pixel angle, zoom- and resolution-aware) passes a band's Nyquist,
the rotor's phase becomes meaningless but its magnitude does not: the band sheds from
resolved geometry (grade-resolved) to the BRDF's slope variance σ² (grade-0 statistics).
Energy changes grade; it is never deleted. The bookkeeping TELESCOPES: per-band
mean-square slope integrated on CPU from the same model spectrum the plot draws
(∫k²S df, banded by the synthesis cuts, exaggeration baked), plus a sub-resolved floor
calibrated as CoxMunk(wind) − Σ bands — so at full shed the sum is EXACTLY the globe's
σ², and the two water descriptions become the same pixel. At the helm the floor alone
remains — a tight glitter riding resolved wave faces (the missing sun glint, now present;
the sky mirror went DISCLESS so the sun is counted once). On a calm 2 m/s day the physics
says it plainly: the long swell's slope variance is ~1e-4 — nearly ALL glitter lives in
the sub-resolved floor, at every altitude.

Storm aerials then found three real artifacts the old distance-fades had been hiding:
the globe mesh's water plane at −2 m POKING THROUGH shoal-amplified troughs (facet-shaped
patches of the wrong water — sunk to −8 m; the globe sampling the displacement bank itself
is the M7 unification), reflections dipping below the horizon on steep faces (clamped to
horizon sky — the abs() mirror was tried first and traded black for zenith teal), and
sampled slopes past any physical wave face (capped at 1.1 — steeper IS breaking, which is
M7's wavelets). Churn's froth fade became footprint-based like everything else.

Named next: Hs-whitening parity in the sea's far field (storm color convergence), the
globe mesh sampling the FFT bank directly (one geometry path as well as one BRDF), and
per-band DIRECTIONAL σ² (Cox-Munk's up/crosswind anisotropy from the same integrals).

**M6u addendum (same day): the other half of one water.** The user's frame pair (0:16 vs
0:17 of the flood ride) showed the handoff still SNAPPING -- M6t had unified the energy,
but the sea grid replaced the globe's whole ocean with a different COLOR MODEL (scattering
asymptote under a hazed sky mirror vs shelf-tinted albedo + composed imagery, unhazed).
Convergence: above the estuary's own altitudes (kFar over 700-2800 m camera height) the
sea pixel evaluates THE GLOBE'S EXACT water formula -- same composed channels, same
ComposedHeightLod the globe would pick, same lighting constants, Hs whitening via the
retired fadeD slot -- while the sky mirror and the near-field haze fade out with it. At
the handoff band the two renderers emit the same pixel; the quadtree's promise (no LOD
ever pops) now holds for the water's color as well as its energy. The remaining single
owner-swap (globe mesh sampling the FFT bank, retiring the second geometry entirely)
stays the M7 unification.

## 20. M6v (2026-08-30): THE WATER ATLAS — heterogeneous water data through the one registry

The user's charge: confirm the texture treatment extends to water — "different LODs and
sparseness… not the same measurements in the deep ocean as the coasts… some inlets record
more parameters than others… GIS-encoded data… single point measurements from buoys…
easily queryable… aligned heights, geopositions, projections and altitudes… low-medium
quality default global data that the HQ New England enhances… easily reprojectable — print
maps onto paper without breaking the physics." Focus: Merrimack / Gloucester / Boston.
Plus a 200 GB big-data grant at D:\DataCache (static datasets = offline HQ insets).

**The fiber is the phasor.** water.tide.{M2,S2,N2,K1,O1} are FieldChannels in the SAME
compositor: a new two-component fiber (FieldSource / AddFieldChannel / WindowField RG16F
128² tiles, soak-rule cache identity, one paint loop). A constituent is a Cl(2)+ spinor
(re, im); h(x,t) = msl(x) + Σc Re[P_c(x)·e^{iωc(t−T0)}] — rotor application, the same
statement the wave bands made in M6t. Interpolation happens IN THE PLANE (amp/phase
bilinear collapses amplitude across a phase gradient; re/im cannot).

**The stack is the user's rule embodied** (ETOPO←NE15s←CUDEM, for water):
  L0 equilibrium tide (analytic, global, LOW — zero download, honest structure)
  L1 EOT20 (1/8°, global, MEDIUM — SEANOE 2 GB once into D:\DataCache, CC-BY 4.0,
     netCDF4→flat rg32; validated raw: M2 at buoy 44013 = 1.293 m @ 108° — textbook
     Gulf of Maine; one Fundy artifact cell clamped)
  L2 the NE station field (point harmonics, phasor IDW, ~2 km, feathered 20→90 km) — the
     survey WIDENED from 6 to 20 fitted stations (harvest_water.py discovers the region:
     Gloucester Harbor 8441841, Rockport, Annisquam, Essex, Beverly, Salem, Lynn, Boston
     Light, Nut Island, Scituate…, all M1-fit to sub-mm RMS), 229 current stations and 3
     buoys registered, 265 survey points folded into the vpack as a points layer.

**THE EPOCH LADDER** (the phase version of the M6d datum lesson): station fits carry
fit-epoch phases; global atlases carry Greenwich lags; mixing them raw smuggles a phase
cliff into the seam. Every global source is re-referenced per constituent by the
amplitude-weighted circular consensus of the stations it overlaps — no astronomy
transcribed; the stations are the epoch authority. Measured: the offshore seam steps
2.1 mm across the L2→L1 feather (raw mixing would step decimetres).

**Audited in --selftest (watertest):** Mercator round-trip of every station geoposition;
the datum ladder (MSL-in-NAVD within ±0.6 m at every linked station); the composed stack
reproduces all 20 station fits to 0.0 mm; seam continuity; the field soak contract
(a mid-Atlantic tile keeps only global sources); paint-vs-CPU-stack tile identity.

**THE REPROJECTION PROOF:** --water-map prints the same sources through a CUSTOM Lambert
conformal sheet built for the page (analytic inverse per pixel): M2 co-amplitude field,
GSHHG ink coastline, NE-15″ relief land, the whole station survey, the OSM jetties, a
graticule. No physics touched — a projection is just another realization. (Two dead ends
recorded: vpack coast parity fill is unsound — the polylines are CLIPPED, false closure
chords flipped the Atlantic to land; relief is the honest land authority at chart scale.)

Named next: the GLOBAL WEATHER/OCEAN SIM MANAGER consuming these channels (detail rising
on zoom — GoMOFS 700 m FVCOM fields as the Gulf's L1.5, current-ellipse constituent
channels from the 229 stations, buoy spectra as live transient sources); CUDEM land for
chart-scale coastlines; more EOT20 constituents (the zip is cached, conversion is free).

## 21. M6w (2026-08-30): THE ONE BED — bathymetry through the painted stack, for everyone

The user's charge before the physics manager: bathymetry must paint through LODs like the
textures AND serve the renderer and the weather/physics manager alike. The audit found the
bed had TWO owners drifting apart: the renderer read the composed height channel
(ETOPO←NE15″←CUDEM, cached tiles), the physics read a private flat CUDEM grid — and the
M6r jetty walls existed only on the physics side. Worse, a live STALE-CACHE bug: baking
walls into BathyModel changed the channel's source CONTENT without changing its cache
identity, so height tiles painted before M6r silently served the un-walled bed forever.

**Hand edits are a stack source now (EditsHeightSource).** mask=land polygons paint their
riprap crest INTO earth.height; mask=water dredges. The source's cache identity is the
geojson CONTENT hash — an operator edit repaints exactly the touched tiles at every rung
(the soak rule finally works FOR the operator), and the old pre-M6r caches become correct
again rather than stale (the raw CUDEM source reverts to un-walled). Pinned in composetest
(the edits-identity contract).

**The solver's bed realizes from the channel.** BathyModel::RealizeFromChannel +
Compositor::SampleHeightStack (the physics-facing twin of SampleFieldStack): the SWE grid
re-fills from the SAME stack the renderer's tiles paint from — CUDEM where it covers,
NE-15″/ETOPO beyond, edits on top. A bathymetry disagreement between solver, renderer, and
any future product is no longer expressible. main.cpp hoists the planets' CPU models + the
height channel ABOVE the solver; raw planes stay immutable sources (bathyRaw), the solver
grid is a realization (bathy).

**Physics blessed it:** the 14 h validation cycle IMPROVED — r 0.945→**0.959**, best-lag
−24→**+4 min**, flood dominance 1.06→**1.16** (ACT 1.48) — the honest NE-15″ shelf beyond
the CUDEM edges beats the old nodata walls. 395 K cells moved vs the raw grid: the edge
feathers + the edit structures, quantified in the startup log.

**The focus coast got its HQ insets:** harvest_bathy is windowed (--window merrimack|
capeann|boston; raw NCEI tiles cached on D:\DataCache); Cape Ann (3159×1782) and Boston
Harbor (2591×1862) CUDEM planes at 13.7 m now stack in earth.height as
noaa.cudem.capeann/.boston — Gloucester and Boston bathymetry is channel-truth for the
manager, six layers total with the edits on top.

Named next: solver windows over the new insets (a Boston Harbor SWE is now a georef away),
CUDEM land for chart-scale coastlines in --water-map, DirectStorage for the composed
folder, and THE GLOBAL WEATHER/OCEAN SIM MANAGER — its bathymetry API already exists
(SampleHeightStack at any rung, RealizeFromChannel for any lattice).

## 22. M6x (2026-08-30): THE GLOBAL WEATHER/PHYSICS MANAGER — residency rises on zoom

The convergence of M6v/M6w: `src/sim/WeatherManager` — ONE query surface over every water
and near-surface product. Query(lat, lon, t, groundResM) → { level, current, Hs/Tp/dir,
wind, bed, depth }, each component answered by the FINEST RESIDENT RUNG that covers the
point, each carrying PROVENANCE (heterogeneity is queryable, not hidden). Stateless global
fields carry the world — the tide atlas' constituent rotors, GFS 10 m wind vectors,
GFS-Wave global Hs, the Gulf point partitions, GoMOFS surface currents, the one composed
bed. STATEFUL solver windows are the sparse exception: the Merrimack SWE registers as an
EXTERNAL window (the render loop drives it; the manager mirrors its eta/uv banks to CPU at
2 s cadence); BOSTON HARBOR registers DORMANT and spins up when the camera (or a probe)
enters its footprint below 30 km — bed realized from the one channel on activation, solver
parameterized by SweConfig (spongeX0 in Mass Bay, west boundary = wall: the Charles is
dammed), 0.5 h of history, mirrors live. Detail rises on zoom because RESIDENCY rises on
zoom — the tiled-atlas statement, applied to physics.

Verified by --ocean-probe (the harness IS the zoom rule, headless):
  mid-Atlantic: bed −3374 m (ETOPO), Hs 1.50 (global grid), GFS wind, tide from the
  global constituent stack, currents honestly absent; Boston Harbor: the window ACTIVATES,
  level answers from swe.boston (gate vs the full-harmonic station: 110 mm), solved
  currents, bed −17.1 m from noaa.cudem.boston — the shipping channel. Station gates
  PASS (≤ 0.25 m; the ~120 mm baseline error is the honest 5-of-25-constituent
  truncation). Debug find: ReadProbes' padded eta wrap dims were cosmetic lies (the
  reserved texture's desc is the LOGICAL grid); a mirror loop that trusted them ran off
  the readback buffer — ReadFields now uses nx/ny truth.

**THE RENDER PRODUCT (the user's contract, M7):** the manager's output for rendering is
ONE 2D TILED RESOURCE of wave vertexes with parameter side-buffers (level, current, foam,
sigma^2, provenance flags) — no piecewise water anywhere; LOD handled by tiled residency /
amplification / tessellation. The bank composes exactly what the manager already owns:
the tide rotors, the solver eta/flux banks (already TileAtlas2D), the FFT cascade
displacement, the one bed. The M6t/M6u BRDF+color unification was the shading half; this
bank is the geometry half, and it retires SeaLayer's private grid the way GlobeMesh
retired TerrainLayer's renderer.

Named next: the wave-vertex bank itself (M7); GoMOFS 3D fields + ESTOFS water levels as
the Gulf's mid rungs; a Boston current-station prediction harvest for a proper currents
gate; async window spin-up (the activation hitch is ~5 s, logged); windows as Exchange
channels (water.eta.<window> for plugins).

## 23. M7 (2026-08-30): THE WAVE VERTEX BANK — one tiled water, and the state-diagram architecture

**The user's render contract, delivered:** water geometry as ONE 2D tiled resource. Two
RGBA16F TileAtlas2D banks — disp = (dx, dy, dz, foam), param = (levelNavd, sigma^2, u, v) —
laid out as SIX camera-anchored mip rings (4.8 m texels at the camera to 154 m at 79 km,
each ring 4x4 logical tiles of 128^2). Every wet tile recomposes EVERY FRAME by one
list-driven kernel (WaterBank.hlsl) from everything the weather manager federates: the FFT
cascades folded by each ring's own texel footprint (the M6t grade shedding — a coarse ring
carries as sigma^2 what a fine ring carries as vertexes, so ring handovers conserve energy
by construction), the SWE banks where resident (eta + solved currents), and per-tile corner
params the CPU samples from the atlas stacks (tide level by constituent rotors, local Hs
from the global wave grid so mid-ocean seas track their OWN storm, the one bed). LAND TILES
ARE NULL — unmapped in the tiled resource, never dispatched (74/96 resident at the inlet).
Re-anchoring is free because content is stateless-recomputed. ~0.5 ms/frame.

**--one-water:** the SeaLayer grid RETIRES (drawEnabled=false — the TerrainLayer pattern;
its FFT/SWE/churn compute keeps running as the bank's source) and the globe's meshlets
displace water vertexes from the bank (level + folded waves), with per-pixel normals
finite-differenced from the same texels — the glint rides real wave faces, the sigma^2 and
foam come from the bank params. One geometry, one shading, no piecewise water, orbit to
helm. Verified: storm helm shows rolling swell with the glitter bending along wave
contours; calm helm subtle honest undulation; aerials seamless; all gates green; 4.8 ms.

**Bring-up traps (both stage-bisected):** static-sampler SampleLevel on bindless arrays
silently returns ZERO outside the pixel stage on this driver — the mesh stage AND the
compute kernel both hit it; every bank read is now manual-bilinear Load (stage-proof, and
the vertex density matches the texel density anyway). And the M6x padded-vs-logical dims
trap struck again in the kernel's SWE sampling (normalize by PADDED dims against a LOGICAL
resource = shifted reads) — Loads with logical dims retired it.

**THE STATE-DIAGRAM ARCHITECTURE (the user's formalization, adopted as doctrine):** the
system is a state-transition diagram whose NODES are GA engines — the texture compositor,
the water atlas, the weather manager, the FFT, the SWE solvers, this bank's kernel — and
whose EDGES carry GEOMETRIC PRODUCTS: constituent phasors rotated by e^{iwt}, motor
sandwiches placing markers, wedge filters decimating surveys, the fold moving energy
between grades. The ACCEPTING STATE is a realization: the renderer's frame is one, the LCC
paper charts are others, --export's meshes a third. The wave vertex bank is the edge into
the render-accepting state; nothing downstream asks who computed what.

**The precision fork (user question, recorded):** our quadtrees realize into RESERVED
(tiled) Texture2Ds — hardware-filtered but capped at 16384^2 per resource, with global
float UVs that lose precision as virtual domains deepen (whence windows and re-anchoring).
The alternative for the cm rungs: a Texture2DArray of tile SLICES + an indirection table —
tile-local UVs keep full float precision at ANY depth (genuine cm pixels, no smoke and
mirrors), domains unbounded, cost = one indirection + 1-texel gutters for filtering. The
hybrid is the plan: tiled resources where hardware filtering over moderate domains wins
(cubes, z14 windows, this bank); array+indirection for z18+ (the MassGIS 15 cm streaming,
future cm bathymetry). Named M7b.

**M7a (2026-08-30): the derivative side-plane — the helm gets its sparkle back.** The
bank's ring texel caps what geometry can carry (4.8 m at the camera), but a helm-height
PIXEL footprints millimetres — the bands in between were falling into sigma^2 and the
glint trail smeared into milk. The fold now splits THREE ways per band: wRing (the tile's
fold, geometry in the bank), wPix (the same smoothstep on the pixel's own ground
footprint), and the difference wDetail = saturate(wPix - wRing) — bands the pixel resolves
but the ring cannot. The PS reads those straight from the FFT cascade DERIVATIVE textures
(full 256^2 resolution; PS-stage SampleLevel is the one place the driver's bindless
sampler path works, conveniently the one place we need it), scaled by a third bank plane
`detail = (hsScale, dry, -, -)` so the local sea state and the flats guard ride along, and
hands the same energy back out of sigma^2 (the M6t constants, hsScale^2-scaled, floor
kept). The three tiers TELESCOPE: at altitude wPix <= wRing and every term vanishes
untouched; at the helm the storm glint shatters into real glints (PNG entropy 2.6x at
identical framing — the decisive diff). Calm stays glassy and that is CORRECT: the ×8
probe proved the path fires (storm diff 370k pixels, calm 121) — a lone 0.5 m Gaussian
swell partition simply has no band-2 energy; sparkle is a property of the SPECTRUM, not a
decoration. All five gates green.

**M7c (2026-08-30): THE TWO RAYS — ray tracing against our own quadtrees.** The user
asked for ray/path tracing that shades well from space AND the helm. The answer is not a
BLAS: a water pixel is a Fresnel split between two rays, and both are answered by data
the atlas already realizes. The REFLECTED ray is the sandwich r = −n d n against the
analytic sky (SkyRadianceDirDiscless — the horizon now mirrors the sky, the term every
tuning pass before was missing). The REFRACTED ray bends by Snell — a rotor in the
incidence bivector (d ∧ n), the closed form in the shader IS R d ~R expanded — then
casts into the water and lands on the BED: two secant steps against ComposedHeight (the
one height stack), wearing ComposedColor (the one imagery stack) as albedo. Beer-Lambert
attenuates per channel over the real path (Jerlov-ish Kd, red dies first — which is
exactly why shoals read turquoise from orbit); deep water collapses to the shelf scatter
color the globe always drew, so the far field stays converged with no altitude branch.
Depth = live level − bed per pixel, so the bars stand out at LOW TIDE and drown at high —
the tide owns the chart. The march runs only under 30 m pixel footprints (past that the
refracted hit IS the pixel's own bed and the vertical closed form takes over; they agree
where they meet). LESSON (seen, fixed): a Fresnel term on the M7a sparkle normals
aliases into grey per-pixel speckle — Fresnel and the two ray DIRECTIONS ride the
band-limited ring normal (nSmooth); the sparkle normal feeds only the sun glint, whose
microfacet math averages sub-pixel slopes statistically. DXR stays a named experiment
(inline RayQuery over resident-tile AABBs, M6c note): at 2 secant steps against a
heightfield we own, there is nothing for RT cores to save. --rail-jetty: tip → tip →
bird's eye, shot at a low-tide --start.

**M7d (2026-08-30): THE BED CLASSIFIER — the compositor's first SYNTHESIS NODE, and the
seed of the terrain-building tool.** The user named the architecture: a node in the state
diagram (or a state diagram within the compositor) that BUILDS textures, underwater or
above, "with what the client or agent decides." The shape it took: **a synthesis source is
a state-diagram node living inside a channel stack; its EDGE pulls sibling channels (here:
the height stack — depth and slope, three SampleHeightStack calls per texel); its PROGRAM
is a data file.** BedSynthSource paints into earth.color BETWEEN google and massgis —
stack order is the authority ranking: Google's photo of open water loses to the
classifier, the classifier loses to an ortho that actually saw the bed. The program is
data/bed/bed_rules.json + bed_zones.geojson (authored with defaults if absent, never
clobbered — the M6p law): ordered rules gated on slope (rock/sand), shallow/deep dry
albedos, folded two-octave grain (an octave a rung cannot resolve is SHED, not aliased —
M6t wearing a color hat), an alpha ramp that fades where transmittance kills the bed
term anyway (−9→−14) and where land imagery takes over (+0.4→+1.2), and zone polygons
(properties.bed names a rule, properties.albedo overrides paint) as the surface a future
agentic tool or UX edits. THE IDENTITY IS THE PROGRAM PLUS ITS INPUTS: the content hash
covers rules + zones + the height stack's signature, so editing the program (or reshaping
the bed) repaints exactly the touched tiles at every rung — an agent iterating on the
rules file gets exact incremental repaints for free. Colors are DRY bed albedo: the
renderer's refracted ray applies the water's own attenuation, so the satellite look is
REPRODUCED by physics, not quoted — the ebb-shoal fan now reads sand-bright at low tide,
drowns back to teal at high, and the photo becomes a VALIDATION TARGET (render at the
photo's own tide and sun, compare — the buoy-gate doctrine, for pixels). The user's
thesis, recorded: bathymetry + the water simulation + the two rays + future atmospherics
should rival the satellite photo, especially in lighting. Registry line: L1 synth.bed,
wgs84.synthesis (height-stack product). Five gates green.

**M7e (2026-08-30): THE REFERENCE PASS — the photos said what was missing.** The user
brought aerial photos of the inlet (calm evening, top-down, and a storm-surf frame) and
called the aerial water "lame." Four causes, four fixes. (1) THE GROUPS: from altitude the
eye never sees the wave — it sees the ENVELOPE (groups, wind streaks, glitter grain), and
the fold's spatially-FLAT sigma^2 was erasing exactly that variance. The PS now
accumulates |deriv| per band weighted by wPix (what the pixel resolves) and modulates the
glint width (s2 *= 0.70 + 0.60*envN) plus a faint streak albedo — spatial glitter grain
that telescopes away by ~10 km footprints like everything else. (2) NAVY, NOT TEAL: the
references show the green band is NARROW — open water reads deep blue just past the bar.
The shelf ramp tightened from -160 m to -45 m and the deep endpoint darkened to
(0.008, 0.030, 0.080). (3) ROCK, NOT THE PHOTO: under a jetty footprint the imagery is a
smear of foam, which rendered the surveyed jetties as pale blobs. Edit-flagged land now
wears boulder-scale hash-grain rock (cell folded to the pixel footprint so it never
shimmers) — in BOTH the far path and the near-material block, which was quietly
overriding the first fix inside 2.7 km with its pale slope-riprap. (4) THE FOAM MEMORY:
the storm reference is the churn regime — breaking over the whole ebb shoal with foam
STREAMING seaward on the jet. The churn atlas (M4/M5, advected by the solved current) now
feeds the bank kernel's foam channel (gSlotsC + gChurn rows, manual-bilinear Load, 16384 m
domain at 2 m texels), so the one water carries its history at every altitude — the
named-next "churn into the bank" landed under reference pressure. --rail-jetty extended
GROUND TO SPACE: 40 s, helm at the gap -> tip-to-tip -> bird's eye -> 7 km -> 80 km ->
800 km orbit, one formula the whole way. Known residue: mesh vertex density at the jetty
walls still lets wave crests clip through the sloped side triangles between land and
water vertexes — the amplification-shader leg owns that; imagery z-boundary rectangles
still show through the refracted bed in the channel where google is the only authority.

**M7f (2026-08-30): THE RESOLUTION PASS — "a bit blocky/low res."** Three quanta were
lying about the data. (1) THE z17 DETAIL WINDOW: the renderer's near-field imagery ceiling
was the z14 window's 9.5 m texels — google z17 and the MassGIS orthos were being thrown
away at realization. A third live tenant (earth.color.detail, 16384^2 at z17, ~1.2 m px,
~14 km centred on the inlet mouth) joins the compose ladder as its third rung: same
Mercator frame, same channel stack, want emission riding the same node walk 8x finer, and
a FINER-ONLY gate (z17 mip m == z14 mip m-3) so a half-warmed detail tile can never
replace sharper coarse truth with mush. The CB grew three rows (gCsU4/gCsDet/gCsEd) —
macro and struct append together, every composed consumer recompiles consistently.
(2) THE FINE EDIT MASK: every land/water decision and the jetty flag sampled a 4096^2
mask over the whole 16 km window — 38 m texels; a 25 m jetty was one texel of staircase.
The same edit polygons now rasterize AGAIN over their own bbox (4096^2 over ~2.3 km =
0.56 m/texel); the coarse mask keeps the coastline, the fine mask keeps the structures.
The jetties render at true width — and the OLD SOUTH JETTY (OSM 559887262), a buried
remnant crossing the beach, became visible for the first time: surveyed truth the 38 m
mask had been erasing. Classification height input sharpened one rung too (38 m -> 19 m,
window mip 1). (3) THE PROGRAM DEEPENED: google's blocky water photo still leaked into
the refracted ray between the classifier's old -14 m fade and the ~-25 m transmittance
floor — bed_rules.json alpha now runs full to -24, off at -38, so everywhere the ray can
see bottom it sees the CLASSIFIER, and the photo's mip rectangles never touch the water.
Two rules edits, zero code — the M7d thesis doing its job. Known residue: the storm surf
zone over the bar fuses into a flat white sheet (churn + breaking clamp saturate
together) — breaker individuation belongs to the churn leg. Five gates green.

**M7g (2026-08-30): THE RESIDENCY REFINEMENT — "the grey pops in and out at different
angles."** Two authority bugs wearing residency's clothes. (1) THE MIP FLOOR: the
patchwork of vintages at altitude was want-vs-have divergence — regions short of their
wanted mip fell back per PIXEL to whatever coarser rung was resident, and adjacent
regions sampled different google captures. The top of every window pyramid (mips 4..7,
~85 tiles, a few MB per tenant) is now wanted EVERY frame: never evicted, warm from the
first frames, so a high view samples ONE capture uniformly and a fast ascent cannot
outrun the loader into grey. What patchwork remains is google's own capture geography
inside a single zoom — the M6j doctrine (pixels ship as google made them) accepts that,
and the M6j postmortem in Sources.cpp stays the law: no paint-time re-grading. (2) THE
MATERIAL MODEL YIELDS: the close-up beach/dune constants (M6j, born when the near field
was a 9.5 m blur) were REPLACING the imagery at 92% inside 500 m — the flat grey slab at
the helm, popping with the distance ramp as the camera turned. New ComposedColorTexelM()
reports how fine the imagery is actually RESIDENT at a pixel (window and detail rungs,
residency-map-true); where it outresolves ~1.5 m the generic constants fade to 15%. The
tide's wet band and the surveyed rock keep full authority — they know what no photo can.
Five gates green.

**M7h (2026-08-30): THE HANDOFF AND THE SETTLE — "I wonder if the compositor has a
bug."** It did — a one-directional ladder. The z14 window never yielded to the cube: even
when the view outresolved everything it had (want past its pinned floor), it kept
painting its own different-zoom capture over the cube's coherent mosaic, and its
footprint sat on the planet as a vintage RECTANGLE (the persistent tan block in every
high frame). The rule that fixed the z17 rung in M7f is now symmetric: EVERY rung earns
its place by adding detail, and where it cannot, it vanishes — the window fades to the
cube across want mips 5.5..7. The second bug wore the compositor's clothes but was the
RAIL's: frame 0 recorded the coldest instant of the run, with the height window still
streaming, so the classifier read coarse fallback and called half the channel LAND (the
flat grey panels and 150 m terraced wedges at t=0, different every run). Rails now hold
their opening pose for 150 unrecorded settle frames — residency, the solver mirror, and
the composed caches are warm before the camera rolls. Five gates green.

**M7i (2026-08-30): EXPOSURE, AND THE ORIENTATION LEDGER — "is the water inverted on the
north/south axis?"** It was not — the full chain was audited edge by edge and every
crossing agrees — but the report was RIGHT that the whitecaps disobeyed the terrain: the
FFT sea is fetch-blind, so the sheltered harbor basin whitecapped as hard as the open
bar, and sheltered-water foam reads exactly like a mirrored ocean. Three fixes: bank
corner hsScale now attenuates west of the throat on the same x-ramp the jet and churn
already use (0.18 floor for local chop, full sea seaward of the tips, solver-window
gated); the kernel's cascade foam rides saturate(hsScale) (it folded the WAVES down in
sheltered water but deposited foam at full strength); the PS detail speckle rides the
detail plane's hsScale the same way. Foam now hugs exposure geography. THE ORIENTATION
LEDGER (the audit, recorded so the next edge is checked against a table, not re-derived):
world +x = east, +z = north (z = (lat − orgLat)·mPerLat); bank atlas texel row +v = +z,
write and read agree by construction; churn atlas texel row +v = +z flat mapping
(SeaChurn TileTexel ↔ Sea.hlsl cuv ↔ bank LoadBilinearClamp all direct); CUDEM/bathy
textures row 0 = NORTH — every sampler MUST flip (Sea.hlsl:71 float2(uv.x, 1−uv.y),
SeaChurn:75 suv, bank kernel SWE reads (1−uv.y)); SWE eta/uv atlases row 0 = north, flip
required and present in all three readers; FFT cascades wrap-sample world/patchL
unflipped everywhere (kernel, Sea.hlsl PS, Globe.hlsl detail loop). The user's deeper
ask — an AST for the math so axes cannot be silently crossed — is named for the GPU GA
engine leg: when the tide rotors move on-GPU, field access goes through TYPED frame
handles (world / atlas-v-up / raster-v-down) whose conversions are explicit nodes in the
state diagram, and the ledger above becomes code instead of prose. Five gates green.

**M7j (2026-08-30): THE GA AST — the state diagram becomes data, and it found the bug
before it compiled.** The user, after three sessions of axis archaeology: "do the AST now
— it should hold all the domains, units, scales, ranges or whatever we are chaining or
folding together." Built: src/core/GaAst.h/.cpp — every edge of the state diagram
registered as data {from, to, field, src/dst Frame(space, +v direction, org, m/unit),
flip-applied, units, range, gain, code anchor}, PRINTED every run ([gaast] — the workflow
as an AST that cannot go stale because the wiring builds it), and VALIDATED: the FLIP
RULE (frames that disagree about +v need exactly one declared flip) and the ORPHAN RULE
(a producer whose consumers are all inactive is dead physics walking). And the payoff
came while REGISTERING the edges, before the code even ran: encoding Sea.hlsl's rows
surfaced SweShadow — the solver's line-of-sight swell-exposure field (CPU march toward
the peak-wave source, rebuilt when direction or level moves) — feeding ONLY the retired
SeaLayer draw path. One-water had lost the shelter edge silently; that was the basin
whitecap bug M7i mis-patched with a hand-drawn x-ramp (which then flattened the channel
and beaches: "worse"). Fix: the ramp died; the bank kernel consumes the TRUE shadow
(same SWE frame, same (1−v) flip, floor 0.18), folds displacement, foam, and shed σ² by
it (amplitude², exposure²), and writes hsScale·expo into the detail plane so the PS
sparkle/envelope tiers inherit shelter for free. NEW GATE gatest (src/core/GaTest.cpp,
gate 2 of 6): the sandwich (−n v n, twice = identity), the refraction rotor vs the
shader's closed form (+ Snell pinned), the fold telescope's endpoints (footprint==ring
adds zero; full pixel adds exactly the shortfall), the Cl(2)+ spinor blend (mean phase
exact, amplitude folds as cos(Δφ/2)), and the AST's flip rule + the orientation ledger's
ground truths asserted edge by edge. Six gates green. The doctrine: NEW EDGES REGISTER
OR THEY DON'T SHIP — a frame mismatch is now a boot-time report, not a debugging session.

**THE HYPERVISOR (same day, user: "a debug function that prints all the work and
transformations for some steps... validate against other tools... know the exact step
something is going wrong even before we hit the GPU").** --trace lat,lon walks ONE sample
through the whole chain and prints every transformation with its AST edge: frame
conversion (with the org/scale constants), bed (with provenance), level, current, depth
and the breaking threshold, sea state -> hsScale law, exposure, the fold weights per
ring, and — the cross-check — 9b: a READBACK of the actual bank texel (disp, foam,
level, sigma2, current, hsScale*expo) against the CPU's expectation. Plus
trace_shadow.pgm so the exposure field can be LOOKED at. It caught three real bugs in
its first hour: (1) the swell shadow could build from the DEFAULT direction (toward
east) before partitions loaded — the march then walked west into the dunes from every
ocean cell and the entire sea read as deep shadow (fix: m_peakDirValid — no real
direction, no shadow); (2) parts[0] is FILE order, not energy order — at calm hours the
peak direction came from the 0.05 m westerly wind chop instead of the easterly swell
(fix: the peak is the most energetic partition; cPeak rides it too); (3) THE LOST LINE —
M7i's ramp edit had swallowed the unconditional t.hsScale assignment and every bank tile
had shipped hsScale = 0 since: the "water seems worse" report was a dead-flat sea, and
the 9b cross-check found it in one probe (CPU expected 0.42, the bank said 0.00) after
three rounds of visual review could not localize it. Named next for the trace: a
celestial step (the sun disc renders suspiciously EAST in an 18:15 EDT helm shot — sun
azimuth wants a hypervisor line and a gatest pin), and a churn/bank texel diff.

**M7l (2026-08-30): THE PILLARS JOIN THE DIAGRAM — debug tools as the product, not the
byproduct.** The user's program: every pillar gets its AST, its proof, and its hypervisor
check, with a report each debug session ("the free biproducts mathematicians get from
GA"). Landed: (1) the COMPOSITOR and RESIDENCY pillars registered (24 edges total now) —
including the asymmetry the table exists to keep straight: the window uv is v-SOUTH
(mercator order, sampled with NO flip) while every water atlas is v-NORTH (flip at each
row-0-north raster); the paint loop's merc-inverse-per-texel IS the compositor's flip.
(2) THE COMPOSED-TILE CROSS-CHECK (--trace step 11): same point, three answers that must
agree — the CPU stack (the law), the RESIDENT GPU texel of the height window (read back
one texel at the mip the CPU residency map names), and the map itself. First run: GPU
-9.58 m vs stack -9.59 m, MATCH at 8 mm. stack -> cache -> GPU, end to end. (3) THE
SESSION REPORT: every run ends with [report] — AST edge count + flip verdict, compose
painted/cache-hit, fetches, pending. (4) PIX programmatic captures: --pix N loads the
INSTALLED PIX's WinPixGpuCapturer.dll (no NuGet — and the export drifted: modern PIX
exports CaptureNextFrame, not PIXGpuCaptureNextFrames; we parse for both) and writes
gagame.wpix with every pass already named by its state-diagram node via PixScope.
--dump-fibers exports the three bank planes as PNGs and gates their VALUE RANGES against
the AST declarations (first run: all five in range — and the numbers themselves verified
the healed water: disp.y +-1.3 m for a 1.2 m sea, level -1.28 at low tide). Named next:
pixtool automation (diff a capture's bank dispatch outputs against fiber PNGs), a
cube-face texel cross-check (step 11 for the color cube), CsSweDerive/eta writer edges.

**M7m (2026-08-30): THE SANITY LENSES, THE TEST CARD, AND THE SCRIPTORIUM.** The
session's pivot completed: debug tooling is the product. (1) --lens
worldxz|winuv|mip|ring — values as COLOR: the world checker (a frame or scale error is a
seam), the window uv gradient (its v-SOUTH direction visible), the residency heat map,
the bank rings with their 4-texel checker. A domain error stops being an argument and
becomes a broken pattern. (2) --inject — EDGE PATTERN INJECTION: the bank kernel writes a
world-aligned test card (50 m checker + north wedges every 500 m) into the foam fiber
instead of physics; if the card arrives on screen continuous across rings, unmirrored,
wedges north — the bank->render edge is clean by inspection. First run: clean, and the
card also demonstrated the dry gate and the fold eating sub-texel wedges at coarse rings
(honest attenuation, visible). (3) THE SCRIPTORIUM INTEGRATION: the AST writes
docs/GA_AST.md every boot (the diagram as a markdown table, generated never hand-edited)
and the scriptorium reindexed it — a future session QUERIES the edge table instead of
re-deriving conventions from shader archaeology. Six gates green.

**M7n (2026-08-30): THE COINCIDENCE CARD, THE HALF-TEXEL CATCH, AND THE CONVENTION
RULE.** The user: "the patterns should also be used to make sure the wave tiles are
aligned correctly too." Built the two-color coincidence test: --inject cascade makes the
bank kernel draw a quadrant card from ITS belief of cascade-1's wrap uv into the foam
fiber (red); --lens cascade makes the PS draw the SAME card from ITS mapping (green).
Agreement is pure yellow; any relative flip, offset, or scale between the wave tiles'
two consumers fringes red/green. FIRST RUN: quadrants coincided (no flip/scale between
kernel and PS) — but a ONE-SIDED red fringe rode every seam border, the signature of a
HALF-TEXEL TRANSLATION. The audit that followed found it: the kernel wrote texel id at
org + id*texel (corners) while BankSample reconstructs centers (local - 0.5) — and the
kernel was internally mixed (its corner-lerp f already used id + 0.5). Every bank field
had sat half a ring texel off: 2.4 m at ring 0, 77 m at ring 5, invisible on waves,
undeniable on the card. Fix: the kernel writes CENTERS; the re-run card is clean yellow
with symmetric resolution blur only. THE GA READING (the review the user asked for):
every sampling edge is a pullback along an affine map, and the group of errors it can
carry is exactly {flip, scale, translation}; the flip rule caught reflections, the
range/scale rows catch dilations, and the card exposed the TRANSLATIONS — so the AST now
carries the missing generator: Frame.centers (samples at cell centers vs lattice
corners) and Validate's CONVENTION RULE fails any edge whose sides disagree — an
undeclared half-lattice translation is now a boot error. THE FULL AUDIT (each edge
walked): bank kernel<->BankSample FIXED (centers both); kernel corner-lerp f centers ok;
SWE reads: solver writes centers (worldX0 + (t.x+0.5)dx), all three readers reconstruct
centers with the row flip — CORRECT; churn: writer centers (texel+0.5)*2m, both readers
centers — CORRECT; shadow: builder centers, both readers centers + flip — CORRECT;
cascades: FFT texel i lives at i/N*L (corners) but EVERY consumer (kernel
LoadBilinearWrap, Sea.hlsl and Globe.hlsl hardware samplers) reads centers — a CONSTANT
half-cascade-texel phase shift (1.5/0.36/0.09 m) identical in all consumers of a
periodic random field: harmless, documented here so nobody "fixes" one consumer alone;
compose windows: painter and hardware samplers both centers (composetest-pinned). Six
gates green.

**M7o (2026-08-30): THE SLICE PLANE — the doctrine's first full lap.** The user: add a
new GA node to see if the AST work makes features easier, with something easy to verify.
The pipeline ran exactly as written: (1) THE MATH — the cut is a plane pi = (n, d); the
keep predicate is the sign of the inner product s(P) = P.n - d; the correctness statement
is that the sandwich -pi P pi negates s, so the predicate splits precisely the halves
the reflection exchanges. (2) THE PROOF — the identity asserted numerically in
proofs/slice_plane.py AND pinned in gatest (4b). (3) THE SMALL RENDER —
proofs/slice_plane.png: a 2D circle-planet sliced at z = d with the profile on the cut
edge, drawn by the same predicate. (4) THE ENGINE NODE — --slice d: ONE inner product
and ONE discard in the globe PS; the node registers its AST edge at wiring time like
everything else ([gaast] node user.plane). (5) THE VERIFICATION — the cut runs
razor-straight through world z = 40 at the jetty line; the exposed silhouette is the
terrain profile, checkable against --bathy-map. Elapsed: minutes, not sessions — the
AST answer to "does this make adding features easier" is yes, because the new node had
to say where it lives (frames, units, predicate) before it could ship. Six gates green.

**THE DX12 GOTCHAS LEDGER (and the non-GA-workaround audit the user asked for: every
workaround marked, with whether a GA formulation would have needed it at all):**
1. Bindless static-sampler SampleLevel returns ZERO outside the pixel stage on this
   driver (RTX 5060) — mesh AND compute. Workaround: manual-bilinear Loads everywhere a
   kernel samples bindlessly (LoadBilinearWrap/Clamp). NON-GA WORKAROUND — and arguably
   GA never needed the sampler: the fold treats a texture as a FIBER over the base space,
   and gathering four fiber values and blending them IS the geometric product we wanted;
   filtering hardware was a convenience, not a requirement. The GA formulation sidesteps
   the trap by construction.
2. Padded-vs-logical dims of tiled atlases: reserved texture desc = LOGICAL grid; TilesX
   * TileW padding is atlas ADDRESSING only. Normalizing by padded dims reads garbage
   (bit twice: ReadFields M6x, kernel SWE M7). The AST's metersPerUnit + range rows exist
   so this class dies at boot.
3. Reserved-tile NULL reads return zero BY SPEC — not a workaround, the atlas thesis:
   sparse structure carried as algebra ("no water here costs nothing").
4. RGBA16F hardware tile = 128x64: one logical 128^2 tile is a 1x2 hardware pair.
5. R16F window pyramids stop at the one-tile-ish mip (16384 -> 256, max lod 6); the g[2]
   clamp in FillComposedCb is that fact, not a tuning choice.
6. GetCopyableFootprints row pitch is 256-aligned; a WIC writer fed unpadded rows crashed
   at widths that weren't multiples of 64 texels (Image.h carries the scar).
7. PIX capturer export drift: PIXGpuCaptureNextFrames (pix3 name) vs CaptureNextFrame
   (the DLL's real export in current releases, same signature) — resolve both.
8. Static samplers live in the root signature and cannot be indexed bindlessly — another
   push toward Load-based fibers (see 1).
9. Command-list BeginEvent metadata 1 (legacy ANSI) is decoded by PIX and RenderDoc with
   no runtime DLL — markers need no dependency.

**THE ALGEBRA-FIRST DOCTRINE (user, same day — how every session works from now on):**
sessions on this project are Geometric/Clifford/other-algebra FIRST. The pipeline for any
new leg: (1) WRITE THE MATH OUT — the derivation in algebra, on paper/markdown/sympy;
(2) PROVE IT — the identities become assertions (gatest is the proof's permanent home);
(3) RENDER A SMALL EXAMPLE — a standalone 2D/3D prototype in any cheap medium
(matplotlib, canvas, a 128-cell grid) that shows the behavior with the SAME numbers the
proof used, committed under proofs/; (4) ONLY THEN into this renderer — where the same
visualization returns at higher fidelity, framerate, and solar scale, and the hypervisor
(--trace) verifies the engine reproduces the prototype's numbers step by step. The AST
edge for the new field registers at integration time and its `code` anchor names the
proof. What this buys: the math maps cleanly (each stage is small enough to hold whole),
axis/scaling bugs die in the 2D prototype where they cost minutes, and a future session
can re-read the proof instead of reverse-engineering the shader.

Named next: amplification-shader subdivision for sub-meter helm vertex density (also the
jetty-wall clipping fix), breaker individuation in the surf zone (churn saturation), the
bank as an Exchange channel (water.surface for plugins), a
light wind-chop tail partition so calm water carries its capillary glitter, underwater
caustic modulation of the bed term from the cascade derivs, the GPU-side GA engine
growing out of this kernel (the tide rotors evaluated on-GPU from the realized phasor
windows), more synthesis nodes (wetland/eelgrass classifier, a render-vs-photo residual
gate), and the terrain-building tool proper: an agent or UX writing bed_rules /
bed_zones / edits.geojson through a common "program file" contract.
