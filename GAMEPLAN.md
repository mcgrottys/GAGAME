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

**Deferred / next:**
- **Grade normalization as a compositor layer**: Google's per-zoom color grading shows as
  banding between resident-mip regions; fix at PAINT time (histogram/gain matching between
  zoom levels — the first "fun raster-kernel" layer, and a natural GA toy: color ops as rotors
  in a chromaticity algebra).
- **DirectStorage** for the composed-tile folder (the layout is already right).
- **water / air channels** through the same registry (SWE eta and the cloud bank become
  channel realizations; sudden weather updates = a new stack version = a fresh cache folder).
- **Octrees**: the 3-D banks join the compositor the way the quadtrees did.
- **The mesh-shader unification (user call, the endgame)**: one planet surface pipeline where
  mesh shaders amplify geometry straight from the composed height tiles (tier confirmed on
  this GPU), and TerrainLayer stops existing as a separate description of the same Earth. The
  height-window realization is the prerequisite and now exists; the SWE solver can read the
  same window the renderer streams (concurrent read/write is the design constraint to carry).
- **Shoreline upgrade path**: GSHHG f (~100 m fidelity) → NOAA CUSP where finer survey truth
  is wanted; the registry makes that a one-line stack swap.
- Antarctica's grounding-line levels (5/6) are folded crudely into the global mask; irrelevant
  to the estuary, noted for honesty.
