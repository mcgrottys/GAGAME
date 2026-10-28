# Code habits review, 2026-10-28

Scope: a read-only sample of `src/sim/SweSolver.{h,cpp}`, `ImpactWave.h`, `ImpactField.h`, `WaterManager.{h,cpp}`, `WaterSurfaceTree.cpp`, `scene/Rail.{h,cpp}`, `scene/SceneSchema.cpp`, `scene/Props.h`, `scene/WaterBankLayer.cpp`, `compose/SurfaceFrame.cpp`, `app/FrameLoop.cpp`, `app/Assembly.cpp`, `shaders/Swe.hlsl` and `shaders/laws/Laws.hlsli`, at 2d61a89 plus the working tree. Every line cited below was read.

## 1. Verdict

The Python habits here are not about syntax. They come from how a research notebook grows. Each new stage of the work adds a member, a bool, an environment switch and a dated comment paragraph to the class or function that already exists. As a result `SweSolver.cpp` is 4101 lines, `FrameLoop::Frame()` is about 2800 and `Session()` about 1500. Meaning lives in conventions rather than types:

- `int side` 0..3
- `double box[4]` in three different orders
- tile-statistic flag bits written as literals on both the CPU and the GPU
- a provenance string whose first character says whether a value exists
- `-1e18` as "never"
- `bool` plus out-parameter returns

Across `src/`, `std::optional` appears five times, and only for deferred construction. `std::variant`, `std::span`, `[[nodiscard]]` and `std::numbers` appear zero times.

Two changes would move the codebase the most:

1. **A small vocabulary header.** It would hold `LatLon`, `LatLonBox`, `enum class Side`, the tile-statistic record and its flags (shared with HLSL the way `shaders/laws` already shares laws), and `kD2R`/`GreatCircleM`/`DirOf`. This is mechanical work, and it removes whole classes of silent bugs.
2. **Split "decide" from "record".** Pure functions would return small records (the wanted tile set, the stable step, a probe row). The GPU-recording methods would consume those records. In `SweSolver::Level` and the residency code, named functions or enums would replace the bool-flag parameters.

## 2. Findings, by payoff

### F1. Geography as bare arrays and ints (safety)
Boxes are `double[4]` and edges are `int`s or `char`s. Each site's order is given only in a comment, and the orders disagree.
- `sim/WaterManager.h:235` `double box[4]; // lat0, lon0, lat1, lon1` (also `SweSolver.cpp:2448`)
- `scene/SceneSchema.cpp:231` BindBox `"[lon0, lat0, lon1, lat1] degrees"`
- `scene/WaterBankLayer.cpp:373` `box[0] = la0; box[1] = la1; box[2] = lo0; box[3] = lo1;`
- `sim/SweSolver.h:61` `int side = -1; // 0 N, 1 S, 2 W, 3 E`, beside `:150` `char edge; // 'N', 'S', 'W', 'E'`

Why it hurts: three box orders for one shape. A box passed across these files compiles and reads the wrong corner. `SweDomain` already names `lat0, lon0, lat1, lon1` (SweSolver.h:100), so the right shape exists here and only needs promoting.
```cpp
struct LatLon    { double lat = 0, lon = 0; };                 // degrees
struct LatLonBox { LatLon lo, hi;
                   bool Holds(LatLon p) const { return p.lat >= lo.lat && p.lat <= hi.lat && p.lon >= lo.lon && p.lon <= hi.lon; } };
enum class Side : uint8_t { N, S, W, E };
constexpr char Letter(Side s) { return "NSWE"[size_t(s)]; }
```
First step: introduce `LatLonBox`, and convert `LevelBox`, `WaterManager::Window::box` and `MaskBox` (three types, one PR). `BindBox` keeps its JSON order and fills the struct.

### F2. The tile statistics: one record, four hand decoders (safety, clarity)
`CsSweTileStats` packs `uint4(flags, hMax, uMax, vol-or-range)`. The flag bits are literals on both hosts, and the fourth lane changes meaning on a float mode flag. The CPU reads it back as raw bytes with `memcpy`.
- `shaders/Swe.hlsl:907` `if (c.y == 0u) flags |= 2u;` and `:949` `... (gTail2.z > 0.5f) ? gsRange : asuint(gsVol[0])`
- `sim/SweSolver.cpp:1225` and again `:1888` `if ((v[0] & 2u) && ty > 0) want[...] = 1;` (the ring logic is duplicated between `UpdateResidency` and `UpdateLevelResidency`)
- the readback helpers `readR32` (`:1438`, again `:3473`), `readAll` (`:2573`) and `readLevel` (`:2612`) are the same lambda four times. They sit over `Gpu::ReadbackTexture(GpuTexture&, uint32_t* outRowPitch)` (hal/Gpu.h:205), whose comment says "only used by --dump", yet it has 45 callers.

The C#/F# shape is to put the contract in the `laws` dialect, include it from both hosts, and give the CPU a typed pitched view:
```cpp
// shaders/laws/TileStats.hlsli  (one text, two hosts)
static const uint kStatLive = 1u, kStatEdgeN = 2u, kStatEdgeS = 4u, kStatEdgeW = 8u, kStatEdgeE = 16u;
// C++
struct TileStat { uint32_t flags; float hMax, uMax, volOrRange; };
static_assert(sizeof(TileStat) == 16);
struct Plane { std::vector<uint8_t> bytes; uint32_t pitch = 0;
  template <class T> std::optional<T> At(uint32_t x, uint32_t y) const; };
Plane ReadBank(Gpu&, TileAtlas2D&, uint32_t mip, D3D12_RESOURCE_STATES);
```
First step: add `TileStats.hlsli` with the five flags. Then write one `RingWants(const Plane&, ...)` used by both residency functions.

### F3. Provenance strings that carry presence, and a dangling pointer (safety)
`WeatherSample` says which source answered with a `const char*`. Readers then test the first character to learn whether a value exists at all.
- `sim/WaterManager.h:57` `const char* levelSrc = "-";` The comment above it says "(static strings)".
- `sim/WaterSurfaceTree.cpp:215` `(q.levelSrc && q.levelSrc[0] != '-') ? q.levelNavd : std::nan("")`, and again at `:244` and `:291`
- `sim/WaterManager.cpp:135` `s.levelSrc = w->levelTag;` points at `char levelTag[48]` inside a `std::vector<Window>` element (WaterManager.h:247, 269). The string is not static, and the pointer dangles if the vector grows after `AddExternalWindow`'s `push_back`.

```cpp
enum class Source : uint8_t { None, Atlas, Stations, Solver, Gulf /*...*/ };
struct Sampled { double value; Source from; };
std::optional<Sampled> level, current;      // presence is the optional, not '-'
```
First step: add `bool hasLevel/hasBed` (or optionals) and change the three readers in `WaterSurfaceTree.cpp`. Copy `levelTag` into an owned `std::string` in the sample, or intern it in a stable table.

### F4. Out-parameter returns and sentinels where an optional or record fits (clarity, safety)
A `bool` return with out-parameters is the house return convention. "Not yet" is encoded as `-1e18`, `-1e17`, `0.0` or `-1.0`, and each site picks its own.
- `SweSolver.h:128-129` `bool CellOf(double latDeg, double lonDeg, double& tx, double& ty) const;`, `:457` `bool DeviationAt(..., Deviation& out)`, `:417` `bool TraceBed(Gpu&, BedTrace& out, ...)`, `:243` `Volumes(Gpu&, double& columns, double& soil, std::vector<float>*)`
- `:255` `std::function<bool(double, double, double, double&, double&, double&)>`: the out-parameter convention baked into a callback type
- sentinels: `SweSolver.h:648` `m_probeAt = -1e18`, `:654` `m_imposedAt = -1e18`; `SweSolver.cpp:1867` tests `> -1.0e17`, while `:1202` tests `m_statsAt > 0.0` for the same idea; `ImpactField.h` keeps `zeroUnix = -1.0` in both `Event` and `ImpactWave::m_zero` (one state, two copies)
- Rail: 29 header signatures of the form `bool X(..., std::string* why)`

The records already exist (`Deviation`, `BedTrace`, `FieldsRead`). Only the return convention lags behind them.
```cpp
struct Cell { double x, y; };
[[nodiscard]] std::optional<Cell> CellOf(LatLon p) const;
[[nodiscard]] std::optional<Deviation> DeviationAt(LatLon p) const;
std::optional<double> m_imposedAt;           // "never" is nullopt
template <class T> struct Result { std::optional<T> value; std::string why; };   // C++20: no std::expected
```
First step: `CellOf`/`CellOfDir`/`TexelOf` return `std::optional<Cell>`. There are many call sites, and each gets shorter. HLSL keeps its `out` parameters; they are idiomatic there.

### F5. Laws as repeated magic numbers that have drifted (correctness, clarity)
The same law is written inline at several sites, each with its own floor.
- the Courant step, three times with three depth floors: `SweSolver.cpp:580` `0.45f * min(dx, dyM) / sqrt(g * max(hmax, 5.0f))` (hmax = deepest + 3), `:1290` `sqrt(g * max(hMax, 0.5f)) + uMax`, `:2201` `0.45 * ... / (2.0 * sqrt(g * max(-deepest + 10.0, 50.0)))`
- clock policy: `:2870` `drift > 3600.0`, `:2875` `drift > 900.0`; the spin-up cutoff at `SweSolver.h:291` `m_dt < 0.005f`; `:301` `Record(..., 9999)`
- `SweSolver.cpp:609` `m_cb.cd = 0.0015f;`; `:1240` `hMax > 60.0f || uMax > 25.0f`

Why it hurts: the owner's rule is one continuous law. Three CFL expressions are three laws, and the floors decide when the window or a level goes unstable.
```cpp
namespace swe {
inline constexpr float kCourant = 0.45f;
inline constexpr double kScrubResetS = 3600.0, kStarveReanchorS = 900.0;
[[nodiscard]] constexpr float StableDt(float dx, float dy, float g, float hMax, float uMax) {
    return kCourant * (std::min)(dx, dy) / (std::sqrt(g * hMax) + uMax); }
}
```
First step: `StableDt` at all three sites. The floors become arguments with names, and a reader can see where they differ.

### F6. Geodesy and constants re-derived inline (clarity, small correctness)
- `3.14159265…` appears 223 times in `src/`, with 17-digit and 15-digit variants (e.g. `compose/SurfaceFrame.cpp:322`, `:483` and `:505`, three in one file).
- Earth's radius appears 54 times, as both `6371000.0` and `6371008.8` (`scene/BuildingLayer.cpp:453`). `ImpactField.h:141` defaults `m_R = 6371000.0, m_g = 9.81` beside `core/Planet.h`'s `kEarthBody`.
- The haversine is written inline twice in `ProbeCoarse` (`SweSolver.cpp:2675`, `:2719`), beside `GreatCircleM` (`scene/SeaLayer.cpp:205`) and `BuildingLayer::ReachBox`.
- The lat/lon-to-direction formula has 7 copies (`ImpactWave.h:72-75`, `:103`; `SurfaceFrame.cpp:324` ...), although `sim/Ephemeris.h:119` `PlanetDirFromLatLon(..., double& x, double& y, double& z)` exists. It has out-parameters, which may be why nobody uses it.
```cpp
// core/Geo.h
inline constexpr double kD2R = std::numbers::pi / 180.0;
struct Dir3 { double x, y, z; };
[[nodiscard]] inline Dir3 DirOf(LatLon p) { /* the globe's frame: lat = asin(y), lon = atan2(z, x) */ }
[[nodiscard]] inline double GreatCircleM(LatLon a, LatLon b, double R = ThePlanet().R);
```
First step: add `core/Geo.h` and use it in the files touched next. A sweep is not needed.

### F7. Environment variables as configuration and leftover bisection switches (clarity, safety)
There are 17 `getenv` sites. Two different truth rules are in use, and one variable is read in two files.
- `SweSolver.cpp:1135` `std::getenv("GAGAME_SWE_NOSEA") != nullptr`: setting it to `=0` still turns the sea off. `WaterBankLayer.cpp:1270` uses `e[0] == '1'` instead.
- `GAGAME_NO_CASCADE_CREST` is read at `WaterBankLayer.cpp:1270` and again at `sim/WaterSurfaceTree.cpp:513`.
- Self-described "bisection switch"es: `SweSolver.cpp:2424` `GAGAME_TAIL_ORDER_MADE` and `app/FrameLoop.cpp:4401` `GAGAME_TAIL_UNDRAWN`. 5ea5c3c says the marsh columns "are NOT the levels", so the bisection they served has concluded.
- `GAGAME_SWETEST` selects test cases by `strstr` (SweSolver.cpp:3470).

Why it hurts: the memory rule says "config lives in the scene file". Environment switches are invisible to `--print-scene`, the codemap's scene folding and `docs/scene_schema.json`.
```cpp
struct DebugSwitches { bool noSea, noSoil, noCascadeCrest, levelStats; };   // read once at boot, one rule
const DebugSwitches& Debug();                                               // or: a `debug.*` scene section
```
First step: delete the two concluded bisection switches. Route the rest through one `Debug()` reader with one truth rule.

### F8. Bool-parameter soup and a type that is three kinds (clarity)
- `SweSolver.cpp:1682` `AddLevel(gpu, rank, 0u, cx, cy, name, true, true, true, half);` The signature is `(…, bool patch, bool tables, bool sparse, uint32_t sparseHalfCells)` (SweSolver.h:716).
- `RecordLevel(..., bool reset, bool steps)` (SweSolver.h:722). `steps == false` is a different function, the prelude (`SweSolver.cpp:2265` `if (!steps) {`), and it is called both ways at `:2969` and `:2974`.
- `Level` (SweSolver.h:666) carries `foldUp, patch, sparse, fresh, statsDue, probesSaid`. Anchor, patch and sparse patch differ in residency, bed and statistics, and every function re-tests the bools.
```cpp
enum class LevelKind : uint8_t { Anchor, Patch, SparsePatch };
struct LevelSpec { uint32_t rank; uint32_t cells; Cell centre; std::string name;
                   LevelKind kind = LevelKind::Anchor; bool tables = false; uint32_t sparseHalf = 0; };
int AddLevel(Gpu&, const LevelSpec&);   // AddLevel(gpu, {.rank = r, .centre = c, .name = n, .kind = LevelKind::SparsePatch, .sparseHalf = half})
void RecordLevelPrelude(hal::CommandContext&, Gpu&, Level&, double batchEnd, bool reset);
void RecordLevelSteps  (hal::CommandContext&, Gpu&, Level&, double batchEnd, bool reset);
```
First step: split `RecordLevel` into its two halves, a pure rename plus two call sites.

### F9. Giant functions that interleave deciding, recording and reporting (clarity, testability)
- `FrameLoop::Frame()` runs from 2643 to 5440, `Session()` from 542 to 2051, `SweSolver::Init` from 290 to 869, `WaterBankLayer::Render` from 924 to 1630, and `RunSchemeTests` from 3404 to 3762.
- `SweSolver::UpdateResidency` (`SweSolver.cpp:1201-1341`) decodes statistics, applies the eye's reach, aligns 2x2 tiles, sets the time step, maps and unmaps tiles, rebuilds live words and logs the tile picture, all in one body.
- `FrameLoop.cpp:4396-4405` fans the same call out over `waterBank`, `waterBankB` and `m_A.waterBankEye` behind three null checks. The pattern appears 9 times.

The F# shape is a pure core and an imperative shell. Wanted-set computation can be tested without a GPU.
```cpp
struct TileWants { std::vector<uint8_t> want, keep, beyond; float hMax, uMax; uint32_t wet; };
[[nodiscard]] TileWants WantedTiles(const Plane& stats, const TileGrid& g, std::optional<EyeReach> eye,
                                    std::span<const uint8_t> boundary);         // pure
void ApplyWants(Gpu&, const TileWants&);                                        // maps, unmaps, logs
```
First step: extract `WantedTiles` from `UpdateResidency` and share it with `UpdateLevelResidency` (see F2).

### F10. Stringly-typed scene values after parse, and quantities mis-declared (safety)
The schema parses into strings or ints, and consumers compare strings or magic ints.
- `FrameLoop.cpp:838` `mode == 1 && S.scene.view == "orbit"` (`mode` comes from `BindEnum`, which writes an `int&`: `scene/Props.h:144`); `:1102` `S.time.start == "now"`
- `app/Assembly.cpp:494` `marsMode = (S.scene.planet == "mars");` and `:312-316` `s.kind == "seastate" / "buildings" / "roads" / "bridges" / "composite"`
- `scene/Rail.cpp:479` `if (s.up == 1)`. `up` is `BindEnum({"none","gravity"})` with no enum behind it (Rail.h:183).
- `scene/SceneSchema.cpp:211-212` binds `speedMs` and `densityKgM3` as `Q::Dimensionless, "1"`, although `core/GaUnits.h:51` has `Velocity` and `Density`. The unit check that `Quantity` exists for is switched off for exactly these two keys.
```cpp
template <class E> requires std::is_enum_v<E>
Schema& BindEnum(const char* key, E& f, std::initializer_list<const char*> names, const char* doc, Reload r);
enum class SceneMode : uint8_t { Chart, World, Gulf };
const PlanetBody* planet = &kEarthBody;      // resolved at parse; no "mars" downstream
```
First step: declare `speedMs` as `Q::Velocity, "m/s"` and `densityKgM3` as `Q::Density`. Then add the typed `BindEnum<E>` beside the int one and migrate `scene.mode` and `rail.segment.up`.

### F11. Parallel arrays and stride arithmetic where a struct or variant fits (clarity)
- `scene/Rail.h:184-188`: `keys`, `keyT`, `keyAt`, `keyHasAt` (a `vector<bool>`), `keyView` and `keyPose` are six vectors indexed together. `Segment` is a union of fields chosen by `int kind`, and it also mixes parsed fields with "resolved" ones (`:195`).
- `sim/ImpactWave.h:67` `m_terms[size_t(i) * 4 + 0] = k;` and `ImpactField.h:109-120` `h[0]..h[7]`, `words[i * kHeaderWords + 6]`
```cpp
struct RailKey { double t; Motor pose; std::optional<JsonValue> at; std::string view, poseName; };
using SegmentLaw = std::variant<KeysSeg, SpiralSeg, HoldSeg, TurnSeg>;   // Eval = std::visit
struct ImpactTermGpu { float k, omega, A, pad; };  static_assert(sizeof(ImpactTermGpu) == 16);
struct ImpactHeaderGpu { float dir[3], zeroRel, basinDepth, exag, firstTerm, termCount; };
std::span<const ImpactTermGpu> Table() const;
```
First step: `RailKey` (one struct, one vector). `ImpactTermGpu`/`ImpactHeaderGpu` are a 20-line change with `static_assert`s.

### F12. Root-signature slots and constant-buffer lanes as literals (safety at the HAL boundary)
- `SweSolver.cpp` contains 81 calls of the form `ComputeSrvAt(<int>`, `ComputeTable(<int>` or `ComputeConstantsAt(<int>`, for example `:2287` `cmd.ComputeSrvAt(5, pLiveVa);`. The slot meanings are documented only in comments (SweSolver.h `m_table` comment).
- Mode bits travel as floats in packed lanes: `gTail2.z > 0.5f` (Swe.hlsl:904), `SweCbData::unused0` (SweSolver.h:501), `soil[3]` as an on/off switch (SweSolver.cpp:608).

Keep the raw packing, because it is the HLSL contract. Name the slots:
```cpp
enum SweRoot : UINT { kCb = 0, kTable = 2, kLive = 3, kChain = 4, kParentLive = 5, kGhostLive = 7 };   // plain enum: ints at the HAL
```

### F13. Instruments as prose log lines (clarity, gate reliability)
`ProbeCoarse` (`SweSolver.cpp:2561-2739`) does several jobs at once. It reads whole banks back synchronously, computes great circles, keeps "said once" flags (`m_lineProbesSaid`, `L.probesSaid`), and builds one line with `char buf[200]` plus `std::string +=` (`:2593`) across four probe kinds. Commits then quote these lines as gate evidence. The same habit shows in `static bool said` (WaterManager.cpp:93, `:115`) and the public `std::string stats` (SweSolver.h:430).

The shape is rows, not sentences: `struct ProbeRow { double t; std::string_view where; float fine, coarse, law; }`. Emit them to a run file (CSV or JSON lines under `out/`), and let `Log` print a one-line summary. The prose stays for people, and gates diff the rows.

### F14. Comments that narrate history instead of stating invariants, and drift (clarity)
- `SweSolver.h` has 286 comment lines out of 799, with 45 stage/phase/milestone tags and 16 dates. Rationale such as "(Until this date a side was open when…)" (`:53-55`) belongs in `docs/WATER_V2.md` and the commit.
- Drift: `SweSolver.h:593` carries `m_releaseS`'s comment pasted after `m_levelStats`'s, and `:706` holds two comments. `hal/Gpu.h:204` says "only used by --dump" while the function has 45 callers.

Keep the invariant sentences (what holds and why). Move "stage 2d, 2026-10-27: before this…" into the doc that already tells the history. Where a comment explains a field's meaning, a type should say it instead (F1, F4, F12).

## 3. Already in the right shape (keep as house style)

- **One text, two hosts.** `shaders/laws/Laws.hlsli:1-30` and `sim/WaterTerms.h:57-59`. Each law has one body, `real` belongs to the host, and laws return structs. This is the model for F2's statistics contract.
- **Immutable value types for physical constants.** `core/Planet.h:25-37`: `struct PlanetBody` with `inline constexpr kEarthBody/kMarsBody` and `SurfaceGravity()` as an expression-bodied member.
- **Quantities with units in the schema.** `core/GaUnits.h:51` `enum class Quantity` and `scene/SceneSchema.cpp:186-188` `.Bind("hs", p.hs, Q::Length, "m", ...)`. One table feeds the parser, the docs and the codemap.
- **Templates over type erasure where it is hot.** `SweSolver.h:287` and `:311`: `AdvanceTo`/`Spinup` are templated on `F, G, Q`, so there is no `std::function` in the spin-up loop.
- **RAII and closed sets at the HAL.** `PixScope` (SweSolver.cpp:2835), `enum class hal::Owner { Frame, Upload }` (hal/Context.h:38), and `WaterManager(const WaterManager&) = delete` with its reason (WaterManager.h:71). `std::vector<std::unique_ptr<Level>>` (SweSolver.h:699) has one owner.
- **Layout discipline across the boundary.** The "APPENDED AT THE END" rule (SweSolver.h:528) and `static_assert(kMaxRiverFaces == 4, ...)` (SweSolver.cpp:617), backed by `tools/astindex/verify_layouts.py`.

## 4. Considered and rejected

- **`double` everywhere on the CPU.** This is deliberate: anchors and addresses are kept in doubles, and the narrowing to float happens explicitly at the constant-buffer fill. Changing it would reintroduce the shimmer that was fixed.
- **Replacing every `std::function`.** The stored sources (`m_levelAt`, `m_bedAt`, `VesselFactory`) are called a few times a batch or at init. Templates would spread solver internals into headers for no measurable gain. The type to fix is the out-parameter signature inside one of them (F4), not the erasure.
- **String-keyed maps.** `core/Registry.h:77`, `sim/VesselSpec.h:386` and `compose/Compositor.h:329` are keyed by names that come from files, on cold paths. That is the right boundary.
- **Per-frame allocation.** `WaterBankLayer.cpp:931` (`std::vector<BankTile>`, at most 96 tiles) and `LevelsGpuDesc`'s `order` (SweSolver.cpp:2422) are small. The frame budget lives in water pixels. Hoist them when the code is touched, but not as a campaign.
- **Raw arrays and float4 lanes in `SweCbData` and `BankTile`.** These are the shader contract. Name them (F12), but do not wrap them.
- **Blanket `noexcept`/`[[nodiscard]]`.** The engine uses no exceptions, so `noexcept` buys nothing outside move constructors. Use `[[nodiscard]]` on the new optional and record returns only.
- **HLSL `out` parameters.** They are idiomatic there and cost nothing. F4 is a C++ finding.
