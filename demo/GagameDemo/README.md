# GagameDemo

The demo launcher: one .NET 10 WinForms exe that stands beside a GAGAME checkout and drives it
the way `docs/LAUNCH.md` says to. It carries no data, shaders or scenes of its own; it points at
the checkout (found by walking up from its own exe, else asked for once) and runs
`build\bin\gagame.exe <scene> --set streaming.*=...` from the repo root.

```bat
dotnet run --project demo\GagameDemo
dotnet publish demo\GagameDemo -c Release -r win-x64 --self-contained false
```

`GAGAME.sln` at the repo root holds this project and `tools\Scriptorium`; the engine itself stays
a CMake + Ninja build (`build.bat`), and the demo expects that to have run.

## What opens

The scene bar, always. A banner under it names what a fresh machine still lacks (the engine not
built, datasets missing, no Google key) with `Build engine` and `Setup...` beside it; `Dismiss`
hides it until the list changes. `Setup` is a dropdown: cache and limits, the key, the data
files (each opens the install form scrolled to that section), build the engine (`build.bat`,
logged in the window), choose another checkout, open the checkout folder.

## Install form (`Setup` dropdown)

- **Checkout.** The repo and whether `build\bin\gagame.exe` exists; `Build engine` runs `build.bat`.
- **Cache.** The engine reads `cache\` relative to the checkout. `Link` makes that a directory
  junction (`mklink /J`, no admin) to the folder you pick on a big disk. A `cache\` that is already
  a folder with files in it is refused and said so: moving data is the owner's job. `Measure` sizes
  it; `List stale trees` runs the engine's `--tree-prune` listing (changes nothing).
- **Limits.** The Google tile caps are the scene's `streaming.tileBudget`, `dayTiles`, `dayBytes`
  and `googleZoom`; the demo passes them as `--set` on every launch, so they override whatever the
  scene file says.
- **Key.** The engine reads `GAGAME_GOOGLE_MAPS_KEY` from the environment, else from
  `HKCU\Environment`. `Save key` writes that user variable (what `setx` does) and nothing in the
  checkout; the status shows length and prefix only. With no key the form says how to get one.
- **Data.** Every product the engine reads, the harvester that writes it, where it comes from and
  whether its output is present. `Download` runs the harvester with its defaults (`py -3 -I
  harvester\<script>` from the checkout); `Source` opens the provider; `Insert...` runs it for a
  custom place and box, a named window, files dropped into its cache folder, a file argument, or
  skip flags, whichever that harvester supports (`Datasets.cs` is the catalog).

Settings live in `%LOCALAPPDATA%\GAGAME\demo.json`; the key never does.

## Scene bar

`Scene` lists `scenes\*.json` and `scenes\demos\*.json`, titled by each file's `_readme`. Picking
one stops the running engine and boots the next (the restart waits for the first exit, so the GPU
and the cache's files are released first). `Relaunch`, `Stop`, `Open scene file` (edit it: the
engine hot-reloads what is hot) and `Controls` (`docs/LAUNCH.md`) are beside it; the engine's
log is the window.
