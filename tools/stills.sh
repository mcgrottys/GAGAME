#!/usr/bin/env bash
# The five settled stills, one binary, one folder. $1 = exe, $2 = out dir.
# --settle-exact is the only hold that names a resident set (ALGEBRA priors 30): a drained
# queue is a timing statement, not a residency one.
EXE="$1"; OUT="$2"; mkdir -p "$OUT"
C="--sea --one-water --headless --frames 240 --storm 3.0,10,95 --settle-exact"
"$EXE" $C --campos 120,-10 --cam 7,92.5,-1.5 --start 2026-08-28T14:00:00 --dump "$OUT/helm.png"     > "$OUT/helm.log" 2>&1
"$EXE" $C --campos 120,-10 --cam 7,92.5,-1.5 --start 2026-08-28T19:30:00 --dump "$OUT/helm_ebb.png" > "$OUT/helm_ebb.log" 2>&1
"$EXE" $C --campos 380,10 --cam 1500,272,-88 --start 2026-08-28T14:00:00 --dump "$OUT/bird.png"     > "$OUT/bird.log" 2>&1
"$EXE" $C --globe-cam 42.74,-70.87,7         --start 2026-08-28T14:00:00 --dump "$OUT/key7km.png"   > "$OUT/key7km.log" 2>&1
"$EXE" $C --campos 0,0 --cam 200000,0,0      --start 2026-08-28T14:00:00 --dump "$OUT/globe.png"    > "$OUT/globe.log" 2>&1
ls -la "$OUT"/*.png
