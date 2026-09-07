#!/usr/bin/env bash
# The five settled stills, one binary, one folder. $1 = exe, $2 = out dir.
#
# --settle-exact is the only hold that names a resident set (ALGEBRA priors 30): a drained queue
# is a timing statement, not a residency one.
#
# AND EVERY POSE IS RENDERED TWICE, THE FIRST DISCARDED. Measured 2026-09-06 on key7km: over four
# renders of ONE binary the first differs from the rest by max |d| 13 on 3 pixels and renders two,
# three and four are BIT-IDENTICAL to each other. The still converges after one warm-up. A gate
# that renders once per binary is therefore reading a coin toss between "cold vs warm" and "warm
# vs warm", which is how a 3-pixel nothing gets attributed to whatever the commit happened to
# touch. Discard the warm-up and the floor collapses to the real one.
EXE="$1"; OUT="$2"; mkdir -p "$OUT"
C="--sea --one-water --headless --frames 240 --storm 3.0,10,95 --settle-exact"

shot() {  # $1 = name, rest = pose flags
    local name="$1"; shift
    "$EXE" $C "$@" --dump "$OUT/warmup.png" > "$OUT/$name.warmup.log" 2>&1
    "$EXE" $C "$@" --dump "$OUT/$name.png"  > "$OUT/$name.log" 2>&1
}

shot helm     --campos 120,-10 --cam 7,92.5,-1.5 --start 2026-08-28T14:00:00
shot helm_ebb --campos 120,-10 --cam 7,92.5,-1.5 --start 2026-08-28T19:30:00
shot bird     --campos 380,10 --cam 1500,272,-88 --start 2026-08-28T14:00:00
shot key7km   --globe-cam 42.74,-70.87,7         --start 2026-08-28T14:00:00
shot globe    --campos 0,0 --cam 200000,0,0      --start 2026-08-28T14:00:00
rm -f "$OUT/warmup.png"
ls -la "$OUT"/*.png
