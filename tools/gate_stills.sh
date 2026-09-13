#!/usr/bin/env bash
# The M12 gate stills for ONE binary: the five tools/stills.sh poses plus the droste pose, each
# rendered twice with the warm-up discarded (the A/A floor collapses to the real one only then --
# see stills.sh), all under --settle-exact so a still is a function of pose and data alone.
#
#   tools/gate_stills.sh EXE OUT_DIR       (from the repo root; EXE needs its DLLs beside it)
#
# Writes OUT_DIR/<pose>.png + .log and OUT_DIR/done when finished. Compare two of these with
# tools/gate_compare.sh. Sequential on purpose: one machine, one GPU, one cache.
EXE="$1"; OUT="$2"
[ -n "$EXE" ] && [ -n "$OUT" ] || { echo "usage: gate_stills.sh EXE OUT_DIR" >&2; exit 2; }
mkdir -p "$OUT"
echo "[gate] $(date +%T) stills -> $OUT"
bash "$(dirname "$0")/stills.sh" "$EXE" "$OUT"
C="--sea --one-water --headless --frames 240 --storm 3.0,10,95 --settle-exact --start 2026-08-28T14:00:00"
D="--droste --droste-twist 90 --campos 632,30 --cam 71,55,8"
"$EXE" $C $D --dump "$OUT/warmup.png" > "$OUT/droste.warmup.log" 2>&1
"$EXE" $C $D --dump "$OUT/droste.png"  > "$OUT/droste.log" 2>&1
rm -f "$OUT/warmup.png"
echo "[gate] $(date +%T) done; shader failures: $(cat "$OUT"/*.log | grep -cE 'COMPILE FAILED|Validation failed')"
touch "$OUT/done"
