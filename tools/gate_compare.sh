#!/usr/bin/env bash
# Two gate-stills folders side by side (tools/gate_stills.sh output): the pixel verdict per pose
# (tools/imgdiff.py), then the resident-set fingerprints the logs carry.
#
# STRICT half: the per-tenant [settle-exact] mapped-set FNV-1a and the [predict] want stream --
# the resident set IS the picture's input (ALGEBRA priors 30), and it is what stays equal where
# the picture itself has a run-to-run floor (the helm's horizon strip, the ebb helm's churn).
# INFORMATIONAL: the [jobs] submission hash, which folds worker timing and is equal only under
# --jobs-inline. Exit 1 if any resident set or predicted stream differs.
#
#   tools/gate_compare.sh BASE_DIR NEW_DIR [DIFF_DIR]
BASE="$1"; NEW="$2"; DIFF="${3:-$NEW/diff}"
[ -d "$BASE" ] && [ -d "$NEW" ] || { echo "usage: gate_compare.sh BASE_DIR NEW_DIR [DIFF_DIR]" >&2; exit 2; }
mkdir -p "$DIFF"
echo "== pixels: $BASE vs $NEW =="
py -3 "$(dirname "$0")/imgdiff.py" --pairs "$BASE" "$NEW" --out-dir "$DIFF" --no-ssim
echo "== resident sets (strict) and job streams (informational) =="
rc=0
strict() {   # the tenant names and their mapped-set hashes, plus the predicted stream
    grep -hE '^\[settle-exact\]   |^\[predict\]' "$1" | grep -oE '[a-z]+\.[a-z]+ \(|FNV-1a [0-9a-f]+' | tr '\n' ' '
}
jobs_hash() { grep -hE '^\[jobs\] pool joined' "$1" | grep -oE 'hash [0-9a-f]+'; }
for log in "$BASE"/*.log; do
    name=$(basename "$log")
    case "$name" in *warmup*) continue;; esac
    if [ ! -f "$NEW/$name" ]; then echo "  $name: missing in NEW"; rc=1; continue; fi
    a=$(strict "$log"); b=$(strict "$NEW/$name")
    if [ "$a" == "$b" ]; then
        echo "  $name: resident sets + predicted stream EQUAL   (jobs: $(jobs_hash "$log") vs $(jobs_hash "$NEW/$name"))"
    else
        echo "  $name: RESIDENT SETS DIFFER"
        echo "    base: $a"
        echo "    new:  $b"
        rc=1
    fi
done
[ $rc -eq 0 ] && echo "[gate] resident sets: PASS" || echo "[gate] resident sets: FAIL"
exit $rc
