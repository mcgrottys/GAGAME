#!/usr/bin/env python3
# M7v: the whitepaper's own gate. Every `Code:` anchor in docs/ALGEBRA.md must name files
# that exist; every `## id — Title` section must ingest into the scriptorium's math table
# (same parser rules); the AST doc and JSON must agree on edge count. Run after editing
# ALGEBRA.md; CI-ready (exit 1 on findings).
import io
import json
import os
import re
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ok = True

md = io.open(os.path.join(REPO, 'docs', 'ALGEBRA.md'), encoding='utf-8').read()

# 1. section ids parse exactly like the scriptorium's IndexMath
ids = re.findall(r'^## ([a-z0-9-]+) (?:—|--) (.+)$', md, re.M)
print('[algebra_lint] %d sections: %s' % (len(ids), ', '.join(i for i, _ in ids)))
if len(ids) < 13:
    print('[algebra_lint] FAIL: expected >= 13 sections')
    ok = False

# 2. file anchors exist (tokens with a source extension inside Code:/anchor lines)
names = set()
for root in ('src', 'shaders', 'harvester', 'tools', 'proofs', 'docs'):
    for dirpath, _, files in os.walk(os.path.join(REPO, root)):
        names.update(files)
missing = []
for tok in re.findall(r'`([^`]+)`', md):
    for part in re.split(r'[ ,()]+', tok):
        m = re.match(r'^([\w./\\-]+\.(?:hlsl|hlsli|cpp|h|py|md|json|svg))'
                     r'(?::\d+)?$', part)
        if not m:
            continue
        base = os.path.basename(m.group(1))
        if '*' in base or base.endswith('.*'):
            continue
        if base not in names:
            missing.append(part)
for tok in sorted(set(missing)):
    # WaterAtlas.* style globs: check stem exists with any of the extensions
    stem = re.sub(r'\.\w+$', '', os.path.basename(tok))
    if any(n.startswith(stem + '.') for n in names):
        continue
    print('[algebra_lint] FAIL: anchor cites missing file %s' % tok)
    ok = False

# 3. the AST doc/table/json agree
try:
    j = json.load(io.open(os.path.join(REPO, 'docs', 'ga_ast.json'), encoding='utf-8'))
    table = io.open(os.path.join(REPO, 'docs', 'GA_AST.md'), encoding='utf-8').read()
    rows = len(re.findall(r'^\| [a-z]', table, re.M))
    if rows != len(j['edges']):
        print('[algebra_lint] FAIL: GA_AST.md has %d rows but ga_ast.json %d edges '
              '(stale generation?)' % (rows, len(j['edges'])))
        ok = False
    else:
        print('[algebra_lint] AST doc == json: %d edges' % rows)
except FileNotFoundError as e:
    print('[algebra_lint] FAIL: %s' % e)
    ok = False

print('[algebra_lint] %s' % ('PASS' if ok else 'FAIL'))
sys.exit(0 if ok else 1)
