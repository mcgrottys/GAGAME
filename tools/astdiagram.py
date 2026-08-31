#!/usr/bin/env python3
# ================================================================================================
#  astdiagram.py - the GA state diagram, drawn. Reads docs/ga_ast.json (emitted by the engine
#  every boot: the SAME registry gatest validates) and renders Blueprint-style SVG: nodes as
#  titled boxes with ports, edges as labeled wires, a crimson badge where the sampling code
#  flips v, dashed wires for inactive consumers. The JSON is the contract a future node-editor
#  UI (the Blueprint ambition, GAMEPLAN M7u) will load and save; this renderer and that editor
#  share one file format by design.
#
#  Usage:  py -3 tools/astdiagram.py            -> docs/diagrams/ga_full.svg + per-domain pages
# ================================================================================================
import json
import io
import os
import math

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(REPO, 'docs', 'ga_ast.json')
OUT = os.path.join(REPO, 'docs', 'diagrams')

# Domain palette: header fill / wire tint. Assigned by node-name prefix, first match wins.
DOMAINS = [
    ('water',   ['water.', 'ocean.', 'swe.', 'churn.', 'sea.', 'bathy.', 'noaa.', 'eot20.',
                 'gfswave.'],                     '#1f6feb', '#3b82c4'),
    ('compose', ['compose.', 'window.', 'cube.', 'google.', 'massgis.', 'synth.', 'height.',
                 'survey.', 'gis.'],              '#2f8f46', '#4a9e63'),
    ('air',     ['gfs.', 'cloud.', 'mv2.'],       '#8250df', '#9d70e0'),
    ('render',  ['globe.', 'frame.', 'residency.'], '#b8860b', '#c09a3e'),
    ('frames',  ['world.', 'latlon.', 'weather.'], '#57606a', '#6e7781'),
]

def domain_of(node):
    for name, prefixes, head, wire in DOMAINS:
        if any(node.startswith(p) for p in prefixes):
            return name, head, wire
    return 'frames', '#57606a', '#6e7781'

def esc(s):
    return (s.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')
             .replace('"', '&quot;'))

def layout(nodes, edges):
    # Longest-path ranks (DAG-ish; capped relaxation for safety).
    rank = {n: 0 for n in nodes}
    for _ in range(len(nodes)):
        moved = False
        for e in edges:
            if rank[e['to']] < rank[e['from']] + 1:
                rank[e['to']] = rank[e['from']] + 1
                moved = True
        if not moved:
            break
    cols = {}
    for n in nodes:
        cols.setdefault(rank[n], []).append(n)
    # Two barycenter passes order rows to tame crossings.
    order = {n: i for r in cols for i, n in enumerate(sorted(cols[r]))}
    for _ in range(2):
        for r in sorted(cols):
            def bary(n):
                prev = [order[e['from']] for e in edges if e['to'] == n]
                nxt = [order[e['to']] for e in edges if e['from'] == n]
                pool = prev + nxt
                return sum(pool) / len(pool) if pool else order[n]
            cols[r].sort(key=bary)
            for i, n in enumerate(cols[r]):
                order[n] = i
    # Pixel positions. Node height grows with its port count.
    NW, GX, GY, PAD = 200, 170, 34, 46
    pos, size = {}, {}
    for r in sorted(cols):
        y = PAD + 30
        for n in cols[r]:
            ins = sum(1 for e in edges if e['to'] == n)
            outs = sum(1 for e in edges if e['from'] == n)
            h = 30 + max(ins, outs, 1) * 15 + 8
            pos[n] = (PAD + r * (NW + GX), y)
            size[n] = (NW, h)
            y += h + GY
    width = PAD * 2 + (max(cols) + 1) * (NW + GX) - GX
    height = max(pos[n][1] + size[n][1] for n in nodes) + PAD
    return pos, size, width, height

def bez(x1, y1, x2, y2):
    dx = max(55, (x2 - x1) * 0.42)
    return (x1, y1, x1 + dx, y1, x2 - dx, y2, x2, y2)

def bez_at(p, t):
    x1, y1, cx1, cy1, cx2, cy2, x2, y2 = p
    mt = 1 - t
    x = mt**3 * x1 + 3 * mt**2 * t * cx1 + 3 * mt * t**2 * cx2 + t**3 * x2
    y = mt**3 * y1 + 3 * mt**2 * t * cy1 + 3 * mt * t**2 * cy2 + t**3 * y2
    return x, y

def render(edges, title, out):
    nodes = sorted(set(e['from'] for e in edges) | set(e['to'] for e in edges))
    pos, size, width, height = layout(nodes, edges)
    s = []
    s.append(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height + 70}" '
             f'font-family="Segoe UI, Arial, sans-serif">')
    s.append(f'<rect width="{width}" height="{height + 70}" fill="#f6f5f1"/>')
    s.append(f'<text x="{width/2:.0f}" y="30" text-anchor="middle" font-size="19" '
             f'font-weight="600" fill="#24292f">{esc(title)}</text>')
    s.append(f'<text x="{width/2:.0f}" y="48" text-anchor="middle" font-size="11" '
             f'fill="#57606a">generated from docs/ga_ast.json — the registry gatest '
             f'validates; wires carry field [units], crimson badge = v-flip in the '
             f'sampling code, dashed = inactive consumer</text>')
    # Port slots per node: outgoing on the right, incoming on the left, stacked in edge order.
    out_i = {n: 0 for n in nodes}
    in_i = {n: 0 for n in nodes}
    wires = []
    for k, e in enumerate(edges):
        a, b = e['from'], e['to']
        ax, ay = pos[a]; aw, ah = size[a]
        bx, by = pos[b]; bw, bh = size[b]
        oy = ay + 38 + out_i[a] * 15; out_i[a] += 1
        iy = by + 38 + in_i[b] * 15; in_i[b] += 1
        p = bez(ax + aw, oy, bx, iy)
        _, _, wire = domain_of(a)
        dash = ' stroke-dasharray="6 5"' if not e.get('active', True) else ''
        wires.append(f'<path d="M {p[0]:.0f} {p[1]:.0f} C {p[2]:.0f} {p[3]:.0f}, '
                     f'{p[4]:.0f} {p[5]:.0f}, {p[6]:.0f} {p[7]:.0f}" fill="none" '
                     f'stroke="{wire}" stroke-width="1.6" opacity="0.75"{dash}/>')
        wires.append(f'<circle cx="{p[0]:.0f}" cy="{p[1]:.0f}" r="3.2" fill="{wire}"/>')
        wires.append(f'<circle cx="{p[6]:.0f}" cy="{p[7]:.0f}" r="3.2" fill="{wire}"/>')
        mx, my = bez_at(p, 0.5)
        my += -7 if k % 2 == 0 else 13
        label = e['field']
        lw = 5.4 * len(label) + 10
        wires.append(f'<rect x="{mx - lw/2:.0f}" y="{my - 10:.0f}" width="{lw:.0f}" '
                     f'height="13" rx="3" fill="#f6f5f1" opacity="0.9"/>')
        wires.append(f'<text x="{mx:.0f}" y="{my:.0f}" text-anchor="middle" font-size="9.5" '
                     f'fill="#454c54">{esc(label)}</text>')
        if e.get('flip'):
            wires.append(f'<circle cx="{mx + lw/2 + 8:.0f}" cy="{my - 4:.0f}" r="7" '
                         f'fill="#c93c37"/>')
            wires.append(f'<text x="{mx + lw/2 + 8:.0f}" y="{my - 1:.0f}" '
                         f'text-anchor="middle" font-size="9" fill="#fff">&#8645;</text>')
    s.extend(wires)
    for n in nodes:
        x, y = pos[n]; w, h = size[n]
        _, head, _ = domain_of(n)
        s.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="8" fill="#ffffff" '
                 f'stroke="#d0d7de" stroke-width="1.2"/>')
        s.append(f'<path d="M {x} {y+8} q 0 -8 8 -8 h {w-16} q 8 0 8 8 v 18 h -{w} z" '
                 f'fill="{head}"/>')
        s.append(f'<text x="{x + w/2}" y="{y + 18}" text-anchor="middle" font-size="12" '
                 f'font-weight="600" fill="#ffffff">{esc(n)}</text>')
    # Legend.
    ly = height + 28
    lx = 46
    for name, _, head, _ in DOMAINS:
        s.append(f'<rect x="{lx}" y="{ly}" width="14" height="14" rx="3" fill="{head}"/>')
        s.append(f'<text x="{lx + 20}" y="{ly + 11}" font-size="11" '
                 f'fill="#24292f">{name}</text>')
        lx += 26 + 7 * len(name) + 22
    s.append('</svg>')
    with io.open(out, 'w', encoding='utf-8') as f:
        f.write('\n'.join(s))
    print('wrote %s  (%d nodes, %d edges)' % (os.path.relpath(out, REPO), len(nodes),
                                              len(edges)))

def main():
    os.makedirs(OUT, exist_ok=True)
    d = json.load(io.open(SRC, encoding='utf-8'))
    edges = d['edges']
    render(edges, 'GAGAME — the GA state diagram (full catalog)',
           os.path.join(OUT, 'ga_full.svg'))
    for dom, keep in [('water', DOMAINS[0][1] + ['weather.', 'compose.', 'globe.',
                                                 'height.', 'world.', 'latlon.',
                                                 'window.field']),
                      ('compose', DOMAINS[1][1] + ['residency.', 'globe.']),
                      ('air', DOMAINS[2][1] + ['globe.'])]:
        sub = [e for e in edges
               if any(e['from'].startswith(p) for p in keep)
               and any(e['to'].startswith(p) for p in keep)]
        render(sub, 'GAGAME — %s domain' % dom, os.path.join(OUT, 'ga_%s.svg' % dom))

if __name__ == '__main__':
    main()
