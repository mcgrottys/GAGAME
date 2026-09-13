"""hal_lint -- the HAL rule, as a gate (M12 step 3f).

Direct3D 12 lives in src/hal/ and nowhere else. Outside that folder no file may name a D3D12 /
DXGI / DirectStorage / DXC COM interface, a D3D12 descriptor struct, or a resource barrier: those
are the hardware-bound surfaces the facades in src/hal/ exist to hold. The plain enums and
typedefs (DXGI_FORMAT, D3D12_GPU_VIRTUAL_ADDRESS, D3D12_RESOURCE_STATES) stay usable everywhere --
this engine is DX12-first, and an enum of its own would be a portability tax nobody asked for.
What a layer HOLDS it spells as the hal typedef of the same type (hal/Gpu.h: hal::Pso,
hal::RootSignature, hal::Resource and their Ref forms); what it DOES it says through a facade
(hal/Context.h, Pipeline.h, Root.h, Views.h, Resources.h, Tenant.h).

    py -3 tools/hal_lint.py            exit 0 and the PASS line, or exit 1 and every offending line
    py -3 tools/hal_lint.py --plant    self-check: the same walk with one planted violation added,
                                       which must be caught (exit 0 when it is, 2 when it is not)

The self-check exists because a lint that has never been seen to fail has not been asked the
question (ALGEBRA priors 22). It runs the real walk, so it also proves the walk still reads the
tree: a lint that finds nothing because it looked nowhere would fail its own plant.

THE ALLOWLIST is empty. An entry names a file and the offending line's text, with why no hal
spelling of the same type and no facade method fits without changing behaviour; the PASS line
prints how many there are, so an allowlist that grows is seen to grow. (The tool never scans
itself: only .h/.hpp/.cpp under src/ are read, so the examples in this docstring are not hits.)
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, 'src')
HAL = os.path.join(SRC, 'hal')
EXTS = ('.h', '.hpp', '.cpp')

# (file relative to the repo root, forward slashes; the offending line, stripped) -> why.
ALLOW = {
}

# The forbidden spellings, as one alternation. COM interfaces first, then the descriptor
# structs and the barrier; DXGI_FORMAT / D3D12_GPU_VIRTUAL_ADDRESS / D3D12_RESOURCE_STATES stay
# out of the match by not being listed.
FORBIDDEN = re.compile(
    r'\b(ID3D12\w*|IDXGI\w*|IDStorage\w*|IDxc\w*|'
    r'D3D12_\w*_DESC\d*|D3D12_RESOURCE_BARRIER\w*|D3D12_TILE_RANGE_FLAGS|'
    r'D3D12_TILED_RESOURCE_COORDINATE|D3D12_TILE_REGION_SIZE|D3D12_SUBRESOURCE_TILING|'
    r'DSTORAGE_\w+)\b')

# The self-check's plant: a file that does not exist, holding the widest leak step 3b closed.
PLANT_FILE = 'src/scene/__planted__.cpp'
PLANT_LINE = 'ID3D12GraphicsCommandList* cl = nullptr;   // planted'


def scan(text):
    hits = []
    for n, line in enumerate(text.splitlines(), 1):
        code = line.split('//', 1)[0]          # a comment may name the enemy
        m = FORBIDDEN.search(code)
        if m:
            hits.append((n, m.group(1), line.strip()))
    return hits


def sources():
    """Every (path relative to the root, text) under src/ outside src/hal/."""
    out = []
    for dirpath, _, files in os.walk(SRC):
        if os.path.abspath(dirpath).startswith(os.path.abspath(HAL)):
            continue
        for f in sorted(files):
            if not f.endswith(EXTS):
                continue
            path = os.path.join(dirpath, f)
            with open(path, 'r', encoding='utf-8', errors='replace') as fh:
                out.append((os.path.relpath(path, ROOT).replace('\\', '/'), fh.read()))
    return out


def check(files):
    """The verdict over (rel, text) pairs: the offending lines not allowed, and how many were."""
    bad, allowed = [], 0
    for rel, text in files:
        for n, sym, line in scan(text):
            if (rel, line) in ALLOW:
                allowed += 1
            else:
                bad.append((rel, n, sym, line))
    return bad, allowed


def main(argv):
    files = sources()
    if '--plant' in argv:
        files.append((PLANT_FILE, PLANT_LINE + '\n'))
        bad, _ = check(files)
        caught = [h for h in bad if h[0] == PLANT_FILE]
        for rel, n, sym, line in caught:
            print(f'{rel}:{n}: {sym}    {line[:100]}')
        print(f'[hal-lint] self-check: planted line {"CAUGHT" if caught else "MISSED"} '
              f'({len(bad)} line(s) would FAIL the run; {len(files) - 1} real files walked)')
        return 0 if caught else 2
    bad, allowed = check(files)
    for rel, n, sym, line in bad:
        print(f'{rel}:{n}: {sym}    {line[:100]}')
    print(f'[hal-lint] {len(bad)} D3D12 line(s) outside src/hal/ in {len(files)} files, '
          f'{allowed} allowed ({len(ALLOW)} allowlist entries) -- {"FAIL" if bad else "PASS"}')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
