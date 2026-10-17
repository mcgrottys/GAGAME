# tools/astindex/launch.py -- the post-build hook. Starts tools/codemap/run.py (this index, then
# clangd's, then the code map in build/codemap/) in the background and returns at once, so the build
# never waits on them.
#
#   py -3 tools/astindex/launch.py <build dir>
#
# Why each flag: CREATE_NO_WINDOW, not DETACHED_PROCESS -- a detached process has no console, so
# every pool worker and dxc.exe it starts would open a console window of its own; a hidden console
# is inherited instead. CREATE_NEW_PROCESS_GROUP keeps a Ctrl+C in the build's console from
# reaching it. stdout/stderr go to the log file and stdin to NUL, because Ninja waits until every
# holder of its output pipe has closed it, and an inherited pipe would make the build wait for the
# whole index. CREATE_BREAKAWAY_FROM_JOB lets it outlive a build run inside a job object (an IDE);
# where the job forbids breakaway the launch is retried without it. BELOW_NORMAL_PRIORITY_CLASS,
# which its pool workers and dxc inherit, and at most 8 processes: an index that catches up while
# the game runs must not take the game's cores.
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CREATE_NEW_PROCESS_GROUP = 0x00000200
CREATE_NO_WINDOW = 0x08000000
CREATE_BREAKAWAY_FROM_JOB = 0x01000000
BELOW_NORMAL_PRIORITY_CLASS = 0x00004000


def main():
    build = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "..", "..", "build"))
    path = os.path.join(build, "codemap.log")
    big = os.path.exists(path) and os.path.getsize(path) > 1 << 20
    log = open(path, "w" if big else "a", encoding="utf-8")   # append: a run may still be writing it
    cmd = [sys.executable, os.path.join(HERE, "..", "codemap", "run.py"), build,
           "--jobs", str(min(8, os.cpu_count() or 1))]
    base = CREATE_NEW_PROCESS_GROUP | CREATE_NO_WINDOW | BELOW_NORMAL_PRIORITY_CLASS
    for flags in (base | CREATE_BREAKAWAY_FROM_JOB, base):
        try:
            subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                             creationflags=flags, close_fds=True, cwd=build)
            print(f"[codemap] indexing and mapping in the background; log: {path}")
            return
        except OSError:
            continue
    print("[codemap] could not start the indexer (the build is unaffected)")


if __name__ == "__main__":
    main()
