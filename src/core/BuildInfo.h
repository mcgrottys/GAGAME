// The build names its revision. Every log begins with the git rev and the argv that produced
// it, so a baseline still or a bench log can be matched to the binary and the flags that made
// it without a memory note carrying the recipe (before this, out/baseline/*.log named neither).
#pragma once

namespace ga {

// "<short sha>" from `git rev-parse --short HEAD` at configure time, "-dirty" appended when
// src/, shaders/ or CMakeLists.txt differ from that commit, "unknown" when git was not found.
// build.bat reconfigures on every build, so the string tracks the tree the binary came from.
const char* BuildGitRev();

}  // namespace ga
