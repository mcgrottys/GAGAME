#include "core/BuildInfo.h"

// GA_GIT_REV is set on this one translation unit by CMakeLists.txt (configure time), so a
// revision change recompiles this file alone.
#ifndef GA_GIT_REV
#define GA_GIT_REV "unknown"
#endif

namespace ga {

const char* BuildGitRev() { return GA_GIT_REV; }

}  // namespace ga
