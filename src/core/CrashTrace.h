// M7v: symbolized stack traces on unhandled exceptions -- headless crash visibility.
#pragma once

namespace ga {
void InstallCrashTrace();
// MSVC's terminate handler is per thread: every thread the engine starts installs it again.
void InstallThreadCrashTrace();
}
