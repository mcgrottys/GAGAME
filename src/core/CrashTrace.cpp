// ================================================================================================
//  CrashTrace - a symbolized stack trace on any unhandled exception (M7v). The user asked
//  "doesn't Visual Studio spit out stack traces?" -- it does, interactively; this makes the
//  ENGINE do it headlessly, for every --selftest, rail render, and probe run, with no
//  external tooling: SetUnhandledExceptionFilter + dbghelp (ships with Windows). With PDBs
//  next to the exe the frames carry symbol+file:line; without, module+offset still names the
//  culprit. The output goes through Log, so it lands in the same stream the harness reads.
// ================================================================================================
#include "core/CrashTrace.h"

#include <windows.h>

#include <dbghelp.h>

#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <exception>

#include "core/Common.h"

#pragma comment(lib, "dbghelp.lib")

namespace ga {
namespace {

void LogStack(CONTEXT ctx) {
    const HANDLE proc = GetCurrentProcess();
    SymSetOptions(SYMOPT_LOAD_LINES | SYMOPT_UNDNAME | SYMOPT_DEFERRED_LOADS);
    SymInitialize(proc, nullptr, TRUE);

    STACKFRAME64 frame{};
    frame.AddrPC.Offset = ctx.Rip;
    frame.AddrPC.Mode = AddrModeFlat;
    frame.AddrFrame.Offset = ctx.Rbp;
    frame.AddrFrame.Mode = AddrModeFlat;
    frame.AddrStack.Offset = ctx.Rsp;
    frame.AddrStack.Mode = AddrModeFlat;

    for (int depth = 0; depth < 32; ++depth) {
        if (!StackWalk64(IMAGE_FILE_MACHINE_AMD64, proc, GetCurrentThread(), &frame, &ctx,
                         nullptr, SymFunctionTableAccess64, SymGetModuleBase64, nullptr)) {
            break;
        }
        const DWORD64 pc = frame.AddrPC.Offset;
        if (!pc) break;

        char symBuf[sizeof(SYMBOL_INFO) + 256]{};
        SYMBOL_INFO* sym = reinterpret_cast<SYMBOL_INFO*>(symBuf);
        sym->SizeOfStruct = sizeof(SYMBOL_INFO);
        sym->MaxNameLen = 255;
        DWORD64 disp64 = 0;
        const bool haveSym = SymFromAddr(proc, pc, &disp64, sym);

        IMAGEHLP_LINE64 line{};
        line.SizeOfStruct = sizeof(line);
        DWORD dispL = 0;
        const bool haveLine = SymGetLineFromAddr64(proc, pc, &dispL, &line);

        char modName[MAX_PATH] = "?";
        const DWORD64 modBase = SymGetModuleBase64(proc, pc);
        if (modBase) {
            GetModuleFileNameA(reinterpret_cast<HMODULE>(modBase), modName, MAX_PATH);
            const char* slash = strrchr(modName, '\\');
            if (slash) memmove(modName, slash + 1, strlen(slash + 1) + 1);
        }

        if (haveSym && haveLine) {
            Log("[crash]   #%02d %s!%s +0x%llx  (%s:%lu)", depth, modName, sym->Name,
                static_cast<unsigned long long>(disp64), line.FileName,
                static_cast<unsigned long>(line.LineNumber));
        } else if (haveSym) {
            Log("[crash]   #%02d %s!%s +0x%llx", depth, modName, sym->Name,
                static_cast<unsigned long long>(disp64));
        } else {
            Log("[crash]   #%02d %s +0x%llx", depth, modName,
                static_cast<unsigned long long>(pc - modBase));
        }
    }
    Log("[crash] end of stack");
}

LONG WINAPI OnCrash(EXCEPTION_POINTERS* ep) {
    const DWORD code = ep && ep->ExceptionRecord ? ep->ExceptionRecord->ExceptionCode : 0;
    Log("[crash] unhandled exception 0x%08lx -- stack:", static_cast<unsigned long>(code));
    LogStack(*ep->ContextRecord);
    return EXCEPTION_EXECUTE_HANDLER;   // terminate after reporting, no WER dialog stall
}

}  // namespace

// An exception no one catches (a pool job's throw) ends in std::terminate -> abort, a fail-fast
// (0xC0000409) the filter above never sees: name the exception before the process goes.
[[noreturn]] void OnTerminate() {
    const char* what = "(no exception in flight)";
    try {
        if (std::exception_ptr e = std::current_exception()) std::rethrow_exception(e);
    } catch (const std::exception& ex) {
        what = ex.what();
    } catch (...) {
        what = "(not a std::exception)";
    }
    Log("[crash] std::terminate on thread %lu: %s", static_cast<unsigned long>(GetCurrentThreadId()), what);
    std::abort();
}

// The CRT's other fail-fasts (0xC0000409 again): an invalid parameter and abort() itself. Each is
// named before the process goes, since neither passes the filter above.
void OnInvalidParameter(const wchar_t*, const wchar_t*, const wchar_t*, unsigned, uintptr_t) {
    Log("[crash] CRT invalid parameter on thread %lu", static_cast<unsigned long>(GetCurrentThreadId()));
    std::abort();
}

// MSVC keeps the terminate handler PER THREAD: a pool worker's escaping exception ends in the
// default terminate -> abort(), past OnTerminate. So abort() names the exception in flight on
// its own thread, if any, and walks the stack it was called from.
void OnAbort(int) {
    const char* what = "(no exception in flight)";
    try {
        if (std::exception_ptr e = std::current_exception()) std::rethrow_exception(e);
    } catch (const std::exception& ex) {
        what = ex.what();
    } catch (...) {
        what = "(not a std::exception)";
    }
    Log("[crash] abort() on thread %lu: %s -- stack:", static_cast<unsigned long>(GetCurrentThreadId()), what);
    CONTEXT ctx{};
    RtlCaptureContext(&ctx);
    LogStack(ctx);
}

void InstallThreadCrashTrace() { std::set_terminate(OnTerminate); }

void InstallCrashTrace() {
    SetUnhandledExceptionFilter(OnCrash);
    std::set_terminate(OnTerminate);
    _set_invalid_parameter_handler(OnInvalidParameter);
    std::signal(SIGABRT, OnAbort);
}

}  // namespace ga
