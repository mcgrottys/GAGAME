// ================================================================================================
//  ExitTrail - A LINE PER SHUTDOWN PHASE (docs/REVIEW_2026-09-28.md findings 33 and 44).
//
//  The engine sometimes exits 255 or dies in its teardown, and nothing says where. The review
//  names four candidates: an exception in Frame() skips Finish(), so loads run against trees
//  already destroyed; DirectStorage reads are not drained at exit; ~WaveField can wait for a job
//  the pool dropped; and an atlas releases its heaps with queue work still naming them. So every
//  phase of shutdown says its name BEFORE it runs, through Log, which flushes each line as it
//  writes it: a death inside a phase leaves that phase's line the last one in the log. Always on --
//  a dozen lines at exit cost nothing, and the run that dies is never the run that had the flag.
//
//  ExitStep marks the explicit calls (FrameLoop::Finish, main). ExitMark marks a destructor that
//  runs by itself: declared right AFTER the member it announces, it destructs right BEFORE it,
//  because members die in the reverse of their declaration order -- on the normal path and on an
//  exception's unwind alike, and the unwind is the path finding 33 is about.
// ================================================================================================
#pragma once

#include "core/Common.h"

namespace ga {

inline void ExitStep(const char* what) { Log("[exit] %s", what); }

struct ExitMark {
    const char* what = nullptr;
    explicit ExitMark(const char* w) : what(w) {}
    ExitMark(const ExitMark&) = delete;
    ExitMark& operator=(const ExitMark&) = delete;
    ~ExitMark() {
        if (what) ExitStep(what);
    }
};

}  // namespace ga
