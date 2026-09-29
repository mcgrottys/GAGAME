// ================================================================================================
//  TreePrune - THE ONE TOOL IN THIS PROJECT WHOSE PURPOSE IS TO REMOVE DATA, so most of it is a
//  list of what it must never do.
//
//  WHAT IT MAY TOUCH. cache\trees holds DERIVED data: a TileTree folder is painted from the source
//  caches, and a tree that is gone repaints from them on its next flight, with no fetch. That is
//  the only reason pruning is thinkable, and nothing outside cache\trees is derived in that sense.
//  Every change of a source or of the painting code makes a new tree identity and the old folder
//  stays, so the folder grows without bound unless something can say which trees are still used.
//
//  WHAT SAYS SO. Every engine run stamps the tag folders it ensures (TileTree.h, StampLive):
//  <node>.<id>\<tag>\.live, the UTC time, the rev and the scene. The unit is the TAG folder, since
//  a lattice the engine stopped using goes stale inside a node folder that is still live. A folder
//  with no stamp predates stamping; it is judged by its newest write and given twice as long:
//
//      KEEP              stamped within the age
//      STALE             stamped, and the stamp is older than the age
//      KEEP-UNSTAMPED    no stamp, and a file in it was written within twice the age
//      STALE-UNSTAMPED   no stamp, and nothing in it was written within twice the age
//
//  THE MODES. The default is the one that cannot change anything.
//      list    reads directory entries and each .live stamp's bytes; opens nothing else, writes
//              nothing, and never recurses: a tag folder is one level of files.
//      retire  moves every STALE and STALE-UNSTAMPED tag folder, with its belongings, into
//              <root>\.retired\<UTC stamp>\<node>.<id>\<tag>, by ONE rename each. A tag folder's
//              belongings sit BESIDE it in the node folder: <tag>.gaa, the archive packed from it
//              (review finding 25: a leaf is served from the archive before the loose files),
//              and <tag>.gaa.stale, an archive set aside by hand (2026-09-28) because it held tiles
//              its loose files no longer hold. The three move together or not at all: a part that
//              will not move sends the parts before it back. MoveFileExW without
//              MOVEFILE_COPY_ALLOWED: another volume fails, it never becomes a copy. Nothing is
//              deleted, a node folder left empty stays, and <batch>\manifest.json names every move
//              and its undo.
//      purge   deletes the retired batches older than a second age -- and only batches retire
//              made, which carry the manifest it writes before its first move; any other folder
//              under .retired is SKIPPED, whatever its name says. The only path here that deletes,
//              and the only one that recurses (WalkTree).
//
//  WHAT IT REFUSES, each with a [prunetest] case:
//    - A root whose FINAL path (every junction on the way resolved, GetFinalPathNameByHandleW) is
//      not ...\cache\trees, or a folder named trees strictly inside the worktree's own out\. A
//      mistyped key can then never point it at cache\google or at a drive's root, and the path
//      it prints -- the one prune.confirm must name -- is where a change would physically land.
//    - Anything directly under the root not named <name>.<8 lowercase hex> (a note left there by
//      hand, the old earth.color.composite), and inside those, anything but a tag folder and its
//      two archives: printed SKIPPED, never touched. A file beside a tag folder whose name starts
//      <tag>. and is neither archive is not understood, so that tag folder is SKIPPED whole and
//      nothing of it is separated. .retired is never listed as a tree.
//    - Every reparse point (junction, symlink, mount point), in every mode: not entered, not
//      moved, not deleted. A tag folder holding one -- or holding any folder, since a tag folder
//      holds files -- is SKIPPED whole, so the listing never needs to recurse to know it is safe.
//      The walk checks again at every level before it descends.
//    - retire or purge while another gagame*.exe is running, or when the process table cannot be
//      read: a tree in use must not move under its reader. Checked before the first change and
//      again before each one; each folder is re-read and re-judged just before it moves.
//    - retire or purge without prune.confirm set to the root's final path as the listing printed
//      it: that run lists and exits. A confirm naming anything else is a refusal.
//    - An age under a day (a mistyped key would otherwise call every tree stale), or a negative
//      purge age.
//    - A failed move or delete stops the run at that item and says which; the items after it are
//      where they were. A folder whose archive will not follow it is put back first.
//
//  WHAT IT CANNOT SEE. A binary built before the stamp existed reads trees without stamping them.
//  A tree read every day by such a binary and written never looks unused after twice the age,
//  and a stamped one read only by such binaries looks STALE after the age. Retire deletes
//  nothing, so either costs a repaint, not the data; purge is a later, separate decision.
// ================================================================================================
#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace ga::prune {

enum class Mode : uint8_t { List, Retire, Purge };
enum class Verdict : uint8_t { Keep, Stale, KeepUnstamped, StaleUnstamped };
const char* VerdictName(Verdict v);

// How a run ended. The tool's exit code folds it: 0 listed or done, 2 refused with nothing
// changed, 1 stopped at a failure after what it reports.
enum class Status : uint8_t {
    Listed,           // list; or retire/purge with no prune.confirm: nothing was changed
    Done,             // retire/purge: every planned item done (or skipped, and said why)
    RefusedAge,       // prune.ageDays under 1, prune.purgeDays under 0, or not a number
    RefusedRoot,      // the root is not a folder, or fails the rule in the banner
    RefusedConfirm,   // prune.confirm names another path than the root's final one
    RefusedEngines,   // another engine is running, or the process table could not be read
    RefusedBatch,     // .retired is not a plain folder, or the batch could not be made new
    RefusedVolume,    // the batch folder is not on the root's volume
    Stopped,          // a move or a delete failed, or the tree changed under the run
};
const char* StatusName(Status s);
int ExitCode(Status s);

// One running process, as the table lists it.
struct Process {
    uint32_t pid = 0;
    std::string image;   // "gagame.exe"
};

// One <node>.<id>\<tag> folder, as the listing reads it. Times are FILETIME ticks (UTC).
struct Entry {
    std::string node, id, tag;
    std::wstring folder;          // <root>\<node>.<id>\<tag>
    std::wstring gaa;             // <root>\<node>.<id>\<tag>.gaa, or empty when there is none
    std::wstring stale;           // <root>\<node>.<id>\<tag>.gaa.stale, or empty
    uint64_t tiles = 0;           // files named like a tile: f<face>_m<mip>_x<x>_y<y>[_<key>].<kind>
    uint64_t files = 0;           // every file in the folder, the stamp included
    uint64_t bytes = 0;           // those files' bytes plus both archives'
    uint64_t newest = 0;          // the newest write of any file but the stamp (the archives
                                  // included); the folder's own when it holds no other file
    bool stamped = false;
    bool stampRead = false;       // its JSON parsed; else the stamp's file time stands for it
    uint64_t stampWritten = 0;    // the .live file's own last write
    uint64_t stampTime = 0;       // the later of the stamp's utc and its file time
    std::string stampRev, stampScene, stampFile;
    Verdict verdict = Verdict::Keep;
};
struct Skipped {
    std::wstring path;
    std::string why;
};
// One retired batch, <root>\.retired\<UTC stamp>.
struct Batch {
    std::string name;
    std::wstring folder;
    uint64_t time = 0;            // from the NAME: the clock of the retire run that made it
    bool manifest = false;        // <batch>\manifest.json is a plain file: a batch retire made
};
struct Listing {
    std::wstring root;            // the final path
    std::vector<Entry> entries;   // sorted by node, id, tag
    std::vector<Skipped> skipped;
    std::vector<Batch> batches;   // oldest first
};

// One run. The clock and the process table are INPUTS so that the selftest can stand a batch
// ten days back and plant a second engine without waiting ten days or starting one; the tool
// leaves them empty and gets the real ones.
struct Request {
    Mode mode = Mode::List;
    std::string root;             // "" = cache\trees
    double ageDays = 30.0;
    double purgeDays = 7.0;
    std::string confirm;
    uint64_t now = 0;             // FILETIME ticks; 0 = the system clock
    std::function<bool(std::vector<Process>&)> processes;   // empty = the real table
    std::function<void(const std::string&)> say;            // empty = Log("[prune] ...")
};
struct Result {
    Status status = Status::Listed;
    std::string why;              // the refusal or the failure, in words
    Listing listing;
    std::wstring batch;           // retire: the batch folder made, if one was
    std::vector<std::wstring> moved;     // retire: the tag folders moved, by their old paths
    std::vector<std::wstring> purged;    // purge: the batches deleted
    std::vector<Skipped> passed;         // retire/purge: planned items left alone, and why
    std::wstring failedAt;        // Stopped: the item
};

Result Run(const Request& q);

// ---- the pieces, public for the selftest -----------------------------------------------------
uint64_t Now();                                   // the system clock, FILETIME ticks (UTC)
std::string Utc(uint64_t ft);                     // "2026-09-28 21:14Z"
std::string BatchName(uint64_t ft);               // "20260928T211403Z"
bool ParseBatchName(const std::string& name, uint64_t& ft);
std::string Narrow(const std::wstring& w);        // UTF-8
std::wstring Widen(const std::string& s);         // UTF-8
// A folder's final path: opened following every junction on purpose (a worktree reaches the
// cache through one), read back with GetFinalPathNameByHandleW, the \\?\ prefix dropped.
bool FinalPath(const std::wstring& path, std::wstring& out, std::string* why);
// THE ROOT RULE, on a FINAL path: its last folder is named trees, and its parent is named
// cache, or it lies strictly inside `outFinal` (the worktree's out\, final too; "" = none).
bool RootAllowed(const std::wstring& finalRoot, const std::wstring& outFinal, std::string* why);
// `given` ("" = cache\trees) to its final path, then the rule, out\ measured from the cwd.
bool ResolveRoot(const std::string& given, std::wstring& finalRoot, std::string* why);
// <name>.<8 lowercase hex>, the name Sanitize makes, then the id TileTree hashes.
bool NodeName(const std::wstring& n, std::string& node, std::string& id);
bool TagName(const std::wstring& n);              // a lattice tag: cube16k, window_z14_...
bool TileName(const std::wstring& n);
// Another engine in the table: an image named gagame*.exe whose pid is not `self`.
bool OtherEngine(const std::vector<Process>& rows, uint32_t self, std::string* who);
bool Processes(std::vector<Process>& out);        // the real table (Toolhelp32)
bool SameVolume(const std::wstring& a, const std::wstring& b, std::string* why);
// The listing: read-only, never recurses, never follows a reparse point.
Listing List(const std::wstring& finalRoot, uint64_t now, double ageDays);

// THE ONE RECURSION. Count visits what a delete would take and changes nothing; Delete removes
// it, deepest first. A reparse point is recorded and never entered or removed -- the check is
// made again on every folder before the walk descends into it. Delete counts first and refuses
// outright if a link is anywhere under the folder, so it never stops halfway at one; it refuses
// a folder that is not strictly inside `fence`. `unsafeFollow` turns the reparse check off so the
// selftest can SEE what the check stops; it is refused with Delete, so only a count can ever
// cross a link. Depth is bounded: retire makes three levels.
enum class Walk : uint8_t { Count, Delete };
struct WalkStats {
    bool record = false;                  // Count: keep every path in `visited` (the selftest's)
    uint64_t files = 0, folders = 0, bytes = 0;
    std::vector<std::wstring> reparse;    // links met, not entered
    std::vector<std::wstring> visited;    // Count with `record`: every file a delete would take
    std::wstring failedAt;
    std::string why;
};
bool WalkTree(const std::wstring& folder, Walk mode, const std::wstring& fence, WalkStats& st,
              bool unsafeFollow = false);

}  // namespace ga::prune

namespace ga {
// The [prunetest] block (compose/TreePruneTest.cpp): a scratch root under out\prunetest, a canary
// behind a junction, every refusal above planted and caught.
bool RunPruneSelfTest();
}  // namespace ga
