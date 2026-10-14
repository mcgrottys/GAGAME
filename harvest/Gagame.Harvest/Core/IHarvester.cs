namespace Gagame.Harvest;

/// How a harvester takes a custom insert from the demo's dialog.
public enum InsertKind
{
    None,
    /// `--place NAME --box lon0,lat0,lon1,lat1` plus the extras named.
    PlaceBox,
    /// Files dropped into a cache folder the harvester sweeps on its next run.
    DropFolder,
    /// A file as the first argument, plus a box.
    FileArg,
    /// Flags to tick.
    Flags,
}

/// What a harvester says about its custom insert, for the demo's dialog.
public sealed record Insert(
    InsertKind Kind,
    string Hint,
    string[]? Choices = null,
    string? ChoiceFlag = null,
    string[]? Extras = null,
    string? Folder = null,
    string? Pattern = null,
    string[]? Flags = null,
    bool Final = false);

/// One source a harvester draws on: a label and where it lives.
public sealed record Source(string Label, string Url);

/// A harvester: one product of the engine's data, the stages Load -> Normalize -> write,
/// cache-first and polite. The factory makes one by `Name`; the demo reads the rest.
public interface IHarvester
{
    /// The name on the command line: `harvest <Name> [args]`.
    string Name { get; }
    /// A display title.
    string Title { get; }
    /// One or two sentences on what it fetches and writes.
    string What { get; }
    /// The files (from the repo root) whose presence says the product is there; empty when
    /// the output is named by the arguments.
    string[] Outputs { get; }
    /// Where the data comes from.
    Source[] Sources { get; }
    /// The custom insert it supports, if any.
    Insert? Insert { get; }
    /// A note for the demo's list (dependencies, what runs after).
    string? Note => null;
    /// The arguments a plain "Download" runs it with.
    string[] DefaultArgs => Array.Empty<string>();

    /// Runs the harvest. Returns the exit code (0 = wrote what it set out to write).
    int Run(HarvestContext ctx, string[] args);
}
