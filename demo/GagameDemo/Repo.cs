using System.Diagnostics;
using System.Text.Json;
using Microsoft.Win32;

namespace GagameDemo;

/// A scene file the toolbar can launch: its path from the repo root and a title from its readme.
public sealed record SceneEntry(string RelativePath, string Title, string Group);

/// The checkout the demo drives: where the engine, scenes, harvesters and the cache junction are.
public sealed class Repo
{
    public const string KeyVariable = "GAGAME_GOOGLE_MAPS_KEY";

    public string Root { get; }
    public Repo(string root) => Root = Path.GetFullPath(root);

    public string EnginePath => Path.Combine(Root, "build", "bin", "gagame.exe");
    public bool EngineBuilt => File.Exists(EnginePath);
    public string CacheLink => Path.Combine(Root, "cache");
    public string DataDir => Path.Combine(Root, "data");

    public static bool IsRepoRoot(string? path) =>
        !string.IsNullOrEmpty(path) && File.Exists(Path.Combine(path, "build.bat")) &&
        Directory.Exists(Path.Combine(path, "scenes")) && Directory.Exists(Path.Combine(path, "harvester"));

    /// Walk up from `start` until a folder looks like the checkout (the demo's exe lives under it).
    public static string? FindRoot(string start)
    {
        var dir = new DirectoryInfo(start);
        while (dir is not null)
        {
            if (IsRepoRoot(dir.FullName)) return dir.FullName;
            dir = dir.Parent;
        }
        return null;
    }

    // ------------------------------------------------------------------ scenes

    /// Every scene file the engine boots as a place: the shipped bases and the demos, titled by
    /// their `_readme` first line. Recipes and rails are gate recordings, not places to visit.
    public List<SceneEntry> Scenes()
    {
        var list = new List<SceneEntry>();
        void Add(string sub, string group)
        {
            var dir = Path.Combine(Root, sub);
            if (!Directory.Exists(dir)) return;
            foreach (var f in Directory.GetFiles(dir, "*.json").OrderBy(f => f, StringComparer.OrdinalIgnoreCase))
                list.Add(new SceneEntry(Path.GetRelativePath(Root, f).Replace('\\', '/'), SceneTitle(f), group));
        }
        Add("scenes", "Base");
        Add(Path.Combine("scenes", "demos"), "Demos");
        return list;
    }

    static string SceneTitle(string file)
    {
        var name = Path.GetFileNameWithoutExtension(file);
        try
        {
            using var doc = JsonDocument.Parse(File.ReadAllText(file),
                new JsonDocumentOptions { CommentHandling = JsonCommentHandling.Skip, AllowTrailingCommas = true });
            if (doc.RootElement.TryGetProperty("_readme", out var r))
            {
                string? line = r.ValueKind == JsonValueKind.Array
                    ? r.EnumerateArray().Select(e => e.GetString()).FirstOrDefault(s => !string.IsNullOrWhiteSpace(s))
                    : r.GetString();
                if (!string.IsNullOrWhiteSpace(line))
                {
                    // "THE HAULOVER GATE -- a playable scene. You are ..." -> "THE HAULOVER GATE"
                    var cut = line.IndexOf(" -- ", StringComparison.Ordinal);
                    if (cut < 0) cut = line.IndexOf(" (", StringComparison.Ordinal);
                    if (cut < 0) cut = line.IndexOf(": ", StringComparison.Ordinal);
                    var title = (cut > 0 ? line[..cut] : line).Trim().TrimEnd('.', ':');
                    if (title.Length > 48) title = title[..48].TrimEnd() + "...";
                    return $"{name}  ({title})";
                }
            }
        }
        catch (Exception) { /* an unreadable scene is still launchable; the engine says why */ }
        return name;
    }

    // ------------------------------------------------------------------ the engine

    /// The start info for one scene, run from the repo root (shaders/ and cache\ are relative to
    /// it) with the key read fresh from the user environment, so a key saved after this process
    /// started still reaches the engine.
    public ProcessStartInfo EngineStart(string sceneRelative, DemoSettings settings, params string[] extra)
    {
        var psi = Hidden(EnginePath);
        psi.ArgumentList.Add(sceneRelative.Replace('/', '\\'));
        foreach (var (flag, assignment) in settings.EngineSetFlags())
        {
            psi.ArgumentList.Add(flag);
            psi.ArgumentList.Add(assignment);
        }
        foreach (var e in extra) psi.ArgumentList.Add(e);
        var key = ReadKey();
        if (!string.IsNullOrEmpty(key)) psi.Environment[KeyVariable] = key;
        return psi;
    }

    /// build.bat: finds Visual Studio by vswhere, configures CMake + Ninja, writes build\bin\gagame.exe.
    public ProcessStartInfo BuildStart()
    {
        var psi = Hidden("cmd.exe");
        psi.ArgumentList.Add("/c");
        psi.ArgumentList.Add(Path.Combine(Root, "build.bat"));
        return psi;
    }

    /// What a fresh machine still lacks, for the banner: the engine, the key, the datasets the
    /// shipped scenes read. Empty when nothing is missing.
    public List<string> Missing()
    {
        var m = new List<string>();
        if (!EngineBuilt) m.Add("engine not built");
        var data = Datasets.All.Where(d => d.Script is not null && d.Outputs.Length > 0 && !d.Present(this)).ToList();
        if (data.Count > 0) m.Add(data.Count == 1 ? $"{data[0].Name} missing" : $"{data.Count} datasets missing");
        if (string.IsNullOrEmpty(ReadKey())) m.Add("no Google key (the Earth keeps its procedural look)");
        return m;
    }

    /// The tree-prune LIST (changes nothing): what the cache holds and what is stale at this age.
    public ProcessStartInfo PruneListStart(double ageDays)
    {
        var psi = Hidden(EnginePath);
        psi.ArgumentList.Add("--tree-prune");
        psi.ArgumentList.Add("--set");
        psi.ArgumentList.Add($"prune.ageDays={ageDays}");
        return psi;
    }

    /// The harvest console: beside this exe when published together, else the checkout's build of
    /// harvest\harvest (Debug or Release, whichever is newer), else `dotnet run` on its project.
    public (string exe, List<string> lead) HarvestExe()
    {
        var beside = Path.Combine(AppContext.BaseDirectory, "harvest.exe");
        if (File.Exists(beside)) return (beside, new());
        var builds = new[] { "Debug", "Release" }
            .Select(c => Path.Combine(Root, "harvest", "harvest", "bin", c, "net10.0", "harvest.exe"))
            .Where(File.Exists).OrderByDescending(File.GetLastWriteTimeUtc).ToList();
        if (builds.Count > 0) return (builds[0], new());
        return ("dotnet", new() { "run", "--project", Path.Combine(Root, "harvest", "harvest"), "--no-launch-profile", "--" });
    }

    /// A harvester by name through the harvest console, from the root: `harvest <name> args`.
    public ProcessStartInfo HarvestStart(string name, IEnumerable<string> args, string? dataCache = null)
    {
        var (exe, lead) = HarvestExe();
        var psi = Hidden(exe);
        foreach (var l in lead) psi.ArgumentList.Add(l);
        psi.ArgumentList.Add(name);
        foreach (var a in args) psi.ArgumentList.Add(a);
        psi.ArgumentList.Add("--root");
        psi.ArgumentList.Add(Root);
        if (!string.IsNullOrWhiteSpace(dataCache)) { psi.ArgumentList.Add("--datacache"); psi.ArgumentList.Add(dataCache); }
        return psi;
    }

    ProcessStartInfo Hidden(string exe) => new(exe)
    {
        WorkingDirectory = Root,
        UseShellExecute = false,
        RedirectStandardOutput = true,
        RedirectStandardError = true,
        CreateNoWindow = true,
    };

    // ------------------------------------------------------------------ the key

    /// The key the engine will see: this process's environment, else HKCU\Environment (where
    /// `setx` writes; the engine reads the same two places, src/core/TileProviders.cpp).
    public static string ReadKey()
    {
        var v = Environment.GetEnvironmentVariable(KeyVariable);
        if (!string.IsNullOrEmpty(v)) return v;
        try
        {
            using var k = Registry.CurrentUser.OpenSubKey("Environment");
            return k?.GetValue(KeyVariable) as string ?? "";
        }
        catch (Exception) { return ""; }
    }

    /// Writes the user environment variable (what `setx` does); never a file in the repo.
    public static void WriteKey(string key) =>
        Environment.SetEnvironmentVariable(KeyVariable, string.IsNullOrWhiteSpace(key) ? null : key.Trim(),
            EnvironmentVariableTarget.User);

    /// Says a key is set without printing it: length and the first four characters only.
    public static string DescribeKey(string key) =>
        string.IsNullOrEmpty(key) ? "not set" : $"set ({key.Length} characters, begins {key[..Math.Min(4, key.Length)]}...)";

    // ------------------------------------------------------------------ the cache link

    public enum CacheState { Missing, Junction, Folder }

    /// What stands at <repo>\cache today: nothing, a junction (and where it points), or a folder.
    public (CacheState state, string target) CacheStatus()
    {
        var di = new DirectoryInfo(CacheLink);
        if (!di.Exists) return (CacheState.Missing, "");
        if (di.LinkTarget is string t) return (CacheState.Junction, t);
        return (CacheState.Folder, di.FullName);
    }

    /// Points <repo>\cache at `target` by a directory junction (`mklink /J`, no admin). Refuses to
    /// touch a real folder with content in it: moving data is the owner's job, by hand.
    public string LinkCache(string target)
    {
        target = Path.GetFullPath(target);
        var (state, current) = CacheStatus();
        if (state == CacheState.Junction)
        {
            if (string.Equals(Path.GetFullPath(current), target, StringComparison.OrdinalIgnoreCase))
                return $"cache already points at {target}";
            Directory.Delete(CacheLink);   // removes the junction, never what it points at
        }
        else if (state == CacheState.Folder)
        {
            if (string.Equals(Path.GetFullPath(current), target, StringComparison.OrdinalIgnoreCase))
                return $"cache is a plain folder at {target}";
            if (Directory.EnumerateFileSystemEntries(CacheLink).Any())
                return $"REFUSED: {CacheLink} is a real folder with files in it. Move its contents to {target} " +
                       "by hand, then press Link again.";
            Directory.Delete(CacheLink);
        }
        Directory.CreateDirectory(target);
        using var p = Process.Start(new ProcessStartInfo("cmd.exe")
        {
            ArgumentList = { "/c", "mklink", "/J", CacheLink, target },
            UseShellExecute = false, CreateNoWindow = true,
            RedirectStandardOutput = true, RedirectStandardError = true,
        })!;
        var outText = p.StandardOutput.ReadToEnd() + p.StandardError.ReadToEnd();
        p.WaitForExit();
        return p.ExitCode == 0 ? $"cache -> {target}" : $"mklink failed: {outText.Trim()}";
    }

    /// Bytes under a folder, for the install form's cache gauge (junctions inside are not followed).
    public static long FolderBytes(string path, CancellationToken ct)
    {
        long total = 0;
        var stack = new Stack<string>();
        stack.Push(path);
        while (stack.Count > 0)
        {
            ct.ThrowIfCancellationRequested();
            var dir = stack.Pop();
            IEnumerable<string> files, dirs;
            try { files = Directory.EnumerateFiles(dir); dirs = Directory.EnumerateDirectories(dir); }
            catch (Exception) { continue; }
            foreach (var f in files) { try { total += new FileInfo(f).Length; } catch (Exception) { } }
            foreach (var d in dirs)
            {
                try { if (new DirectoryInfo(d).LinkTarget is null) stack.Push(d); } catch (Exception) { }
            }
        }
        return total;
    }

    public static string Gb(long bytes) => $"{bytes / 1e9:0.0} GB";
}
