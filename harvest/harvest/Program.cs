using Gagame.Harvest;

/// harvest <name> [args]       run one harvester from the checkout (found by walking up from the exe,
///                             or named by --root DIR); the big-data drive by --datacache DIR or GAGAME_DATACACHE
/// harvest --list [--json]     what the factory can make: name, title, outputs, sources, inserts
static class Program
{
    static int Main(string[] argv)
    {
        var args = argv.ToList();
        string? root = Take(args, "--root");
        string? dataCache = Take(args, "--datacache");
        if (args.Count == 0 || args[0] is "--help" or "-h")
        {
            Console.WriteLine("harvest <name> [args] | harvest --list [--json]   (--root DIR, --datacache DIR)");
            Console.WriteLine("names: " + string.Join(", ", HarvesterFactory.Names));
            return args.Count == 0 ? 1 : 0;
        }
        if (args[0] == "--list")
        {
            var json = args.Contains("--json");
            if (json)
            {
                var list = HarvesterFactory.All().Select(h => new Dictionary<string, object?>
                {
                    ["name"] = h.Name, ["title"] = h.Title, ["what"] = h.What, ["outputs"] = h.Outputs.ToList(),
                    ["sources"] = h.Sources.Select(s => new Dictionary<string, object?> { ["label"] = s.Label, ["url"] = s.Url }).ToList(),
                    ["defaultArgs"] = h.DefaultArgs.ToList(), ["note"] = h.Note,
                    ["insert"] = h.Insert is null ? null : new Dictionary<string, object?>
                    {
                        ["kind"] = h.Insert.Kind.ToString(), ["hint"] = h.Insert.Hint, ["choices"] = h.Insert.Choices?.ToList(), ["choiceFlag"] = h.Insert.ChoiceFlag,
                        ["extras"] = h.Insert.Extras?.ToList(), ["folder"] = h.Insert.Folder, ["pattern"] = h.Insert.Pattern, ["flags"] = h.Insert.Flags?.ToList(), ["final"] = h.Insert.Final,
                    },
                }).ToList();
                Console.WriteLine(JsonUtil.Dump(list));
            }
            else
                foreach (var x in HarvesterFactory.All())
                    Console.WriteLine($"{x.Name,-18} {x.Title,-34} {string.Join(", ", x.Outputs)}");
            return 0;
        }
        var name = args[0];
        if (!HarvesterFactory.Has(name))
        {
            Console.Error.WriteLine($"no harvester named '{name}'; have: {string.Join(", ", HarvesterFactory.Names)}");
            return 2;
        }
        root ??= FindRoot(AppContext.BaseDirectory) ?? FindRoot(Environment.CurrentDirectory);
        if (root is null)
        {
            Console.Error.WriteLine("not inside a GAGAME checkout (no build.bat + scenes\\ + harvester\\ above here); name it with --root DIR");
            return 2;
        }
        var ctx = new HarvestContext(root, dataCache);
        var h = HarvesterFactory.Create(name);
        try
        {
            return h.Run(ctx, args.Skip(1).ToArray());
        }
        catch (HarvestStop e)
        {
            ctx.Log(e.Message);
            return e.Code;
        }
        catch (Polite.StopException e)
        {
            ctx.Log("STOPPED: " + e.Message);
            return 3;
        }
        catch (Exception e)
        {
            ctx.Log($"{e.GetType().Name}: {e.Message}");
            ctx.Log(e.StackTrace ?? "");
            return 1;
        }
    }

    static string? Take(List<string> args, string flag)
    {
        var i = args.IndexOf(flag);
        if (i < 0 || i + 1 >= args.Count) return null;
        var v = args[i + 1];
        args.RemoveRange(i, 2);
        return v;
    }

    static string? FindRoot(string start)
    {
        var dir = new DirectoryInfo(start);
        while (dir is not null)
        {
            if (File.Exists(Path.Combine(dir.FullName, "build.bat")) && Directory.Exists(Path.Combine(dir.FullName, "scenes")) && Directory.Exists(Path.Combine(dir.FullName, "harvester")))
                return dir.FullName;
            dir = dir.Parent;
        }
        return null;
    }
}
