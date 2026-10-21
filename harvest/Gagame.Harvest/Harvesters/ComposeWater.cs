namespace Gagame.Harvest.Harvesters;

/// Every water layer's composite over one fidelity map, in order: the bed, the rivers, the tides
/// (which read the rivers' reaches for each station's river km), the currents, the sea state. Each writes its own `<layer>.composite.json` under the one
/// output folder; a layer that stops does not stop the others, and the exit code is the worst.
public sealed class ComposeWater : IHarvester
{
    public string Name => "compose-water";
    public string Title => "Water composites (all layers)";
    public string What => "compose-bed, compose-river, compose-tides, compose-currents and compose-seastate over one fidelity map.";
    // THE WORLD (2026-10-21): the shipped scenes stand on scenes/world.json, which reads this map's
    // composite -- its fidelity map is tracked in the repo, its outputs are data.
    public string[] DefaultArgs => ["--fidelity", "harvest/fidelity/world.gafid", "--out", "data/composite/world/"];
    public string[] Outputs => ["data/composite/world/water.bed.composite.json", "data/composite/world/river/river.json",
                                "data/composite/world/tides/stations.json", "data/composite/world/currents/currents.json",
                                "data/composite/world/sea/seastate.json"];
    public Source[] Sources => [];
    public Insert? Insert => new(InsertKind.FileArg, "A fidelity map (.gafid) and an output folder: --fidelity FILE --out data/composite/<name>/");

    static readonly string[] Layers = ["compose-bed", "compose-river", "compose-tides", "compose-currents", "compose-seastate"];

    public int Run(HarvestContext ctx, string[] argv)
    {
        CompositeRun.Paths(new Args(argv), Name);
        var worst = 0;
        foreach (var name in Layers)
        {
            ctx.Log($"==== {name}");
            int code;
            try { code = HarvesterFactory.Create(name).Run(ctx, argv); }
            catch (Polite.StopException e) { ctx.Log($"STOPPED: {e.Message}"); code = 3; }
            catch (HarvestStop e) { ctx.Log(e.Message); code = e.Code; }
            catch (Exception e) { ctx.Log($"{name} failed: {e.GetType().Name}: {e.Message}"); code = 1; }
            worst = Math.Max(worst, code);
        }
        return worst;
    }
}
