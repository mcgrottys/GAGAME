namespace Gagame.Harvest.Harvesters;

/// Every water layer's composite over one fidelity map, in order: the bed, the tides, the
/// currents, the sea state, the rivers. Each writes its own `<layer>.composite.json` under the one
/// output folder; a layer that stops does not stop the others, and the exit code is the worst.
public sealed class ComposeWater : IHarvester
{
    public string Name => "compose-water";
    public string Title => "Water composites (all layers)";
    public string What => "compose-bed, compose-tides, compose-currents, compose-seastate and compose-river over one fidelity map.";
    public string[] Outputs => [];
    public Source[] Sources => [];
    public Insert? Insert => new(InsertKind.FileArg, "A fidelity map (.gafid) and an output folder: --fidelity FILE --out data/composite/<name>/");

    static readonly string[] Layers = ["compose-bed", "compose-tides", "compose-currents", "compose-seastate", "compose-river"];

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
