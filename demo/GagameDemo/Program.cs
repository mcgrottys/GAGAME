namespace GagameDemo;

/// The demo launcher: an install form the first time (cache, key, data), then the scene bar.
/// It runs the engine the way docs/LAUNCH.md says to -- `build\bin\gagame.exe <scene> --set ...`
/// from the repo root -- and carries no data or shaders of its own.
static class Program
{
    [STAThread]
    static void Main(string[] args)
    {
        ApplicationConfiguration.Initialize();

        // The checkout: the one named on the command line, else the one this exe sits inside, else
        // the one remembered, else the one the demo was started from, else ask.
        // `--fidelity [FILE]` opens the fidelity painter alone: no scene bar, no engine started.
        var fidelity = Array.IndexOf(args, "--fidelity");
        string? fidelityFile = null;
        if (fidelity >= 0)
        {
            if (fidelity + 1 < args.Length && !args[fidelity + 1].StartsWith("--", StringComparison.Ordinal)) fidelityFile = Path.GetFullPath(args[fidelity + 1]);
            args = args.Where((_, i) => i != fidelity && !(fidelityFile is not null && i == fidelity + 1)).ToArray();
        }
        var settings = DemoSettings.Load();
        var own = args.Length > 0 && Directory.Exists(args[0]) ? Path.GetFullPath(args[0]) : Repo.FindRoot(AppContext.BaseDirectory);
        if (Repo.IsRepoRoot(own)) settings.RepoRoot = own!;
        if (!Repo.IsRepoRoot(settings.RepoRoot))
        {
            var found = Repo.FindRoot(Environment.CurrentDirectory);
            if (found is null)
            {
                using var pick = new FolderBrowserDialog
                {
                    Description = "Pick the GAGAME checkout (the folder holding build.bat and scenes\\)",
                    UseDescriptionForTitle = true,
                };
                if (pick.ShowDialog() != DialogResult.OK || !Repo.IsRepoRoot(pick.SelectedPath))
                {
                    MessageBox.Show("GagameDemo needs a GAGAME checkout to run from.", "GAGAME demo",
                        MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    return;
                }
                found = pick.SelectedPath;
            }
            settings.RepoRoot = found;
            settings.Save();
        }

        if (fidelity >= 0)
        {
            Application.Run(new FidelityForm(new Repo(settings.RepoRoot), settings, fidelityFile));
            return;
        }
        // The scene bar is the first thing seen; setup is a dropdown there and a banner for what is missing.
        Application.Run(new MainForm(new Repo(settings.RepoRoot), settings));
    }
}
