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

        var repo = new Repo(settings.RepoRoot);
        if (!settings.Installed)
        {
            using var install = new InstallForm(repo, settings);
            if (install.ShowDialog() != DialogResult.OK) return;
        }
        Application.Run(new MainForm(repo, settings));
    }
}
