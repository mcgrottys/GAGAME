using System.Diagnostics;

namespace GagameDemo;

/// The scene bar: a toolbar up top that swaps the running engine between scenes, the engine's
/// log beneath it. One engine at a time; picking another scene stops the first and boots the next.
public sealed class MainForm : Form
{
    readonly Repo _repo;
    readonly DemoSettings _settings;
    readonly ProcessRunner _runner;
    readonly ToolStripDropDownButton _scenes = new("Scene") { DisplayStyle = ToolStripItemDisplayStyle.Text };
    readonly ToolStripButton _relaunch = new("Relaunch") { Enabled = false };
    readonly ToolStripButton _stop = new("Stop") { Enabled = false };
    readonly ToolStripLabel _current = new("no scene running");
    readonly ToolStripStatusLabel _status = new("idle");
    readonly TextBox _log = new() { Multiline = true, ReadOnly = true, ScrollBars = ScrollBars.Both, Dock = DockStyle.Fill, Font = new Font("Consolas", 9f), WordWrap = false };
    string? _scene;

    public MainForm(Repo repo, DemoSettings settings)
    {
        _repo = repo;
        _settings = settings;
        _runner = new ProcessRunner(SynchronizationContext.Current!, Log, Running);

        Text = "GAGAME demo";
        StartPosition = FormStartPosition.CenterScreen;
        Size = new Size(1000, 640);
        MinimumSize = new Size(700, 400);

        var bar = new ToolStrip { GripStyle = ToolStripGripStyle.Hidden, RenderMode = ToolStripRenderMode.System };
        FillScenes();
        _relaunch.Click += (_, _) => { if (_scene is not null) Launch(_scene); };
        _stop.Click += (_, _) => _runner.Stop();
        var setup = new ToolStripButton("Setup...");
        setup.Click += (_, _) => Setup();
        var openScene = new ToolStripButton("Open scene file");
        openScene.Click += (_, _) => { if (_scene is not null) Open(Path.Combine(_repo.Root, _scene)); };
        var help = new ToolStripButton("Controls");
        help.Click += (_, _) => Open(Path.Combine(_repo.Root, "docs", "LAUNCH.md"));
        var clear = new ToolStripButton("Clear log");
        clear.Click += (_, _) => _log.Clear();
        bar.Items.AddRange([_scenes, _relaunch, _stop, new ToolStripSeparator(), _current,
            new ToolStripSeparator(), setup, openScene, help, clear]);

        var status = new StatusStrip();
        status.Items.Add(_status);

        Controls.Add(_log);
        Controls.Add(bar);
        Controls.Add(status);

        Load += (_, _) =>
        {
            Log($"[demo] checkout {_repo.Root}");
            Log(_repo.EngineBuilt ? "[demo] engine built; pick a scene from the toolbar" : "[demo] build\\bin\\gagame.exe is missing: run build.bat first");
            if (_repo.EngineBuilt && !string.IsNullOrEmpty(_settings.LastScene) && File.Exists(Path.Combine(_repo.Root, _settings.LastScene)))
                Launch(_settings.LastScene);
        };
        FormClosing += (_, _) => _runner.Dispose();
    }

    void FillScenes()
    {
        _scenes.DropDownItems.Clear();
        string? group = null;
        foreach (var s in _repo.Scenes())
        {
            if (s.Group != group)
            {
                if (group is not null) _scenes.DropDownItems.Add(new ToolStripSeparator());
                _scenes.DropDownItems.Add(new ToolStripLabel(s.Group) { Font = new Font(Font, FontStyle.Bold), Enabled = false });
                group = s.Group;
            }
            var item = new ToolStripMenuItem(s.Title) { Tag = s.RelativePath, ToolTipText = s.RelativePath };
            item.Click += (_, _) => Launch(s.RelativePath);
            _scenes.DropDownItems.Add(item);
        }
        _scenes.DropDownItems.Add(new ToolStripSeparator());
        var rescan = new ToolStripMenuItem("Rescan scenes\\");
        rescan.Click += (_, _) => FillScenes();
        _scenes.DropDownItems.Add(rescan);
    }

    /// Stops whatever runs and boots `scene`. The engine takes a moment to release the GPU and the
    /// cache's files, so the next boot waits for the exit rather than racing it.
    void Launch(string scene)
    {
        if (!_repo.EngineBuilt) { Log("[demo] the engine is not built"); return; }
        _scene = scene;
        _settings.LastScene = scene;
        _settings.Save();
        foreach (ToolStripItem i in _scenes.DropDownItems)
            if (i is ToolStripMenuItem m) m.Checked = Equals(m.Tag, scene);
        if (_runner.IsRunning)
        {
            Log("[demo] stopping the running scene ...");
            _pending = scene;
            _runner.Stop();
            return;
        }
        _runner.Start(_repo.EngineStart(scene, _settings), scene);
    }

    string? _pending;

    void Running(bool on)
    {
        _stop.Enabled = on;
        _relaunch.Enabled = _scene is not null;
        _current.Text = on ? $"running {_scene}" : "no scene running";
        _status.Text = on ? $"gagame.exe pid {_runner.Pid}" : "idle";
        if (!on && _pending is string next)
        {
            _pending = null;
            _runner.Start(_repo.EngineStart(next, _settings), next);
        }
    }

    void Setup()
    {
        using var f = new InstallForm(_repo, _settings);
        f.ShowDialog(this);
        Log("[demo] settings " + (f.DialogResult == DialogResult.OK ? "saved; they apply at the next launch" : "unchanged"));
    }

    void Log(string line)
    {
        if (_log.TextLength > 2_000_000) _log.Clear();
        _log.AppendText(line + Environment.NewLine);
    }

    static void Open(string target)
    {
        try { Process.Start(new ProcessStartInfo(target) { UseShellExecute = true }); }
        catch (Exception) { }
    }
}
