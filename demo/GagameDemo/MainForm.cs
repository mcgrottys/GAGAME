using System.Diagnostics;

namespace GagameDemo;

/// The first thing seen: a toolbar up top that swaps the running engine between scenes, with
/// Setup as a dropdown beside it and a banner for whatever a fresh machine still lacks (the
/// engine, data, the key). The engine's log is the window. One engine at a time; picking another
/// scene stops the first and boots the next.
public sealed class MainForm : Form
{
    Repo _repo;
    readonly DemoSettings _settings;
    readonly ProcessRunner _runner;
    readonly ToolStripDropDownButton _scenes = new("Scene") { DisplayStyle = ToolStripItemDisplayStyle.Text };
    readonly ToolStripDropDownButton _setup = new("Setup") { DisplayStyle = ToolStripItemDisplayStyle.Text };
    readonly ToolStripButton _relaunch = new("Relaunch") { Enabled = false };
    readonly ToolStripButton _stop = new("Stop") { Enabled = false };
    readonly ToolStripLabel _current = new("no scene running");
    readonly ToolStripStatusLabel _status = new("idle");
    readonly Panel _banner = new() { Dock = DockStyle.Top, BackColor = Color.FromArgb(255, 249, 196), Padding = new Padding(10, 6, 10, 6), AutoSize = true, Visible = false };
    readonly Label _bannerText = new() { AutoSize = true, Margin = new Padding(0, 6, 12, 0) };
    readonly Button _bannerBuild = new() { Text = "Build engine", AutoSize = true };
    readonly Button _bannerSetup = new() { Text = "Setup...", AutoSize = true };
    readonly TextBox _log = new() { Multiline = true, ReadOnly = true, ScrollBars = ScrollBars.Both, Dock = DockStyle.Fill, Font = new Font("Consolas", 9f), WordWrap = false };
    string? _scene;
    string? _pending;

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
        FillSetup();
        _relaunch.Click += (_, _) => { if (_scene is not null) Launch(_scene); };
        _stop.Click += (_, _) => _runner.Stop();
        var openScene = new ToolStripButton("Open scene file");
        openScene.Click += (_, _) => { if (_scene is not null) Open(Path.Combine(_repo.Root, _scene)); };
        var help = new ToolStripButton("Controls");
        help.Click += (_, _) => Open(Path.Combine(_repo.Root, "docs", "LAUNCH.md"));
        var clear = new ToolStripButton("Clear log");
        clear.Click += (_, _) => _log.Clear();
        bar.Items.AddRange([_scenes, _relaunch, _stop, new ToolStripSeparator(), _current,
            new ToolStripSeparator(), _setup, openScene, help, clear]);
        // the debug views on a row of their own, so the scene bar never overflows into a chevron
        var debugBar = new ToolStrip { GripStyle = ToolStripGripStyle.Hidden, RenderMode = ToolStripRenderMode.System };
        debugBar.Items.AddRange(DebugGroup());

        // The nag banner: what is missing, and the two things that mend most of it.
        var flow = new FlowLayoutPanel { AutoSize = true, WrapContents = true, Dock = DockStyle.Fill };
        var dismiss = new Button { Text = "Dismiss", AutoSize = true };
        _bannerBuild.Click += (_, _) => Build();
        _bannerSetup.Click += (_, _) => Setup(null);
        dismiss.Click += (_, _) => { _settings.DismissedBanner = string.Join("; ", _repo.Missing()); _settings.Save(); RefreshBanner(); };
        flow.Controls.AddRange([_bannerText, _bannerBuild, _bannerSetup, dismiss]);
        _banner.Controls.Add(flow);

        var status = new StatusStrip();
        status.Items.Add(_status);

        Controls.Add(_log);
        Controls.Add(_banner);
        Controls.Add(debugBar);
        Controls.Add(bar);
        Controls.Add(status);

        Load += (_, _) =>
        {
            Log($"[demo] checkout {_repo.Root}");
            RefreshBanner();
            if (_repo.EngineBuilt && !string.IsNullOrEmpty(_settings.LastScene) && File.Exists(Path.Combine(_repo.Root, _settings.LastScene)))
                Launch(_settings.LastScene);
            else if (_repo.EngineBuilt) Log("[demo] pick a scene from the toolbar");
        };
        Activated += (_, _) => RefreshBanner();   // a key set or a harvest run elsewhere shows on return
        FormClosing += (_, _) => _runner.Dispose();
    }

    // ------------------------------------------------------------------ the toolbar

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
            var item = new ToolStripMenuItem(s.Title) { Tag = s.RelativePath, ToolTipText = s.RelativePath, Checked = s.RelativePath == _scene };
            item.Click += (_, _) => Launch(s.RelativePath);
            _scenes.DropDownItems.Add(item);
        }
        _scenes.DropDownItems.Add(new ToolStripSeparator());
        var rescan = new ToolStripMenuItem("Rescan scenes\\");
        rescan.Click += (_, _) => FillScenes();
        _scenes.DropDownItems.Add(rescan);
    }

    void FillSetup()
    {
        _setup.DropDownItems.Clear();
        void Item(string text, Action act) { var i = new ToolStripMenuItem(text); i.Click += (_, _) => act(); _setup.DropDownItems.Add(i); }
        Item("Cache and limits...", () => Setup(InstallForm.SectionCache));
        Item("Google key...", () => Setup(InstallForm.SectionKey));
        Item("Data files...", () => Setup(InstallForm.SectionData));
        Item("Fidelity map...", () => new FidelityForm(_repo, _settings).Show(this));
        _setup.DropDownItems.Add(new ToolStripSeparator());
        Item("Build engine (build.bat)", Build);
        Item("Choose checkout...", ChooseCheckout);
        Item("Open checkout folder", () => Open(_repo.Root));
    }

    /// The engine's debug views (scene section `debug`; G in the window steps through the views):
    /// each control is kept in the settings and passed as `--set debug.*` at the next launch, and
    /// only where it differs from the engine's default, so a debug scene's own section stands.
    ToolStripItem[] DebugGroup()
    {
        string[] views = ["", "none", "flat", "edges", "normals", "height", "depth", "meshlets", "albedo", "residency", "residency.height", "residency.landsea"];
        var view = new ToolStripComboBox { DropDownStyle = ComboBoxStyle.DropDownList, AutoSize = false, Width = 120, ToolTipText = "debug.view (blank = the scene's)" };
        view.Items.AddRange(views.Select(v => (object)(v.Length == 0 ? "(scene)" : v)).ToArray());
        view.SelectedIndex = Math.Max(0, Array.IndexOf(views, _settings.DebugView));
        view.SelectedIndexChanged += (_, _) => { _settings.DebugView = views[view.SelectedIndex]; DebugChanged(); };

        var wire = new NumericUpDown { Minimum = 0, Maximum = 16, DecimalPlaces = 1, Increment = 0.5m, Width = 52, Value = (decimal)Math.Clamp(_settings.DebugWirePx, 0, 16) };
        wire.ValueChanged += (_, _) => { _settings.DebugWirePx = (double)wire.Value; DebugChanged(); };

        var colour = new ToolStripButton("■") { ToolTipText = "debug.wireColor (right click: the engine's)", ForeColor = WireColour() ?? SystemColors.GrayText };
        colour.MouseUp += (_, e) =>
        {
            if (e.Button == MouseButtons.Right) _settings.DebugWireColor = null;
            else
            {
                using var d = new ColorDialog { Color = WireColour() ?? Color.FromArgb(255, 217, 51), FullOpen = true };
                if (d.ShowDialog(this) != DialogResult.OK) return;
                _settings.DebugWireColor = [Math.Round(d.Color.R / 255.0, 3), Math.Round(d.Color.G / 255.0, 3), Math.Round(d.Color.B / 255.0, 3), 1];
            }
            colour.ForeColor = WireColour() ?? SystemColors.GrayText;
            DebugChanged();
        };

        string[] bys = ["", "colour", "edge", "depth"];
        var by = new ToolStripComboBox { DropDownStyle = ComboBoxStyle.DropDownList, AutoSize = false, Width = 86, ToolTipText = "debug.wireBy (blank = the scene's)" };
        by.Items.AddRange(bys.Select(v => (object)(v.Length == 0 ? "(scene)" : v)).ToArray());
        by.SelectedIndex = Math.Max(0, Array.IndexOf(bys, _settings.DebugWireBy));
        by.SelectedIndexChanged += (_, _) => { _settings.DebugWireBy = bys[by.SelectedIndex]; DebugChanged(); };

        var contour = new NumericUpDown { Minimum = 0, Maximum = 10000, DecimalPlaces = 1, Increment = 1, Width = 64, Value = (decimal)Math.Clamp(_settings.DebugContourM, 0, 10000) };
        contour.ValueChanged += (_, _) => { _settings.DebugContourM = (double)contour.Value; DebugChanged(); };

        return [new ToolStripLabel("Debug"), view, new ToolStripLabel("wire px"), new ToolStripControlHost(wire) { ToolTipText = "debug.wirePx (0 = no wire)" },
            colour, by, new ToolStripLabel("contour m"), new ToolStripControlHost(contour) { ToolTipText = "debug.contourM (0 = none)" }];
    }

    Color? WireColour() => _settings.DebugWireColor is { Length: 4 } c
        ? Color.FromArgb((int)(Math.Clamp(c[0], 0, 1) * 255), (int)(Math.Clamp(c[1], 0, 1) * 255), (int)(Math.Clamp(c[2], 0, 1) * 255))
        : null;

    void DebugChanged()
    {
        _settings.Save();
        _status.Text = "debug views apply at the next launch (Relaunch)";
    }

    // ------------------------------------------------------------------ the banner

    void RefreshBanner()
    {
        var missing = _repo.Missing();
        var key = string.Join("; ", missing);
        var show = missing.Count > 0 && key != _settings.DismissedBanner;
        _bannerText.Text = show ? "Missing: " + string.Join("  |  ", missing) : "";
        _bannerBuild.Visible = !_repo.EngineBuilt;
        _banner.Visible = show;
    }

    // ------------------------------------------------------------------ actions

    /// Stops whatever runs and boots `scene`. The engine takes a moment to release the GPU and the
    /// cache's files, so the next boot waits for the exit rather than racing it.
    void Launch(string scene)
    {
        if (!_repo.EngineBuilt) { Log("[demo] the engine is not built: Setup > Build engine"); RefreshBanner(); return; }
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

    void Build()
    {
        if (_runner.IsRunning) { Log("[demo] stop the running scene first"); return; }
        if (!File.Exists(Path.Combine(_repo.Root, "build.bat"))) { Log("[demo] no build.bat in the checkout"); return; }
        _current.Text = "building the engine";
        _runner.Start(_repo.BuildStart(), "build.bat");
    }

    void ChooseCheckout()
    {
        using var pick = new FolderBrowserDialog
        {
            Description = "The GAGAME checkout to run (the folder holding build.bat and scenes\\)",
            UseDescriptionForTitle = true, SelectedPath = _repo.Root,
        };
        if (pick.ShowDialog(this) != DialogResult.OK) return;
        if (!Repo.IsRepoRoot(pick.SelectedPath)) { Log("[demo] not a GAGAME checkout: " + pick.SelectedPath); return; }
        if (_runner.IsRunning) _runner.Stop();
        _repo = new Repo(pick.SelectedPath);
        _settings.RepoRoot = _repo.Root;
        _settings.Save();
        _scene = null;
        Log($"[demo] checkout {_repo.Root}");
        FillScenes();
        RefreshBanner();
    }

    void Setup(string? section)
    {
        using var f = new InstallForm(_repo, _settings, section);
        f.ShowDialog(this);
        Log("[demo] settings " + (f.DialogResult == DialogResult.OK ? "saved; they apply at the next launch" : "unchanged"));
        RefreshBanner();
    }

    void Running(bool on)
    {
        _stop.Enabled = on;
        _relaunch.Enabled = _scene is not null && !on;
        _setup.Enabled = true;
        _current.Text = on ? (_scene is null ? "building the engine" : $"running {_scene}") : "no scene running";
        _status.Text = on ? $"pid {_runner.Pid}" : "idle";
        if (!on)
        {
            RefreshBanner();
            if (_pending is string next)
            {
                _pending = null;
                _runner.Start(_repo.EngineStart(next, _settings), next);
            }
        }
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
