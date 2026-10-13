using System.Diagnostics;
using Gagame.Harvest;

namespace GagameDemo;

/// The install form: where the cache stands and its limits, the key, and the data the engine
/// reads -- each line with its harvester, its source and the custom insert it takes.
public sealed class InstallForm : Form
{
    readonly Repo _repo;
    readonly DemoSettings _settings;
    readonly ProcessRunner _runner;
    CancellationTokenSource? _gauge;

    // cache
    readonly TextBox _cachePath = new() { Width = 420 };
    readonly TextBox _dataCache = new() { Width = 420, PlaceholderText = @"D:\DataCache\GAGAME (the default when empty)" };
    readonly Queue<string> _queue = new();
    readonly Label _cacheStatus = new() { AutoSize = true, MaximumSize = new Size(640, 0) };
    readonly NumericUpDown _tileBudget = Num(0, 1_000_000);
    readonly NumericUpDown _dayTiles = Num(0, 10_000_000);
    readonly NumericUpDown _dayGb = Num(0, 1000, 2);
    readonly NumericUpDown _zoom = Num(0, 19);
    readonly NumericUpDown _ageDays = Num(1, 3650);

    // key
    readonly Label _keyStatus = new() { AutoSize = true };
    readonly TextBox _keyBox = new() { Width = 420, UseSystemPasswordChar = true, PlaceholderText = "paste a Google Map Tiles API key" };

    // data
    readonly ListView _list = new() { View = View.Details, FullRowSelect = true, HideSelection = false, Dock = DockStyle.Fill, MultiSelect = false };
    readonly Button _download = new() { Text = "Download", AutoSize = true };
    readonly Button _insert = new() { Text = "Insert...", AutoSize = true };
    readonly Button _source = new() { Text = "Source...", AutoSize = true };
    readonly Button _stop = new() { Text = "Stop", AutoSize = true, Enabled = false };
    readonly Label _what = new() { AutoSize = true, MaximumSize = new Size(640, 0), Height = 48 };

    readonly TextBox _log = new() { Multiline = true, ReadOnly = true, ScrollBars = ScrollBars.Both, Dock = DockStyle.Fill, Font = new Font("Consolas", 9f), WordWrap = false };
    readonly Button _finish = new() { Text = "Save and close", AutoSize = true, DialogResult = DialogResult.OK };
    readonly Dictionary<string, GroupBox> _groups = new();
    readonly Button _build = new() { Text = "Build engine", AutoSize = true };
    readonly Label _engine = new() { AutoSize = true, Margin = new Padding(3, 7, 3, 3) };

    public const string SectionCache = "cache", SectionKey = "key", SectionData = "data", SectionCheckout = "checkout";

    /// `section` names the group to scroll into view when the form opens (the Setup dropdown's items).
    public InstallForm(Repo repo, DemoSettings settings, string? section = null)
    {
        _repo = repo;
        _settings = settings;
        _runner = new ProcessRunner(SynchronizationContext.Current!, Log, Running);

        Text = "GAGAME demo: install";
        StartPosition = FormStartPosition.CenterScreen;
        Size = new Size(980, 900);
        MinimumSize = new Size(820, 700);

        var top = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 1, AutoScroll = true };
        top.Controls.Add(CheckoutGroup());
        top.Controls.Add(CacheGroup());
        top.Controls.Add(KeyGroup());
        top.Controls.Add(DataGroup());

        var split = new SplitContainer { Dock = DockStyle.Fill, Orientation = Orientation.Horizontal, SplitterDistance = 620 };
        split.Panel1.Controls.Add(top);
        split.Panel2.Controls.Add(_log);

        var buttons = new FlowLayoutPanel { FlowDirection = FlowDirection.RightToLeft, Dock = DockStyle.Bottom, AutoSize = true, Padding = new Padding(8) };
        var cancel = new Button { Text = "Cancel", AutoSize = true, DialogResult = DialogResult.Cancel };
        buttons.Controls.AddRange([cancel, _finish]);
        _finish.Click += (_, _) => Finish();
        CancelButton = cancel;

        Controls.Add(split);
        Controls.Add(buttons);
        Load += (_, _) =>
        {
            RefreshCache(); RefreshKey(); RefreshData(); RefreshEngine();
            if (section is not null && _groups.TryGetValue(section, out var g)) top.ScrollControlIntoView(g);
        };
        FormClosed += (_, _) => { _gauge?.Cancel(); _runner.Dispose(); };
    }

    static NumericUpDown Num(decimal min, decimal max, int decimals = 0) =>
        new() { Minimum = min, Maximum = max, DecimalPlaces = decimals, Width = 110, ThousandsSeparator = true };

    GroupBox Group(string key, string title, Control body)
    {
        var g = new GroupBox { Text = title, Dock = DockStyle.Top, AutoSize = true, Padding = new Padding(10, 6, 10, 8) };
        body.Dock = DockStyle.Fill;
        g.Controls.Add(body);
        _groups[key] = g;
        return g;
    }

    static TableLayoutPanel Grid(params (string label, Control c)[] rows)
    {
        var t = new TableLayoutPanel { ColumnCount = 2, AutoSize = true };
        t.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
        t.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
        foreach (var (label, c) in rows)
        {
            var r = t.RowCount++;
            t.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            if (label.Length == 0) { t.Controls.Add(c, 0, r); t.SetColumnSpan(c, 2); continue; }
            t.Controls.Add(new Label { Text = label, AutoSize = true, Margin = new Padding(3, 7, 8, 3) }, 0, r);
            t.Controls.Add(c, 1, r);
        }
        return t;
    }

    static FlowLayoutPanel Flow(params Control[] cs)
    {
        var f = new FlowLayoutPanel { AutoSize = true, WrapContents = false, Margin = Padding.Empty };
        f.Controls.AddRange(cs);
        return f;
    }

    static Label Note(string text) => new() { Text = text, AutoSize = true, MaximumSize = new Size(760, 0), ForeColor = SystemColors.GrayText, Margin = new Padding(3, 2, 3, 6) };

    // ------------------------------------------------------------------ checkout

    Control CheckoutGroup()
    {
        _build.Click += (_, _) =>
        {
            if (!File.Exists(Path.Combine(_repo.Root, "build.bat"))) { Log("[demo] no build.bat in the checkout"); return; }
            _runner.Start(_repo.BuildStart(), "build.bat");
        };
        return Group(SectionCheckout, "Checkout", Grid(
            ("Repo", new Label { Text = _repo.Root, AutoSize = true, Margin = new Padding(3, 7, 3, 3) }),
            ("Engine", Flow(_engine, _build, Note("build.bat finds Visual Studio by vswhere and builds with CMake + Ninja (docs/LAUNCH.md, launch-build); a few minutes, logged below")))));
    }

    void RefreshEngine()
    {
        _engine.Text = "build\\bin\\gagame.exe: " + (_repo.EngineBuilt ? "built" : "NOT BUILT");
        _engine.ForeColor = _repo.EngineBuilt ? Color.DarkGreen : Color.Firebrick;
    }

    // ------------------------------------------------------------------ cache

    Control CacheGroup()
    {
        var browse = new Button { Text = "Browse...", AutoSize = true };
        browse.Click += (_, _) =>
        {
            using var d = new FolderBrowserDialog { Description = "The folder the cache lives in (a big disk; the repo's cache\\ becomes a junction to it)", UseDescriptionForTitle = true, SelectedPath = _cachePath.Text };
            if (d.ShowDialog(this) == DialogResult.OK) _cachePath.Text = d.SelectedPath;
        };
        var link = new Button { Text = "Link", AutoSize = true };
        link.Click += (_, _) =>
        {
            try { Log("[demo] " + _repo.LinkCache(_cachePath.Text)); }
            catch (Exception ex) { Log("[demo] link failed: " + ex.Message); }
            RefreshCache();
        };
        var browseBig = new Button { Text = "Browse...", AutoSize = true };
        browseBig.Click += (_, _) =>
        {
            using var d = new FolderBrowserDialog { Description = "The big-data drive for raw downloads", UseDescriptionForTitle = true, SelectedPath = _dataCache.Text };
            if (d.ShowDialog(this) == DialogResult.OK) _dataCache.Text = d.SelectedPath;
        };
        var gauge = new Button { Text = "Measure", AutoSize = true };
        gauge.Click += (_, _) => Gauge();
        var prune = new Button { Text = "List stale trees", AutoSize = true };
        prune.Click += (_, _) =>
        {
            if (!_repo.EngineBuilt) { Log("[demo] the engine is not built"); return; }
            _runner.Start(_repo.PruneListStart((double)_ageDays.Value), "tree-prune (list)");
        };

        _cachePath.Text = _settings.CacheRoot;
        _dataCache.Text = _settings.DataCache;
        _tileBudget.Value = _settings.TileBudget;
        _dayTiles.Value = _settings.DayTiles;
        _dayGb.Value = (decimal)(_settings.DayBytes / 1e9);
        _zoom.Value = _settings.GoogleZoom;
        _ageDays.Value = (decimal)_settings.PruneAgeDays;

        return Group(SectionCache, "Cache: where it lives and how much the imagery may fetch", Grid(
            ("", Note("The engine reads cache\\ beside its exe's checkout. Point that at a folder on a big disk: Link makes cache\\ a directory junction to it (nothing is moved; a cache folder with files in it is left alone and said so).")),
            ("Folder", Flow(_cachePath, browse, link, gauge)),
            ("Now", _cacheStatus),
            ("Big-data drive", Flow(_dataCache, browseBig, Note("where the harvesters keep raw downloads (CUDEM tiles, CUSP, NHD, BlueTopo, eHydro, OFS, EOT20): GAGAME_DATACACHE"))),
            ("", Note("Google tile limits (scene keys streaming.*; passed as --set on every launch). The day caps count every engine on the machine together, per UTC day; 0 = no request that day.")),
            ("Fetches per run", Flow(_tileBudget, Note("tileBudget: the hard cap on Google fetches in one run (3000 for a long ride)"))),
            ("Requests per day", Flow(_dayTiles, Note("dayTiles: requests sent, whatever came back"))),
            ("GB per day", Flow(_dayGb, Note("dayBytes: tiles landed whole"))),
            ("Finest zoom", Flow(_zoom, Note("googleZoom 0..19; each step finer is four times the tiles (14 is the shipped source)"))),
            ("Stale after (days)", Flow(_ageDays, prune, Note("tree-prune lists what the cache's tile trees hold and which are stale; it changes nothing. Retire and purge stay a command line (docs/LAUNCH.md, launch-data).")))));
    }

    void RefreshCache()
    {
        var (state, target) = _repo.CacheStatus();
        _cacheStatus.Text = state switch
        {
            Repo.CacheState.Missing => "no cache\\ yet: the engine would make a plain folder in the checkout on first run",
            Repo.CacheState.Junction => $"cache\\ is a junction to {target}",
            _ => $"cache\\ is a plain folder in the checkout",
        };
        if (string.IsNullOrWhiteSpace(_cachePath.Text))
            _cachePath.Text = state == Repo.CacheState.Missing ? _repo.CacheLink : target;
    }

    void Gauge()
    {
        _gauge?.Cancel();
        var cts = _gauge = new CancellationTokenSource();
        var path = _repo.CacheLink;
        if (!Directory.Exists(path)) { Log("[demo] nothing to measure yet"); return; }
        Log("[demo] measuring " + path + " ...");
        Task.Run(() =>
        {
            try
            {
                var bytes = Repo.FolderBytes(path, cts.Token);
                BeginInvoke(() => Log($"[demo] cache holds {Repo.Gb(bytes)}"));
            }
            catch (OperationCanceledException) { }
        });
    }

    // ------------------------------------------------------------------ key

    Control KeyGroup()
    {
        var show = new CheckBox { Text = "show", AutoSize = true, Margin = new Padding(6, 7, 3, 3) };
        show.CheckedChanged += (_, _) => _keyBox.UseSystemPasswordChar = !show.Checked;
        var save = new Button { Text = "Save key", AutoSize = true };
        save.Click += (_, _) =>
        {
            if (_keyBox.Text.Trim().Length == 0) { Log("[demo] nothing to save"); return; }
            Repo.WriteKey(_keyBox.Text);
            _keyBox.Clear();
            Log("[demo] key saved to the user environment as " + Repo.KeyVariable);
            RefreshKey(); RefreshData();
        };
        var clear = new Button { Text = "Remove key", AutoSize = true };
        clear.Click += (_, _) =>
        {
            if (MessageBox.Show(this, $"Remove {Repo.KeyVariable} from the user environment?", Text, MessageBoxButtons.YesNo, MessageBoxIcon.Question) != DialogResult.Yes) return;
            Repo.WriteKey("");
            Log("[demo] key removed");
            RefreshKey(); RefreshData();
        };
        var get = new LinkLabel { Text = "Get a Map Tiles API key", AutoSize = true, Margin = new Padding(3, 7, 3, 3) };
        get.LinkClicked += (_, _) => Open("https://developers.google.com/maps/documentation/tile/get-api-key");

        return Group(SectionKey, "Google Map Tiles key", Grid(
            ("Status", _keyStatus),
            ("Key", Flow(_keyBox, show, save, clear, get)),
            ("", Note($"The engine reads {Repo.KeyVariable} from the process environment, else from HKCU\\Environment, at boot. Save key writes that user variable (what `setx {Repo.KeyVariable} <key>` does) and nothing in the checkout: secrets\\ is gitignored and the key is never logged (length and prefix only). Without a key the Earth keeps its procedural look; imagery is bound by Google's terms."))));
    }

    void RefreshKey()
    {
        var key = Repo.ReadKey();
        _keyStatus.Text = Repo.DescribeKey(key);
        _keyStatus.ForeColor = key.Length == 0 ? Color.Firebrick : Color.DarkGreen;
    }

    // ------------------------------------------------------------------ data

    Control DataGroup()
    {
        _list.Columns.Add("Dataset", 230);
        _list.Columns.Add("Status", 80);
        _list.Columns.Add("Harvester", 170);
        _list.Columns.Add("Writes", 260);
        _list.Columns.Add("Insert", 90);
        _list.Height = 300;
        _list.SelectedIndexChanged += (_, _) => Selected();

        _download.Click += (_, _) => RunSelected(null);
        _insert.Click += (_, _) =>
        {
            if (Current() is not { Insert: not null } ds) return;
            using var d = new InsertDialog(_repo, ds);
            if (d.ShowDialog(this) == DialogResult.OK) RunSelected(d.Args);
        };
        _source.Click += (_, _) => OpenSources();
        _stop.Click += (_, _) => _runner.Stop();
        var fetchAll = new Button { Text = "Fetch all required", AutoSize = true };
        fetchAll.Click += (_, _) =>
        {
            if (_runner.IsRunning) { Log("[demo] something is already running"); return; }
            _queue.Clear();
            foreach (var name in Datasets.Required)
            {
                var ds = Datasets.All.FirstOrDefault(d => d.Script == name);
                if (ds is not null && !ds.Present(_repo)) _queue.Enqueue(name);
            }
            if (_queue.Count == 0) { Log("[demo] every required dataset is present"); return; }
            Log($"[demo] fetching {_queue.Count} required dataset(s) in order: {string.Join(", ", _queue)}");
            StartQueued();
        };
        var refresh = new Button { Text = "Refresh", AutoSize = true };
        refresh.Click += (_, _) => RefreshData();
        var sources = new LinkLabel { Text = "SOURCES.md", AutoSize = true, Margin = new Padding(12, 7, 3, 3) };
        sources.LinkClicked += (_, _) => Open(Path.Combine(_repo.Root, "SOURCES.md"));
        var launch = new LinkLabel { Text = "docs/LAUNCH.md", AutoSize = true, Margin = new Padding(6, 7, 3, 3) };
        launch.LinkClicked += (_, _) => Open(Path.Combine(_repo.Root, "docs", "LAUNCH.md"));

        var panel = new TableLayoutPanel { ColumnCount = 1, AutoSize = true, Dock = DockStyle.Fill };
        panel.Controls.Add(Note("Data is not in the repository (README). Each line is a product the engine reads and the harvester that fetches it, politely and cache-first, from its provider: Download runs it with its defaults; Source opens the provider; Insert runs it for your own place, window, file or flags. Harvesters run as `py -3 -I harvester\\<script>` from the checkout. A fresh clone renders nothing until these have run; the shipped scenes want at least Tides, Sea state, Currents, Bathymetry, Globe and Shorelines."));
        panel.Controls.Add(_list);
        panel.Controls.Add(_what);
        panel.Controls.Add(Flow(_download, _insert, _source, _stop, fetchAll, refresh, sources, launch));
        return Group(SectionData, "Data files", panel);
    }

    void RefreshData()
    {
        var keep = _list.SelectedIndices.Count > 0 ? _list.SelectedIndices[0] : -1;
        _list.BeginUpdate();
        _list.Items.Clear();
        foreach (var ds in Datasets.All)
        {
            var status = ds.Status(_repo);
            var item = new ListViewItem(ds.Name) { Tag = ds };
            item.SubItems.Add(status);
            item.SubItems.Add(ds.Script is null ? "(engine, at run time)" : ds.Script + (ds.DefaultArgs.Length > 0 ? " " + string.Join(' ', ds.DefaultArgs) : ""));
            item.SubItems.Add(ds.Outputs.Length > 0 ? string.Join(", ", ds.Outputs) : "");
            item.SubItems.Add(ds.Insert?.Kind switch
            {
                InsertKind.PlaceBox => "place / box",
                InsertKind.DropFolder => "drop files",
                InsertKind.FileArg => "file + box",
                InsertKind.Flags => "flags",
                _ => "",
            });
            item.ForeColor = status is "present" or "keyed" ? Color.DarkGreen : status is "missing" or "no key" ? Color.Firebrick : SystemColors.WindowText;
            _list.Items.Add(item);
        }
        _list.EndUpdate();
        if (keep >= 0 && keep < _list.Items.Count) _list.Items[keep].Selected = true;
        Selected();
    }

    Dataset? Current() => _list.SelectedItems.Count > 0 ? _list.SelectedItems[0].Tag as Dataset : null;

    void Selected()
    {
        var ds = Current();
        _what.Text = ds is null ? "" : ds.What + (ds.Note is null ? "" : "  " + ds.Note);
        var idle = !_runner.IsRunning;
        _download.Enabled = idle && ds is { Script: not null } && (ds.DefaultArgs.Length > 0 || ds.Insert?.Kind != InsertKind.FileArg);
        _insert.Enabled = idle && ds is { Insert: not null };
        _source.Enabled = ds is { Sources.Length: > 0 };
    }

    void RunSelected(List<string>? args)
    {
        if (Current() is not { Script: not null } ds) return;
        _queue.Clear();
        _runner.Start(_repo.HarvestStart(ds.Script, args ?? (IEnumerable<string>)ds.DefaultArgs, _dataCache.Text.Trim()), ds.Name);
    }

    void StartQueued()
    {
        if (_queue.Count == 0) return;
        var name = _queue.Dequeue();
        var ds = Datasets.All.First(d => d.Script == name);
        _runner.Start(_repo.HarvestStart(name, ds.DefaultArgs, _dataCache.Text.Trim()), ds.Name);
    }

    void OpenSources()
    {
        if (Current() is not { Sources.Length: > 0 } ds) return;
        if (ds.Sources.Length == 1) { Open(ds.Sources[0].url); return; }
        var menu = new ContextMenuStrip();
        foreach (var (label, url) in ds.Sources) menu.Items.Add(label, null, (_, _) => Open(url));
        menu.Show(_source, new Point(0, _source.Height));
    }

    static void Open(string target)
    {
        try { Process.Start(new ProcessStartInfo(target) { UseShellExecute = true }); }
        catch (Exception) { /* no handler: nothing to do */ }
    }

    // ------------------------------------------------------------------ run state, log, finish

    void Running(bool on)
    {
        _stop.Enabled = on;
        _finish.Enabled = !on;
        _build.Enabled = !on;
        if (!on) { RefreshData(); RefreshEngine(); StartQueued(); } else Selected();
    }

    void Log(string line)
    {
        _log.AppendText(line + Environment.NewLine);
    }

    void Finish()
    {
        _settings.CacheRoot = _cachePath.Text.Trim();
        _settings.DataCache = _dataCache.Text.Trim();
        _settings.TileBudget = (uint)_tileBudget.Value;
        _settings.DayTiles = (uint)_dayTiles.Value;
        _settings.DayBytes = (double)_dayGb.Value * 1e9;
        _settings.GoogleZoom = (uint)_zoom.Value;
        _settings.PruneAgeDays = (double)_ageDays.Value;
        _settings.Save();
    }
}
