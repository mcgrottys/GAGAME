using System.Globalization;
using Gagame.Harvest;

namespace GagameDemo;

/// THE FIDELITY PAINTER (docs/FIDELITY.md section 1): paint, on a globe, the finest grain each
/// region of the planet wants, into a .gafid file every harvester reads. A brush stroke paints
/// lattice cells about a quarter of the brush radius across (FidelityRegions.Cap), each set to
/// the chosen grain; the cell says WHERE, the grain HOW FINE, so a 50 km brush can ask for 1 m
/// data. Erase returns the ground under the brush to the layer's default. The summary under the
/// controls is what the paint would cost a height layer: an estimate, not a source's holding.
/// A layer is any data layer the file, a composite or a source config names; painting one writes
/// that layer's cells. The Storage tab turns the same globe to what IS stored (StorageIndex).
public sealed class FidelityForm : Form
{
    const int UndoDepth = 32;

    readonly DemoSettings _settings;
    readonly Repo _repo;
    readonly GlobeView _globe = new() { Dock = DockStyle.Fill };
    readonly RadioButton _look = new() { Text = "Look", AutoSize = true, Checked = true };
    readonly RadioButton _paint = new() { Text = "Paint", AutoSize = true };
    readonly RadioButton _erase = new() { Text = "Erase", AutoSize = true };
    readonly ComboBox _layer = new() { DropDownStyle = ComboBoxStyle.DropDownList, Width = 140 };
    readonly TextBox _default = new() { Width = 90 };
    readonly TrackBar _grain = new() { Minimum = 0, Maximum = 80, TickFrequency = 10, Width = 210, Value = 30 };
    readonly Label _grainText = new() { AutoSize = true, Font = new Font("Segoe UI", 10f, FontStyle.Bold) };
    readonly TrackBar _radius = new() { Minimum = 0, Maximum = 126, TickFrequency = 20, Width = 210 };
    readonly NumericUpDown _radiusM = new() { Minimum = 1, Maximum = 2_000_000, DecimalPlaces = 0, Width = 110, ThousandsSeparator = true, Increment = 100 };
    readonly Label _radiusText = new() { AutoSize = true };
    readonly CheckBox _showGrat = new() { Text = "Graticule", AutoSize = true, Checked = true };
    readonly CheckBox _showLattice = new() { Text = "Lattice cells", AutoSize = true, Checked = true };
    readonly CheckBox _showCoast = new() { Text = "Coastline", AutoSize = true, Checked = true };
    readonly CheckBox _showCells = new() { Text = "Painted cells", AutoSize = true, Checked = true };
    readonly NumericUpDown _lon0 = Deg(-180, 180), _lat0 = Deg(-90, 90), _lon1 = Deg(-180, 180), _lat1 = Deg(-90, 90);
    readonly TextBox _boxGrain = new() { Width = 90, Text = "10 m" };
    readonly Label _summary = new() { AutoSize = true, MaximumSize = new Size(285, 0), Font = new Font("Consolas", 8.5f) };
    readonly ToolStripStatusLabel _hover = new("") { Spring = true, TextAlign = ContentAlignment.MiddleLeft };
    readonly ToolStripStatusLabel _note = new("");
    readonly ToolStripButton _undo = new("Undo") { Enabled = false, ToolTipText = "Undo the last stroke (Ctrl+Z)" };

    // the storage view
    readonly TabControl _tabs = new() { Dock = DockStyle.Left, Width = 340 };
    readonly FidelityOverlay _fidelity = new();
    readonly StorageOverlay _storage = new();
    readonly Label _feeds = new() { AutoSize = true, MaximumSize = new Size(300, 0), ForeColor = SystemColors.GrayText };
    readonly ComboBox _sSource = Combo(), _sParam = Combo(), _sStore = Combo(), _sLevel = Combo();
    readonly CheckBox _sCoverage = new() { Text = "Coverage boxes (who answers: composites)", AutoSize = true, Checked = true };
    readonly Label _sStatus = new() { AutoSize = true, MaximumSize = new Size(300, 0) };
    readonly Label _sHere = new() { AutoSize = true, MaximumSize = new Size(300, 0), Font = new Font("Consolas", 8f) };
    readonly ListView _sTotals = new() { View = View.Details, FullRowSelect = true, Height = 260, Width = 300, HideSelection = false };
    readonly Button _sRescan = new() { Text = "Rescan", AutoSize = true };
    bool _scanning;
    string _layerName = FidelityMap.AllLayers;

    static ComboBox Combo() => new() { DropDownStyle = ComboBoxStyle.DropDownList, Width = 290, DropDownWidth = 420 };

    bool StorageMode => _tabs.SelectedIndex == 1;

    FidelityMap _map = new();
    string? _path;
    bool _dirty;
    readonly LinkedList<FidelityMap> _history = new();
    bool _syncing;

    // the stroke and the drag
    Point? _dragFrom;
    bool _rotating;
    (double lat, double lon)? _lastDab;

    static NumericUpDown Deg(decimal min, decimal max) =>
        new() { Minimum = min, Maximum = max, DecimalPlaces = 5, Width = 96, Increment = 0.1m };

    public FidelityForm(Repo repo, DemoSettings settings, string? file = null)
    {
        _settings = settings;
        _repo = repo;
        Text = "GAGAME: fidelity map";
        StartPosition = FormStartPosition.CenterScreen;
        Size = new Size(1280, 860);
        MinimumSize = new Size(900, 600);
        KeyPreview = true;

        var bar = new ToolStrip { GripStyle = ToolStripGripStyle.Hidden, RenderMode = ToolStripRenderMode.System };
        void Button(string text, Action act, string tip) { var b = new ToolStripButton(text) { ToolTipText = tip }; b.Click += (_, _) => act(); bar.Items.Add(b); }
        Button("New", New, "A new map: one layer '*' wanting the coarsest everywhere");
        Button("Open...", () => Open(null), "Open a .gafid fidelity map");
        Button("Save", () => Save(_path), "Save (Ctrl+S)");
        Button("Save as...", () => Save(null), "Save under a new name");
        bar.Items.Add(new ToolStripSeparator());
        _undo.Click += (_, _) => Undo();
        bar.Items.Add(_undo);
        bar.Items.Add(new ToolStripSeparator());
        Button("Whole globe", () => { _globe.PixelsPerRadius = 0.45 * Math.Min(_globe.Width, _globe.Height); _globe.Moving(); SetRadius(_globe.BrushRadiusM); }, "Zoom out to the whole planet");

        var status = new StatusStrip();
        status.Items.AddRange([_hover, _note]);

        Controls.Add(_globe);
        Controls.Add(Tabs());
        Controls.Add(bar);
        Controls.Add(status);

        _globe.MouseDown += GlobeDown;
        _globe.MouseMove += GlobeMove;
        _globe.MouseUp += GlobeUp;
        _globe.MouseWheel += GlobeWheel;
        _globe.MouseLeave += (_, _) => { _globe.Brush = null; _globe.Invalidate(); };
        KeyDown += (_, e) =>
        {
            if (e.Control && e.KeyCode == Keys.Z) { Undo(); e.Handled = true; }
            if (e.Control && e.KeyCode == Keys.S) { Save(_path); e.Handled = true; }
        };
        FormClosing += (_, e) => { if (!ConfirmDiscard()) e.Cancel = true; };

        Load += (_, _) =>
        {
            _globe.LoadBase(repo.DataDir);
            _globe.PixelsPerRadius = 0.45 * Math.Min(_globe.Width, _globe.Height);
            var start = file ?? (File.Exists(_settings.LastFidelityFile) ? _settings.LastFidelityFile : null);
            if (start is not null) LoadFile(start); else ShowMap();
            var (lat, lon) = (_settings.FidelityViewLat, _settings.FidelityViewLon);
            _globe.LookAt(lat, lon);
            BoxFromView();
            SetRadius(_settings.FidelityBrushM);
            RefreshGrain();
        };
        var baseTimer = new System.Windows.Forms.Timer { Interval = 500 };
        baseTimer.Tick += (_, _) => { _note.Text = _globe.BaseNote; if (!_globe.BaseNote.StartsWith("loading", StringComparison.Ordinal)) baseTimer.Stop(); };
        baseTimer.Start();
        FormClosed += (_, _) =>
        {
            baseTimer.Dispose();
            var (lat, lon) = _globe.CentreLatLon();
            (_settings.FidelityViewLat, _settings.FidelityViewLon, _settings.FidelityBrushM) = (lat, lon, (double)_radiusM.Value);
            _settings.Save();
        };
    }

    // ------------------------------------------------------------------ the side panel

    /// Two views of the one globe: the fidelity map to paint, and what is stored.
    Control Tabs()
    {
        var paint = new TabPage("Fidelity");
        var store = new TabPage("Storage");
        var p = Panel();
        p.Dock = DockStyle.Fill;
        paint.Controls.Add(p);
        var s = StoragePanel();
        s.Dock = DockStyle.Fill;
        store.Controls.Add(s);
        _tabs.TabPages.AddRange([paint, store]);
        _globe.Overlay = _fidelity;
        _tabs.SelectedIndexChanged += (_, _) =>
        {
            _globe.Overlay = StorageMode ? _storage : _fidelity;
            _globe.Brush = null;
            if (StorageMode && _storage.Index.Folders.Count == 0 && !_scanning) Scan(rebuild: false);
            _showCells.Text = StorageMode ? "Stored bytes" : "Painted cells";
            _globe.Invalidate();
        };
        return _tabs;
    }

    Control Panel()
    {
        var t = new TableLayoutPanel { Dock = DockStyle.Left, Width = 310, ColumnCount = 1, AutoScroll = true, Padding = new Padding(8, 6, 6, 6) };
        void Add(Control c) { t.RowStyles.Add(new RowStyle(SizeType.AutoSize)); t.Controls.Add(c); }
        Label Head(string s) => new() { Text = s, AutoSize = true, Font = new Font("Segoe UI", 9f, FontStyle.Bold), Margin = new Padding(0, 10, 0, 2) };
        FlowLayoutPanel Row(params Control[] cs) { var f = new FlowLayoutPanel { AutoSize = true, WrapContents = false, Margin = new Padding(0) }; f.Controls.AddRange(cs); return f; }
        Label Note(string s) => new() { Text = s, AutoSize = true, Margin = new Padding(3, 7, 3, 3) };

        Add(Head("Mouse"));
        Add(Row(_look, _paint, _erase));
        Add(new Label { Text = "Left drag rotates in Look, paints or erases otherwise; right drag always rotates; wheel zooms.", AutoSize = true, MaximumSize = new Size(285, 0), ForeColor = SystemColors.GrayText });

        Add(Head("Layer"));
        var applyDefault = new Button { Text = "Set", AutoSize = true };
        applyDefault.Click += (_, _) => SetDefault();
        Add(Row(_layer));
        Add(Row(Note("default"), _default, applyDefault));
        Add(_feeds);
        _layer.SelectedIndexChanged += (_, _) =>
        {
            _layerName = (string)_layer.SelectedItem!;
            _default.Text = FidelityRegions.FormatLength(_map.Get(_layerName).DefaultGrainM);
            _fidelity.Show(_map, _layerName);
            ShowFeeds();
            MapChanged();
            Summarize();
        };

        Add(Head("Grain wanted"));
        Add(_grainText);
        Add(_grain);
        _grain.ValueChanged += (_, _) => RefreshGrain();

        Add(Head("Brush radius"));
        Add(_radius);
        Add(Row(_radiusM, Note("m")));
        Add(_radiusText);
        _radius.ValueChanged += (_, _) => { if (!_syncing) SetRadius(Math.Pow(10, _radius.Value / 20.0)); };
        _radiusM.ValueChanged += (_, _) => { if (!_syncing) SetRadius((double)_radiusM.Value); };

        Add(Head("Show"));
        Add(Row(_showGrat, _showLattice));
        Add(Row(_showCoast, _showCells));
        foreach (var c in new[] { _showGrat, _showLattice, _showCoast, _showCells })
            c.CheckedChanged += (_, _) =>
            {
                (_globe.ShowGraticule, _globe.ShowLattice, _globe.ShowCoast, _globe.ShowCells) = (_showGrat.Checked, _showLattice.Checked, _showCoast.Checked, _showCells.Checked);
                _globe.Invalidate();
            };

        Add(Head("Paint a box"));
        Add(Row(Note("lon0"), _lon0, Note("lat0"), _lat0));
        Add(Row(Note("lon1"), _lon1, Note("lat1"), _lat1));
        var paintBox = new Button { Text = "Paint box", AutoSize = true };
        var eraseBox = new Button { Text = "Erase box", AutoSize = true };
        var fromView = new Button { Text = "= view", AutoSize = true };
        paintBox.Click += (_, _) => Box(erase: false);
        eraseBox.Click += (_, _) => Box(erase: true);
        fromView.Click += (_, _) => BoxFromView();
        Add(Row(Note("grain"), _boxGrain));
        Add(Row(paintBox, eraseBox, fromView));

        Add(Head("Summary"));
        Add(_summary);
        return t;
    }

    // ------------------------------------------------------------------ grain and brush

    float Grain => (float)(1e-3 * Math.Pow(10, _grain.Value / 10.0));

    void RefreshGrain()
    {
        _grainText.Text = FidelityRegions.FormatLength(Grain);
        _grainText.ForeColor = FidelityOverlay.GrainColour(Grain) is var c && c.GetBrightness() > 0.75 ? Color.FromArgb(150, 120, 0) : FidelityOverlay.GrainColour(Grain);
    }

    void SetRadius(double m)
    {
        m = Math.Clamp(m, 1, 2_000_000);
        _syncing = true;
        _radius.Value = Math.Clamp((int)Math.Round(Math.Log10(m) * 20), _radius.Minimum, _radius.Maximum);
        _radiusM.Value = (decimal)Math.Round(m);
        _syncing = false;
        _globe.BrushRadiusM = m;
        int level = FidelityRegions.LevelForExtent(_map, 2 * m);
        _radiusText.Text = $"cells ~{FidelityRegions.FormatLength(_map.CellEdgeM(level))} (level {level})";
        _globe.Invalidate();
    }

    // ------------------------------------------------------------------ the mouse

    void GlobeDown(object? s, MouseEventArgs e)
    {
        _globe.Focus();
        _dragFrom = e.Location;
        _rotating = e.Button == MouseButtons.Right || (e.Button == MouseButtons.Left && (_look.Checked || StorageMode));
        if (!_rotating && e.Button == MouseButtons.Left)
        {
            Remember();
            _lastDab = null;
            Dab(e.Location);
        }
    }

    void GlobeMove(object? s, MouseEventArgs e)
    {
        var at = _globe.PickLatLon(e.X, e.Y);
        _globe.Brush = _look.Checked || StorageMode ? null : at;
        Hover(at);
        if (_dragFrom is Point from)
        {
            if (_rotating)
            {
                // the plane of the eye axis and the drag: the point under the cursor follows it
                double dx = e.X - from.X, dy = from.Y - e.Y, len = Math.Sqrt(dx * dx + dy * dy);
                if (len > 0)
                {
                    double ang = len / _globe.PixelsPerRadius;
                    var r = Rotor.Between([0, 0, 1], [dx / len * Math.Sin(ang), dy / len * Math.Sin(ang), Math.Cos(ang)]);
                    _globe.View = (r * _globe.View).Normalized();
                    _globe.Moving();
                }
                _dragFrom = e.Location;
            }
            else if (e.Button == MouseButtons.Left) Dab(e.Location);
        }
        _globe.Invalidate();
    }

    void GlobeUp(object? s, MouseEventArgs e)
    {
        bool painted = _dragFrom is not null && !_rotating;
        _dragFrom = null;
        if (painted) Summarize();
    }

    void GlobeWheel(object? s, MouseEventArgs e)
    {
        // zoom about the cursor: the ground under it stays under it
        var before = _globe.Pick(e.X, e.Y);
        double f = Math.Pow(1.25, e.Delta / 120.0);
        _globe.PixelsPerRadius = Math.Clamp(_globe.PixelsPerRadius * f, _globe.MinScale, _globe.MaxScale);
        if (before is not null && _globe.Pick(e.X, e.Y) is double[] after)
        {
            _globe.ToView(before, out double bx, out double by, out double bz);
            _globe.ToView(after, out double ax, out double ay, out double az);
            _globe.View = (Rotor.Between([bx, by, bz], [ax, ay, az]) * _globe.View).Normalized();
        }
        _globe.Moving();
        SetRadius(_globe.BrushRadiusM);
        Hover(_globe.PickLatLon(e.X, e.Y));
    }

    /// One dab of the brush, skipped until the cursor has moved half a radius on the ground.
    void Dab(Point p)
    {
        if (_globe.PickLatLon(p.X, p.Y) is not (double lat, double lon)) return;
        double r = _globe.BrushRadiusM;
        if (_lastDab is (double la, double lo))
        {
            double[] a = new double[3], b = new double[3];
            FidelityMap.DirOfLatLon(la, lo, a);
            FidelityMap.DirOfLatLon(lat, lon, b);
            if (FidelityRegions.Angle(a, b) * _map.PlanetRadiusM < 0.5 * r) return;
        }
        _lastDab = (lat, lon);
        var (cells, _) = FidelityRegions.Cap(_map, lat, lon, r, FidelityRegions.LevelForExtent(_map, 2 * r));
        if (_erase.Checked) FidelityRegions.Clear(_map, _layerName, cells);
        else _map.PaintMany(_layerName, cells, Grain);
        Changed();
    }

    void Hover((double lat, double lon)? at)
    {
        if (at is not (double lat, double lon)) { _hover.Text = $"view {FidelityRegions.FormatLength(_globe.ViewWidthM)} across"; return; }
        if (StorageMode) { StorageHover(lat, lon); return; }
        var cells = _map.Get(_layerName).Cells;
        string answer = "default";
        for (int L = FidelityMap.MaxLevel; L >= 0; --L)
        {
            var k = FidelityMap.CellOf(lat, lon, L);
            if (cells.ContainsKey(k)) { answer = "cell " + FidelityRegions.FormatKey(k); break; }
        }
        var here = FidelityMap.CellOf(lat, lon, FidelityRegions.LevelForExtent(_map, 2 * _globe.BrushRadiusM));
        _hover.Text = string.Create(CultureInfo.InvariantCulture,
            $"{lat:0.000000}, {lon:0.000000}   wants {FidelityRegions.FormatLength(_map.GrainAt(_layerName, lat, lon))} ({answer})   brush cell {FidelityRegions.FormatKey(here)}   view {FidelityRegions.FormatLength(_globe.ViewWidthM)} across");
    }

    // ------------------------------------------------------------------ boxes and the default

    void Box(bool erase)
    {
        double lon0 = (double)_lon0.Value, lat0 = (double)_lat0.Value, lon1 = (double)_lon1.Value, lat1 = (double)_lat1.Value;
        if (lon1 <= lon0 || lat1 <= lat0) { MessageBox.Show(this, "The box wants lon0 < lon1 and lat0 < lat1.", Text); return; }
        float grain;
        try { grain = FidelityRegions.ParseGrain(_boxGrain.Text); }
        catch (FormatException) { MessageBox.Show(this, $"Not a grain: '{_boxGrain.Text}' (try 30 m, 5 cm, 2 km).", Text); return; }
        Remember();
        var (cells, level) = FidelityRegions.Box(_map, lon0, lat0, lon1, lat1);
        if (erase) FidelityRegions.Clear(_map, _layerName, cells);
        else _map.PaintMany(_layerName, cells, grain);
        _note.Text = $"{(erase ? "cleared" : "painted")} {cells.Count} cells of level {level} ({FidelityRegions.FormatLength(_map.CellEdgeM(level))})";
        Changed();
        Summarize();
    }

    void BoxFromView()
    {
        var (clat, clon) = _globe.CentreLatLon();
        double half = 0.5 * _globe.ViewWidthM / (_map.PlanetRadiusM * Geo.DegToRad), h = half * _globe.Height / Math.Max(1, _globe.Width);
        double w = Math.Min(180, half / Math.Max(0.05, Math.Cos(clat * Geo.DegToRad)));
        _lon0.Value = (decimal)Math.Clamp(clon - w, -180, 180); _lon1.Value = (decimal)Math.Clamp(clon + w, -180, 180);
        _lat0.Value = (decimal)Math.Clamp(clat - h, -90, 90); _lat1.Value = (decimal)Math.Clamp(clat + h, -90, 90);
    }

    void SetDefault()
    {
        try
        {
            float g = FidelityRegions.ParseGrain(_default.Text);
            Remember();
            _map.Get(_layerName, create: true).DefaultGrainM = g;
            _default.Text = FidelityRegions.FormatLength(g);
            Changed();
            Summarize();
        }
        catch (FormatException) { MessageBox.Show(this, $"Not a grain: '{_default.Text}' (try 30 m, 2 km, or coarsest).", Text); }
    }

    // ------------------------------------------------------------------ the map and undo

    void Remember()
    {
        _history.AddLast(_map.Clone());
        while (_history.Count > UndoDepth) _history.RemoveFirst();
        _undo.Enabled = true;
    }

    void Undo()
    {
        if (_history.Last is not { } last) return;
        _history.RemoveLast();
        _map = last.Value;
        _undo.Enabled = _history.Count > 0;
        _dirty = true;
        ShowMap();
    }

    void MapChanged()
    {
        _fidelity.Changed();
        _globe.Invalidate();
    }

    void Changed()
    {
        _dirty = true;
        MapChanged();
        UpdateTitle();
    }

    void ShowMap()
    {
        _globe.Map = _map;
        // the layers: "*", the file's own, and every data layer a composite or a source config names
        _feedMap = LayerSources.Read(_repo.Root);
        var names = new SortedSet<string>(StringComparer.Ordinal) { FidelityMap.AllLayers };
        names.UnionWith(_map.Layers.Select(l => l.Name));
        names.UnionWith(_feedMap.Keys);
        _layer.Items.Clear();
        foreach (var n in names) _layer.Items.Add(n);
        if (!names.Contains(_layerName)) _layerName = FidelityMap.AllLayers;
        _layer.SelectedItem = _layerName;
        _fidelity.Show(_map, _layerName);
        ShowFeeds();
        _default.Text = FidelityRegions.FormatLength(_map.Get(_layerName).DefaultGrainM);
        MapChanged();
        UpdateTitle();
        Summarize();
    }

    void UpdateTitle() => Text = $"GAGAME: fidelity map - {(_path is null ? "untitled" : Path.GetFileName(_path))}{(_dirty ? " *" : "")}";

    /// Cells, the ground per grain band, and the height-layer estimate.
    void Summarize()
    {
        var l = _map.Get(_layerName);
        var (bands, painted, rest, restBytes) = FidelityRegions.Stats(_map, _layerName);
        var lines = new List<string> { $"{l.Cells.Count:#,0} cells, {painted / 1e6:#,0.##} km² painted" };
        foreach (var b in bands)
            lines.Add($"{(double.IsFinite(b.FromM) ? FidelityRegions.FormatLength(b.FromM) + "+" : "coarsest"),-8} {b.AreaM2 / 1e6,14:#,0.##} km²");
        if (restBytes > 0) lines.Add($"rest at {FidelityRegions.FormatLength(l.DefaultGrainM)}: {rest / 1e6:#,0} km²");
        lines.Add("");
        lines.Add($"ESTIMATE, height layer:");
        lines.Add($"  {FidelityRegions.FormatBytes(bands.Sum(b => b.HeightBytes) + restBytes)}");
        lines.Add("  (area / grain² x 4 B, +1/3 pyramid;");
        lines.Add("   no source's actual holding)");
        _summary.Text = string.Join(Environment.NewLine, lines);
    }

    // ------------------------------------------------------------------ layers and their sources

    SortedDictionary<string, List<LayerSources.Feed>> _feedMap = new();

    /// Which sources feed the layer, as the composites and the source configs name them.
    void ShowFeeds()
    {
        var lines = new List<string>();
        if (_layerName != FidelityMap.AllLayers && !_map.Layers.Any(l => l.Name == _layerName))
            lines.Add("Not in the file yet: its readers take '*' until it is painted (painting it starts it empty).");
        var feeds = _layerName == FidelityMap.AllLayers ? _feedMap.SelectMany(kv => kv.Value.Select(f => (layer: kv.Key, f))).ToList()
            : _feedMap.GetValueOrDefault(_layerName, new()).Select(f => (layer: _layerName, f)).ToList();
        lines.Add(feeds.Count == 0 ? "No composite or source config names this layer." : "Fed by:");
        foreach (var g in feeds.GroupBy(x => (x.layer, x.f.Id)).Take(16))
            lines.Add($"  {(_layerName == FidelityMap.AllLayers ? g.Key.layer + ": " : "")}{g.Key.Id} ({string.Join(", ", g.Select(x => x.f.From).Distinct())})");
        _feeds.Text = string.Join(Environment.NewLine, lines);
    }

    // ------------------------------------------------------------------ the storage view

    Control StoragePanel()
    {
        var t = new TableLayoutPanel { ColumnCount = 1, AutoScroll = true, Padding = new Padding(8, 6, 6, 6) };
        void Add(Control c) { t.RowStyles.Add(new RowStyle(SizeType.AutoSize)); t.Controls.Add(c); }
        Label Head(string s) => new() { Text = s, AutoSize = true, Font = new Font("Segoe UI", 9f, FontStyle.Bold), Margin = new Padding(0, 10, 0, 2) };
        Add(new Label
        {
            Text = "Bytes on disk, read by name and size (never a tile's payload): the tile trees and their archives, " +
                   "the composed cache, the composites' tiers and point files. Each in the lattice cell of its tile.",
            AutoSize = true, MaximumSize = new Size(300, 0), ForeColor = SystemColors.GrayText,
        });
        var rebuild = new Button { Text = "Rebuild", AutoSize = true };
        _sRescan.Click += (_, _) => Scan(rebuild: false);
        rebuild.Click += (_, _) => Scan(rebuild: true);
        var row = new FlowLayoutPanel { AutoSize = true, WrapContents = false, Margin = new Padding(0) };
        row.Controls.AddRange([_sRescan, rebuild]);
        Add(row);
        Add(_sStatus);
        Add(Head("Source")); Add(_sSource);
        Add(Head("Parameter")); Add(_sParam);
        Add(Head("Store")); Add(_sStore);
        Add(Head("Cell level")); Add(_sLevel);
        Add(_sCoverage);
        foreach (var c in new[] { _sSource, _sParam, _sStore, _sLevel }) c.SelectedIndexChanged += (_, _) => Refilter();
        _sCoverage.CheckedChanged += (_, _) => { _storage.ShowCoverage = _sCoverage.Checked; _globe.Invalidate(); };
        Add(Head("Totals (filtered)"));
        _sTotals.Columns.Add("parameter", 104);
        _sTotals.Columns.Add("store", 62);
        _sTotals.Columns.Add("bytes", 74, HorizontalAlignment.Right);
        _sTotals.Columns.Add("srcs", 38, HorizontalAlignment.Right);
        _sTotals.DoubleClick += (_, _) =>
        {
            // a row narrows the filters to its parameter and store
            if (_sTotals.SelectedItems.Count == 0 || _sTotals.SelectedItems[0].Tag is not (string p, string s)) return;
            Select(_sParam, p); Select(_sStore, s);
        };
        Add(_sTotals);
        Add(Head("Here"));
        Add(_sHere);
        return t;
    }

    static void Select(ComboBox c, string value)
    {
        foreach (var i in c.Items) if (i is Choice ch && ch.Value == value) { c.SelectedItem = i; return; }
    }

    /// A filter choice: the value it filters by (null = all) and its text with the bytes it holds.
    sealed record Choice(string? Value, string Text) { public override string ToString() => Text; }

    /// Brings the index up to date on a worker (the first scan of a cache reads every folder; a
    /// rescan, only the folders whose stamp moved), saves it, and shows it.
    void Scan(bool rebuild)
    {
        if (_scanning) return;
        _scanning = true;
        _sRescan.Enabled = false;
        _sStatus.Text = rebuild ? "rebuilding the index ..." : "scanning ...";
        var root = _repo.Root;
        Task.Run(() =>
        {
            var sw = System.Diagnostics.Stopwatch.StartNew();
            var x = rebuild ? new StorageIndex() : StorageIndex.Load(StorageIndex.DefaultPath, root);
            double load = sw.Elapsed.TotalSeconds;
            x.Update(root);
            if (x.Changed) x.Save(StorageIndex.DefaultPath);
            return (x, load);
        }).ContinueWith(t =>
        {
            _scanning = false;
            _sRescan.Enabled = true;
            if (t.Exception is { } e) { _sStatus.Text = "scan failed: " + e.GetBaseException().Message; return; }
            var (x, load) = t.Result;
            long size = File.Exists(StorageIndex.DefaultPath) ? new FileInfo(StorageIndex.DefaultPath).Length : 0;
            _sStatus.Text = $"{x.Folders.Count:#,0} folders: {x.FoldersRead:#,0} read, {x.FoldersReused:#,0} unchanged, in {x.ScanSeconds:0.0} s " +
                            $"(index loaded in {load:0.0} s, {size / 1e6:0.0} MB at {StorageIndex.DefaultPath}); " +
                            $"{FidelityRegions.FormatBytes(x.Entries.Sum(e => e.Bytes))} in cells, {FidelityRegions.FormatBytes(x.AllUnmapped.Sum(u => u.Bytes))} not in a cell";
            _storage.Show(x);
            FillFilters(x);
            Refilter();
        }, TaskScheduler.FromCurrentSynchronizationContext());
    }

    void FillFilters(StorageIndex x)
    {
        void Fill(ComboBox c, IEnumerable<(string key, long bytes)> items, Func<string, string>? label = null)
        {
            var keep = (c.SelectedItem as Choice)?.Value;
            c.Items.Clear();
            c.Items.Add(new Choice(null, "(all)"));
            foreach (var (k, b) in items.OrderByDescending(i => i.bytes)) c.Items.Add(new Choice(k, $"{(label?.Invoke(k) ?? k)}  ({FidelityRegions.FormatBytes(b)})"));
            c.SelectedIndex = 0;
            if (keep is not null) Select(c, keep);
        }
        var all = x.Entries.ToList();
        Fill(_sSource, all.GroupBy(e => e.Source).Select(g => (g.Key, g.Sum(e => e.Bytes))));
        Fill(_sParam, all.GroupBy(e => e.Param).Select(g => (g.Key, g.Sum(e => e.Bytes))));
        Fill(_sStore, all.GroupBy(e => e.Store).Select(g => (g.Key, g.Sum(e => e.Bytes))));
        Fill(_sLevel, all.GroupBy(e => StorageIndex.Unpack(e.Cell).Level).Select(g => (g.Key.ToString(CultureInfo.InvariantCulture), g.Sum(e => e.Bytes))),
            k => $"level {k}, cells ~{FidelityRegions.FormatLength(_map.CellEdgeM(int.Parse(k, CultureInfo.InvariantCulture)))}");
    }

    void Refilter()
    {
        _storage.Source = (_sSource.SelectedItem as Choice)?.Value;
        _storage.Param = (_sParam.SelectedItem as Choice)?.Value;
        _storage.Store = (_sStore.SelectedItem as Choice)?.Value;
        _storage.Level = (_sLevel.SelectedItem as Choice)?.Value is string l ? int.Parse(l, CultureInfo.InvariantCulture) : null;
        _storage.Refilter();
        _sTotals.BeginUpdate();
        _sTotals.Items.Clear();
        foreach (var g in _storage.Filtered.GroupBy(e => (e.Param, e.Store)).OrderByDescending(g => g.Sum(e => e.Bytes)).Take(200))
        {
            var it = new ListViewItem([g.Key.Param, g.Key.Store, FidelityRegions.FormatBytes(g.Sum(e => e.Bytes)), g.Select(e => e.Source).Distinct().Count().ToString(CultureInfo.InvariantCulture)])
            { Tag = (g.Key.Param, g.Key.Store), ToolTipText = $"{g.Sum(e => (long)e.Tiles):#,0} tiles" };
            _sTotals.Items.Add(it);
        }
        _sTotals.EndUpdate();
        _globe.Invalidate();
    }

    void StorageHover(double lat, double lon)
    {
        var (here, answers) = _storage.At(lat, lon);
        var lines = new List<string> { string.Create(CultureInfo.InvariantCulture, $"{lat:0.0000}, {lon:0.0000}: {FidelityRegions.FormatBytes(here.Sum(h => h.bytes))} stored here") };
        foreach (var (source, store, bytes, tiles) in here.Take(12))
            lines.Add($" {FidelityRegions.FormatBytes(bytes),9}  {store,-8} {source}");
        if (here.Count > 12) lines.Add($"  ... {here.Count - 12} more");
        if (answers.Count > 0)
        {
            lines.Add("Who answers here (coverage[]):");
            foreach (var c in answers.Take(8))
                lines.Add($"  {c.Layer} {FidelityRegions.FormatLength(c.GrainM)} {c.Input}{(c.GapFill ? " (gap fill)" : "")}  [{c.Product}]");
        }
        _sHere.Text = string.Join(Environment.NewLine, lines);
        _hover.Text = lines[0] + $"   cells at level {_globe.DisplayLevel}   view {FidelityRegions.FormatLength(_globe.ViewWidthM)} across";
    }

    // ------------------------------------------------------------------ files

    bool ConfirmDiscard()
    {
        if (!_dirty) return true;
        var r = MessageBox.Show(this, "Save the changes to the fidelity map first?", Text, MessageBoxButtons.YesNoCancel, MessageBoxIcon.Question);
        return r == DialogResult.No || (r == DialogResult.Yes && Save(_path));
    }

    void New()
    {
        if (!ConfirmDiscard()) return;
        _map = new FidelityMap();
        _path = null;
        _dirty = false;
        _history.Clear();
        _undo.Enabled = false;
        ShowMap();
    }

    void Open(string? path)
    {
        if (!ConfirmDiscard()) return;
        if (path is null)
        {
            using var d = new OpenFileDialog { Filter = "Fidelity map (*.gafid)|*.gafid|All files|*.*", InitialDirectory = Folder() };
            if (d.ShowDialog(this) != DialogResult.OK) return;
            path = d.FileName;
        }
        LoadFile(path);
    }

    void LoadFile(string path)
    {
        try
        {
            _map = FidelityMap.Load(path);
            _path = path;
            _dirty = false;
            _history.Clear();
            _undo.Enabled = false;
            _settings.LastFidelityFile = path;
            _settings.Save();
            ShowMap();
            _note.Text = $"opened {path}";
        }
        catch (Exception e) when (e is IOException or InvalidDataException or EndOfStreamException or UnauthorizedAccessException)
        {
            MessageBox.Show(this, e.Message, Text, MessageBoxButtons.OK, MessageBoxIcon.Warning);
        }
    }

    bool Save(string? path)
    {
        if (path is null)
        {
            using var d = new SaveFileDialog { Filter = "Fidelity map (*.gafid)|*.gafid", DefaultExt = "gafid", InitialDirectory = Folder(), FileName = Path.GetFileName(_path ?? "world.gafid") };
            if (d.ShowDialog(this) != DialogResult.OK) return false;
            path = d.FileName;
        }
        try
        {
            _map.Save(path);
            _path = path;
            _dirty = false;
            _settings.LastFidelityFile = path;
            _settings.Save();
            UpdateTitle();
            _note.Text = $"saved {path}";
            return true;
        }
        catch (Exception e) when (e is IOException or UnauthorizedAccessException)
        {
            MessageBox.Show(this, e.Message, Text, MessageBoxButtons.OK, MessageBoxIcon.Warning);
            return false;
        }
    }

    /// The folder a dialog opens in: the open file's, else data\fidelity in the checkout.
    string Folder()
    {
        if (_path is not null) return Path.GetDirectoryName(Path.GetFullPath(_path))!;
        var d = Path.Combine(_repo.DataDir, "fidelity");
        Directory.CreateDirectory(d);
        return d;
    }
}
