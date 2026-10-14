using System.Globalization;
using Gagame.Harvest;

namespace GagameDemo;

/// THE FIDELITY PAINTER (docs/FIDELITY.md section 1): paint, on a globe, the finest grain each
/// region of the planet wants, into a .gafid file every harvester reads. A brush stroke paints
/// lattice cells about a quarter of the brush radius across (FidelityRegions.Cap), each set to
/// the chosen grain; the cell says WHERE, the grain HOW FINE, so a 50 km brush can ask for 1 m
/// data. Erase returns the ground under the brush to the layer's default. The summary under the
/// controls is what the paint would cost a height layer: an estimate, not a source's holding.
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
        Controls.Add(Panel());
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
        _layer.SelectedIndexChanged += (_, _) => { _globe.Layer = (string)_layer.SelectedItem!; _default.Text = FidelityRegions.FormatLength(_map.Get(_globe.Layer).DefaultGrainM); _globe.MapChanged(); Summarize(); };

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
        _grainText.ForeColor = GlobeView.GrainColour(Grain) is var c && c.GetBrightness() > 0.75 ? Color.FromArgb(150, 120, 0) : GlobeView.GrainColour(Grain);
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
        _rotating = e.Button == MouseButtons.Right || (e.Button == MouseButtons.Left && _look.Checked);
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
        _globe.Brush = _look.Checked ? null : at;
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
        if (_erase.Checked) FidelityRegions.Clear(_map, _globe.Layer, cells);
        else _map.PaintMany(_globe.Layer, cells, Grain);
        Changed();
    }

    void Hover((double lat, double lon)? at)
    {
        if (at is not (double lat, double lon)) { _hover.Text = $"view {FidelityRegions.FormatLength(_globe.ViewWidthM)} across"; return; }
        var cells = _map.Get(_globe.Layer).Cells;
        string answer = "default";
        for (int L = FidelityMap.MaxLevel; L >= 0; --L)
        {
            var k = FidelityMap.CellOf(lat, lon, L);
            if (cells.ContainsKey(k)) { answer = "cell " + FidelityRegions.FormatKey(k); break; }
        }
        var here = FidelityMap.CellOf(lat, lon, FidelityRegions.LevelForExtent(_map, 2 * _globe.BrushRadiusM));
        _hover.Text = string.Create(CultureInfo.InvariantCulture,
            $"{lat:0.000000}, {lon:0.000000}   wants {FidelityRegions.FormatLength(_map.GrainAt(_globe.Layer, lat, lon))} ({answer})   brush cell {FidelityRegions.FormatKey(here)}   view {FidelityRegions.FormatLength(_globe.ViewWidthM)} across");
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
        if (erase) FidelityRegions.Clear(_map, _globe.Layer, cells);
        else _map.PaintMany(_globe.Layer, cells, grain);
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
            _map.Get(_globe.Layer, create: true).DefaultGrainM = g;
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

    void Changed()
    {
        _dirty = true;
        _globe.MapChanged();
        UpdateTitle();
    }

    void ShowMap()
    {
        _globe.Map = _map;
        _layer.Items.Clear();
        foreach (var l in _map.Layers) _layer.Items.Add(l.Name);
        if (_layer.Items.Count == 0) _layer.Items.Add(FidelityMap.AllLayers);
        _layer.SelectedItem = _layer.Items.Contains(_globe.Layer) ? _globe.Layer : _layer.Items[0];
        _default.Text = FidelityRegions.FormatLength(_map.Get(_globe.Layer).DefaultGrainM);
        _globe.MapChanged();
        UpdateTitle();
        Summarize();
    }

    void UpdateTitle() => Text = $"GAGAME: fidelity map - {(_path is null ? "untitled" : Path.GetFileName(_path))}{(_dirty ? " *" : "")}";

    /// Cells, the ground per grain band, and the height-layer estimate.
    void Summarize()
    {
        var l = _map.Get(_globe.Layer);
        var (bands, painted, rest, restBytes) = FidelityRegions.Stats(_map, _globe.Layer);
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
