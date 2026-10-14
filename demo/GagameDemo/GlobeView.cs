using System.Drawing.Drawing2D;
using System.Drawing.Imaging;
using System.Text.Json;
using Gagame.Harvest;

namespace GagameDemo;

/// The fidelity painter's globe: an orthographic view of the planet, software rendered. The
/// orientation is one rotor (planet frame -> view frame: x right, y up, z toward the eye), the
/// zoom is pixels per planet radius. The base is the checkout's own data -- the globe relief
/// raster that data/globe/globe.json names, and every coastline run data/gis/gis.json names --
/// drawn under the graticule, the lattice's cells and the painted fidelity cells.
public sealed class GlobeView : Control
{
    [System.ComponentModel.DesignerSerializationVisibility(System.ComponentModel.DesignerSerializationVisibility.Hidden)]
    public Rotor View { get; set; } = Rotor.Identity;
    [System.ComponentModel.DesignerSerializationVisibility(System.ComponentModel.DesignerSerializationVisibility.Hidden)]
    public double PixelsPerRadius { get; set; } = 300;   // the zoom
    public FidelityMap Map = new();
    public string Layer = FidelityMap.AllLayers;
    public bool ShowGraticule = true, ShowLattice = true, ShowCoast = true, ShowCells = true;
    public double BrushRadiusM = 50_000;
    public (double lat, double lon)? Brush;
    public string BaseNote { get; private set; } = "loading the base ...";

    // the base: a relief raster (metres, int16) on a lat/lon grid, and coast runs as unit vectors
    short[]? _relief;
    int _nx, _ny;
    double _lon0, _lat0, _dlon, _dlat;
    readonly List<double[]> _coast = new();
    static readonly int[] ReliefLut = BuildLut();

    Bitmap? _base;          // the base at the resolution it was rendered
    (Rotor, double, Size, int)? _baseKey;
    Bitmap? _cells;         // the painted cells, opaque, composited with alpha
    (Rotor, double, Size, int, string)? _cellsKey;
    int _mapVersion;
    bool _interactive;
    readonly System.Windows.Forms.Timer _idle = new() { Interval = 220 };

    public GlobeView()
    {
        SetStyle(ControlStyles.AllPaintingInWmPaint | ControlStyles.UserPaint | ControlStyles.OptimizedDoubleBuffer | ControlStyles.ResizeRedraw, true);
        BackColor = Color.FromArgb(14, 16, 22);
        _idle.Tick += (_, _) => { _idle.Stop(); _interactive = false; Invalidate(); };
    }

    /// The scale that fits the whole globe, and the finest: a view a couple of kilometres across.
    public double MinScale => 0.15 * Math.Min(Width, Height);
    public double MaxScale => Math.Max(Width, 1) / (2000.0 / Map.PlanetRadiusM);

    /// The ground width the view spans, metres (the globe's diameter when it all fits).
    public double ViewWidthM => Math.Min(2 * Map.PlanetRadiusM, Width / PixelsPerRadius * Map.PlanetRadiusM);

    public void MapChanged() { _mapVersion++; Invalidate(); }

    /// Marks the view as moving: the base renders coarse until the input rests.
    public void Moving()
    {
        _interactive = true;
        _idle.Stop();
        _idle.Start();
        Invalidate();
    }

    // ------------------------------------------------------------------ the base data

    /// Reads the base off the checkout on a worker; the view repaints when it lands.
    public void LoadBase(string dataDir)
    {
        Task.Run(() =>
        {
            var notes = new List<string>();
            try
            {
                var gj = Path.Combine(dataDir, "globe", "globe.json");
                using var g = JsonDocument.Parse(File.ReadAllText(gj));
                var r = g.RootElement;
                var file = Path.Combine(dataDir, "globe", r.GetProperty("relief_file").GetString()!);
                int nx = r.GetProperty("nx").GetInt32(), ny = r.GetProperty("ny").GetInt32();
                var bytes = File.ReadAllBytes(file);
                if (bytes.Length != nx * ny * 2) throw new InvalidDataException($"{file}: {bytes.Length} bytes, not {nx} x {ny} int16");
                var relief = new short[nx * ny];
                Buffer.BlockCopy(bytes, 0, relief, 0, bytes.Length);
                (_nx, _ny, _lon0, _lat0, _dlon, _dlat) = (nx, ny, r.GetProperty("lon0").GetDouble(), r.GetProperty("lat0").GetDouble(),
                    r.GetProperty("dlon").GetDouble(), r.GetProperty("dlat").GetDouble());
                _relief = relief;
                notes.Add($"relief {Path.GetFileName(file)} ({nx} x {ny}, {r.GetProperty("dlon").GetDouble() * 111.195:0.#} km a cell)");
            }
            catch (Exception e) when (e is IOException or JsonException or KeyNotFoundException or InvalidDataException or UnauthorizedAccessException)
            {
                notes.Add("no relief raster (" + e.Message + ")");
            }
            try
            {
                using var gis = JsonDocument.Parse(File.ReadAllText(Path.Combine(dataDir, "gis", "gis.json")));
                foreach (var p in gis.RootElement.EnumerateObject())
                {
                    if (!p.Name.StartsWith("coast", StringComparison.Ordinal) || p.Value.ValueKind != JsonValueKind.String) continue;
                    var name = p.Value.GetString()!;
                    if (!name.EndsWith(".bin", StringComparison.OrdinalIgnoreCase)) continue;
                    int points = 0;
                    foreach (var line in PolylineBin.Read(Path.Combine(dataDir, "gis", name)))
                    {
                        var run = new double[line.Count * 3];
                        var d = new double[3];
                        for (int i = 0; i < line.Count; ++i)
                        {
                            FidelityMap.DirOfLatLon(line[i].lat, line[i].lon, d);
                            run[3 * i] = d[0]; run[3 * i + 1] = d[1]; run[3 * i + 2] = d[2];
                        }
                        lock (_coast) _coast.Add(run);
                        points += line.Count;
                    }
                    notes.Add($"{name} ({points:#,0} points)");
                }
            }
            catch (Exception e) when (e is IOException or JsonException or UnauthorizedAccessException)
            {
                notes.Add("no coastline (" + e.Message + ")");
            }
            BaseNote = "base: " + string.Join("; ", notes);
            BeginInvoke(() => { _baseKey = null; Invalidate(); });
        });
    }

    static int Argb(double r, double g, double b) =>
        unchecked((int)0xFF000000) | ((int)Math.Clamp(r, 0, 255) << 16) | ((int)Math.Clamp(g, 0, 255) << 8) | (int)Math.Clamp(b, 0, 255);

    /// Relief colour, -11 km .. +9 km by the metre: sea pale to deep blue, land green to rock to snow.
    static int[] BuildLut()
    {
        var lut = new int[20001];
        (double e, double r, double g, double b)[] sea = [(0, 168, 204, 228), (-200, 120, 170, 210), (-2000, 52, 98, 160), (-6000, 18, 40, 94), (-11000, 8, 18, 50)];
        (double e, double r, double g, double b)[] land = [(0, 96, 142, 86), (400, 128, 160, 96), (1200, 168, 158, 108), (2600, 150, 122, 96), (4500, 236, 236, 236), (9000, 255, 255, 255)];
        for (int i = 0; i < lut.Length; ++i)
        {
            double e = i - 11000;
            var stops = e < 0 ? sea : land;
            int k = 0;
            while (k < stops.Length - 2 && Math.Abs(e) > Math.Abs(stops[k + 1].e)) k++;
            double t = Math.Clamp((e - stops[k].e) / (stops[k + 1].e - stops[k].e), 0, 1);
            lut[i] = Argb(stops[k].r + t * (stops[k + 1].r - stops[k].r), stops[k].g + t * (stops[k + 1].g - stops[k].g), stops[k].b + t * (stops[k + 1].b - stops[k].b));
        }
        return lut;
    }

    double ReliefAt(double lat, double lon)
    {
        var rel = _relief!;
        double fx = (lon - _lon0) / _dlon - 0.5, fy = (lat - _lat0) / _dlat - 0.5;
        int x0 = (int)Math.Floor(fx), y0 = (int)Math.Floor(fy);
        double tx = fx - x0, ty = fy - y0;
        int Xw(int x) => ((x % _nx) + _nx) % _nx;
        int Yc(int y) => Math.Clamp(y, 0, _ny - 1);
        int xa = Xw(x0), xb = Xw(x0 + 1), ya = Yc(y0), yb = Yc(y0 + 1);
        double a = rel[ya * _nx + xa], b = rel[ya * _nx + xb], c = rel[yb * _nx + xa], d = rel[yb * _nx + xb];
        return (a + tx * (b - a)) * (1 - ty) + (c + tx * (d - c)) * ty;
    }

    // ------------------------------------------------------------------ projection

    double Cx => Width * 0.5;
    double Cy => Height * 0.5;

    /// The planet frame (x = lon 0, y = north, z = lon 90 E) is left-handed against east, north
    /// and out, so seen from outside it is a mirror image; a rotor cannot reflect, so the view
    /// reflects once, z -> -z, on the way in and out, and the rotor does the rest.
    public void ToView(double[] d, out double x, out double y, out double z) => View.Apply(d[0], d[1], -d[2], out x, out y, out z);

    public double[] ToPlanet(double x, double y, double z)
    {
        View.Reverse.Apply(x, y, z, out double a, out double b, out double c);
        return [a, b, -c];
    }

    /// A planet direction to the screen; false when it faces away (z < 0).
    public bool Project(double[] d, out PointF p, out double z)
    {
        ToView(d, out double x, out double y, out z);
        p = new PointF((float)(Cx + x * PixelsPerRadius), (float)(Cy - y * PixelsPerRadius));
        return z >= 0;
    }

    /// The planet direction under a screen point, or null off the globe.
    public double[]? Pick(double sx, double sy)
    {
        double x = (sx - Cx) / PixelsPerRadius, y = (Cy - sy) / PixelsPerRadius, r2 = x * x + y * y;
        if (r2 > 1) return null;
        return ToPlanet(x, y, Math.Sqrt(1 - r2));
    }

    public (double lat, double lon)? PickLatLon(double sx, double sy) =>
        Pick(sx, sy) is double[] d ? FidelityMap.LatLonOfDir(d) : null;

    /// The view's centre on the planet.
    public (double lat, double lon) CentreLatLon()
    {
        return FidelityMap.LatLonOfDir(ToPlanet(0, 0, 1));
    }

    /// Turns the globe so its centre looks at a point.
    public void LookAt(double lat, double lon)
    {
        var d = new double[3];
        FidelityMap.DirOfLatLon(lat, lon, d);
        // the point to the eye, then north up: turn about the view axis until the pole's image points up
        var r = Rotor.Between([d[0], d[1], -d[2]], [0, 0, 1]);
        r.Apply(0, 1, 0, out double nx, out double ny, out _);
        double ang = Math.Atan2(nx, ny);   // the pole's image angle from screen-up, clockwise
        // cos - sin e12 turns e1 toward e2 (counter-clockwise on screen) by ang: north back to up
        var spin = new Rotor(Math.Cos(ang / 2), 0, 0, -Math.Sin(ang / 2));
        View = (spin * r).Normalized();
    }

    // ------------------------------------------------------------------ painting

    protected override void OnPaint(PaintEventArgs e)
    {
        var g = e.Graphics;
        g.Clear(BackColor);
        if (Width < 4 || Height < 4) return;
        // coarse while moving: a quarter of the pixels; full resolution at rest
        int div = _interactive ? 2 : 1;
        var key = (View, PixelsPerRadius, Size, div);
        if (_baseKey != key || _base is null)
        {
            _base?.Dispose();
            _base = RenderBase(Math.Max(1, Width / div), Math.Max(1, Height / div), div);
            _baseKey = key;
        }
        g.InterpolationMode = div > 1 ? InterpolationMode.Bilinear : InterpolationMode.NearestNeighbor;
        g.PixelOffsetMode = PixelOffsetMode.Half;
        g.DrawImage(_base, new Rectangle(0, 0, Width, Height));
        g.PixelOffsetMode = PixelOffsetMode.Default;
        g.SmoothingMode = SmoothingMode.AntiAlias;

        if (ShowCells)
        {
            var ck = (View, PixelsPerRadius, Size, _mapVersion, Layer);
            if (_cellsKey != ck || _cells is null)
            {
                _cells?.Dispose();
                _cells = RenderCells();
                _cellsKey = ck;
            }
            using var ia = new ImageAttributes();
            ia.SetColorMatrix(new ColorMatrix { Matrix33 = 0.62f });
            g.DrawImage(_cells, new Rectangle(0, 0, Width, Height), 0, 0, Width, Height, GraphicsUnit.Pixel, ia);
        }
        if (ShowCoast) DrawCoast(g);
        if (ShowGraticule) DrawGraticule(g);
        if (ShowLattice) DrawLattice(g);
        DrawBrush(g);
        DrawLegend(g);
    }

    Bitmap RenderBase(int w, int h, int div)
    {
        var bmp = new Bitmap(w, h, PixelFormat.Format32bppArgb);
        var data = bmp.LockBits(new Rectangle(0, 0, w, h), ImageLockMode.WriteOnly, PixelFormat.Format32bppArgb);
        var px = new int[w * h];
        var inv = View.Reverse;
        int space = BackColor.ToArgb(), plain = Argb(40, 70, 120);
        bool hasRelief = _relief is not null;
        Parallel.For(0, h, row =>
        {
            double y = (Cy - (row + 0.5) * div) / PixelsPerRadius;
            for (int col = 0; col < w; ++col)
            {
                double x = ((col + 0.5) * div - Cx) / PixelsPerRadius, r2 = x * x + y * y;
                if (r2 > 1) { px[row * w + col] = space; continue; }
                double z = Math.Sqrt(1 - r2);
                inv.Apply(x, y, z, out double a, out double b, out double c);
                int colour = plain;
                if (hasRelief)
                {
                    double lat = Math.Asin(Math.Clamp(b, -1, 1)) / Geo.DegToRad, lon = Math.Atan2(-c, a) / Geo.DegToRad;   // the reflection back
                    colour = ReliefLut[Math.Clamp((int)Math.Round(ReliefAt(lat, lon)) + 11000, 0, ReliefLut.Length - 1)];
                }
                // the limb darkens a little so the sphere reads as one
                double shade = 0.6 + 0.4 * z;
                px[row * w + col] = Argb(((colour >> 16) & 255) * shade, ((colour >> 8) & 255) * shade, (colour & 255) * shade);
            }
        });
        System.Runtime.InteropServices.Marshal.Copy(px, 0, data.Scan0, px.Length);
        bmp.UnlockBits(data);
        return bmp;
    }

    // ------------------------------------------------------------------ the painted cells

    /// The grain's colour: inferno over the log of the grain, fine (1 mm) bright, coarse (100 km)
    /// dark; perceptually ordered so a finer cell always reads lighter.
    public static Color GrainColour(double grainM)
    {
        (double t, int r, int g, int b)[] k = [(0.0, 40, 11, 84), (0.14, 85, 15, 109), (0.29, 136, 34, 106), (0.43, 186, 54, 85),
            (0.57, 227, 89, 51), (0.71, 249, 140, 10), (0.86, 249, 201, 50), (1.0, 252, 255, 164)];
        double t = double.IsFinite(grainM) ? Math.Clamp(1 - (Math.Log10(Math.Max(grainM, 1e-9)) + 3) / 8, 0, 1) : 0;
        int i = 0;
        while (i < k.Length - 2 && t > k[i + 1].t) i++;
        double f = (t - k[i].t) / (k[i + 1].t - k[i].t);
        return Color.FromArgb((int)(k[i].r + f * (k[i + 1].r - k[i].r)), (int)(k[i].g + f * (k[i + 1].g - k[i].g)), (int)(k[i].b + f * (k[i + 1].b - k[i].b)));
    }

    /// A cell's outline on screen, its edges cut into segments by how big it shows; null when it
    /// faces away or lies off screen. Points behind the limb are pulled onto it.
    PointF[]? Outline(FidelityMap.CellKey c, bool mustFace)
    {
        double n = 1u << c.Level;
        double pxSize = FidelityRegions.CellCap(c).radius * 2 * PixelsPerRadius;
        int seg = Math.Clamp((int)(pxSize / 24), 1, 24);
        var pts = new PointF[4 * seg];
        var d = new double[3];
        bool front = false;
        float x0 = float.MaxValue, y0 = float.MaxValue, x1 = float.MinValue, y1 = float.MinValue;
        for (int e = 0; e < 4; ++e)
            for (int s = 0; s < seg; ++s)
            {
                double a = s / (double)seg;
                double u = e switch { 0 => a, 1 => 1, 2 => 1 - a, _ => 0 }, v = e switch { 0 => 0, 1 => a, 2 => 1, _ => 1 - a };
                FidelityMap.DirOfFaceUv(c.Face, (c.X + u) / n, (c.Y + v) / n, d);
                ToView(d, out double x, out double y, out double z);
                if (z >= 0) front = true;
                else { double l = Math.Sqrt(x * x + y * y); if (l > 0) { x /= l; y /= l; } }
                var p = new PointF((float)(Cx + x * PixelsPerRadius), (float)(Cy - y * PixelsPerRadius));
                pts[e * seg + s] = p;
                x0 = Math.Min(x0, p.X); y0 = Math.Min(y0, p.Y); x1 = Math.Max(x1, p.X); y1 = Math.Max(y1, p.Y);
            }
        if (mustFace && !front) return null;
        if (x1 < 0 || y1 < 0 || x0 > Width || y0 > Height) return null;
        return pts;
    }

    Bitmap RenderCells()
    {
        var bmp = new Bitmap(Math.Max(1, Width), Math.Max(1, Height), PixelFormat.Format32bppArgb);
        using var g = Graphics.FromImage(bmp);
        g.Clear(Color.Transparent);
        // coarse first: a deeper cell overrides the cell above it, as GrainAt reads
        foreach (var (k, grain) in Map.Get(Layer).Cells.OrderBy(kv => kv.Key.Level))
        {
            var pts = Outline(k, mustFace: true);
            if (pts is null) continue;
            using var b = new SolidBrush(GrainColour(grain));
            float w = pts.Max(p => p.X) - pts.Min(p => p.X), h = pts.Max(p => p.Y) - pts.Min(p => p.Y);
            if (w < 2 && h < 2) g.FillRectangle(b, pts[0].X - 1, pts[0].Y - 1, 2, 2);
            else g.FillPolygon(b, pts);
        }
        return bmp;
    }

    // ------------------------------------------------------------------ lines

    void DrawCoast(Graphics g)
    {
        using var pen = new Pen(Color.FromArgb(220, 30, 30, 30), 1f);
        var run = new List<PointF>();
        var d = new double[3];
        List<double[]> coast;
        lock (_coast) coast = _coast.ToList();
        foreach (var line in coast)
        {
            run.Clear();
            for (int i = 0; i < line.Length; i += 3)
            {
                d[0] = line[i]; d[1] = line[i + 1]; d[2] = line[i + 2];
                if (Project(d, out var p, out _)) run.Add(p);
                else { Flush(g, pen, run); run.Clear(); }
            }
            Flush(g, pen, run);
        }
    }

    void Flush(Graphics g, Pen pen, List<PointF> run)
    {
        if (run.Count < 2) return;
        float x0 = run.Min(p => p.X), x1 = run.Max(p => p.X), y0 = run.Min(p => p.Y), y1 = run.Max(p => p.Y);
        if (x1 < 0 || y1 < 0 || x0 > Width || y0 > Height) return;
        g.DrawLines(pen, run.ToArray());
    }

    /// Lines of latitude and longitude at a step that puts a few across the view.
    void DrawGraticule(Graphics g)
    {
        double spanDeg = ViewWidthM / (Map.PlanetRadiusM * Geo.DegToRad);
        double[] steps = [30, 10, 5, 2, 1, 0.5, 0.2, 0.1, 0.05, 0.02, 0.01, 0.005, 0.002, 0.001, 0.0005, 0.0002, 0.0001];
        double step = steps.FirstOrDefault(s => spanDeg / s >= 3, steps[^1]);
        var (clat, clon) = CentreLatLon();
        double latLo = Math.Max(-90, Math.Floor((clat - spanDeg) / step) * step), latHi = Math.Min(90, clat + spanDeg);
        double lonSpan = spanDeg >= 60 || Math.Abs(clat) + spanDeg > 85 ? 180 : spanDeg / Math.Cos(clat * Geo.DegToRad);
        double lonLo = Math.Floor((clon - lonSpan) / step) * step, lonHi = clon + lonSpan;
        int samples = 96;
        using var pen = new Pen(Color.FromArgb(110, 255, 255, 255), 1f);
        using var font = new Font("Segoe UI", 8f);
        var d = new double[3];
        var run = new List<PointF>();
        for (double lat = latLo; lat <= latHi + 1e-9; lat += step)
        {
            run.Clear();
            for (int i = 0; i <= samples; ++i)
            {
                FidelityMap.DirOfLatLon(lat, lonLo + (lonHi - lonLo) * i / samples, d);
                if (Project(d, out var p, out _)) run.Add(p); else { Flush(g, pen, run); run.Clear(); }
            }
            Flush(g, pen, run);
            FidelityMap.DirOfLatLon(lat, clon, d);
            if (Project(d, out var lp, out _) && lp.X >= 0 && lp.Y >= 0 && lp.X < Width && lp.Y < Height)
                g.DrawString(Deg(lat, step), font, Brushes.White, lp.X + 2, lp.Y - 14);
        }
        for (double lon = lonLo; lon <= lonHi + 1e-9; lon += step)
        {
            run.Clear();
            for (int i = 0; i <= samples; ++i)
            {
                FidelityMap.DirOfLatLon(latLo + (latHi - latLo) * i / samples, lon, d);
                if (Project(d, out var p, out _)) run.Add(p); else { Flush(g, pen, run); run.Clear(); }
            }
            Flush(g, pen, run);
        }
    }

    static string Deg(double v, double step)
    {
        int places = Math.Max(0, (int)Math.Ceiling(-Math.Log10(step) - 1e-9));
        return v.ToString("F" + places, System.Globalization.CultureInfo.InvariantCulture) + "°";
    }

    /// The lattice's cells at the level a brush of this view's size paints, near the view's centre.
    public int LatticeLevel => FidelityRegions.LevelForExtent(Map, ViewWidthM);

    void DrawLattice(Graphics g)
    {
        var (clat, clon) = CentreLatLon();
        var (cells, _) = FidelityRegions.Cap(Map, clat, clon, ViewWidthM * 0.75, LatticeLevel);
        using var pen = new Pen(Color.FromArgb(90, 120, 255, 255), 1f);
        foreach (var c in cells)
            if (Outline(c, mustFace: true) is PointF[] pts) g.DrawPolygon(pen, pts);
    }

    void DrawBrush(Graphics g)
    {
        if (Brush is not (double lat, double lon)) return;
        var c = new double[3];
        FidelityMap.DirOfLatLon(lat, lon, c);
        // two unit vectors normal to the centre span the circle's plane
        double[] t = Math.Abs(c[1]) < 0.9 ? [0, 1, 0] : [1, 0, 0];
        double[] e1 = [t[1] * c[2] - t[2] * c[1], t[2] * c[0] - t[0] * c[2], t[0] * c[1] - t[1] * c[0]];
        double l = Math.Sqrt(e1[0] * e1[0] + e1[1] * e1[1] + e1[2] * e1[2]);
        e1 = [e1[0] / l, e1[1] / l, e1[2] / l];
        double[] e2 = [c[1] * e1[2] - c[2] * e1[1], c[2] * e1[0] - c[0] * e1[2], c[0] * e1[1] - c[1] * e1[0]];
        double rho = Math.Min(BrushRadiusM / Map.PlanetRadiusM, Math.PI);
        var run = new List<PointF>();
        using var pen = new Pen(Color.White, 1.5f) { DashStyle = DashStyle.Dash };
        var d = new double[3];
        for (int i = 0; i <= 96; ++i)
        {
            double phi = 2 * Math.PI * i / 96;
            for (int k = 0; k < 3; ++k) d[k] = c[k] * Math.Cos(rho) + (e1[k] * Math.Cos(phi) + e2[k] * Math.Sin(phi)) * Math.Sin(rho);
            if (Project(d, out var p, out _)) run.Add(p); else { Flush(g, pen, run); run.Clear(); }
        }
        Flush(g, pen, run);
    }

    void DrawLegend(Graphics g)
    {
        int x = 12, y = Height - 44, w = 240, h = 12;
        using (var bg = new SolidBrush(Color.FromArgb(170, 0, 0, 0))) g.FillRectangle(bg, x - 6, y - 20, w + 12, 54);
        for (int i = 0; i < w; ++i)
        {
            double grain = Math.Pow(10, -3 + 8.0 * i / (w - 1));
            using var p = new Pen(GrainColour(grain));
            g.DrawLine(p, x + i, y, x + i, y + h);
        }
        using var font = new Font("Segoe UI", 8f);
        g.DrawString("grain wanted (log)", font, Brushes.White, x, y - 17);
        foreach (var (grain, label) in new[] { (1e-3, "1 mm"), (1.0, "1 m"), (1e3, "1 km"), (1e5, "100 km") })
        {
            float px = x + (float)((Math.Log10(grain) + 3) / 8 * (w - 1));
            g.DrawLine(Pens.White, px, y + h, px, y + h + 3);
            var sz = g.MeasureString(label, font);
            g.DrawString(label, font, Brushes.White, Math.Clamp(px - sz.Width / 2, x - 4, x + w - sz.Width + 4), y + h + 3);
        }
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing) { _idle.Dispose(); _base?.Dispose(); _cells?.Dispose(); }
        base.Dispose(disposing);
    }
}
