using System.Diagnostics;
using Gagame.Harvest;

namespace GagameDemo;

/// A custom insert for one dataset: the fields its Insert spec asks for, turned into harvester
/// arguments (or, for a drop folder, the folder opened in Explorer and the next run armed).
public sealed class InsertDialog : Form
{
    readonly Dataset _ds;
    readonly Insert _ins;
    readonly Repo _repo;
    readonly TableLayoutPanel _grid = new() { Dock = DockStyle.Fill, ColumnCount = 2, AutoSize = true, Padding = new Padding(8) };
    readonly Dictionary<string, Control> _fields = new();
    readonly List<CheckBox> _flags = new();
    ComboBox? _choice;
    TextBox? _file;
    CheckBox? _final;

    /// The arguments to run the harvester with; empty when the dialog was cancelled.
    public List<string> Args { get; } = new();

    public InsertDialog(Repo repo, Dataset ds)
    {
        _repo = repo;
        _ds = ds;
        _ins = ds.Insert ?? throw new ArgumentException("dataset takes no insert");
        Text = $"Insert: {ds.Name}";
        StartPosition = FormStartPosition.CenterParent;
        FormBorderStyle = FormBorderStyle.FixedDialog;
        MaximizeBox = false; MinimizeBox = false;
        AutoSize = true; AutoSizeMode = AutoSizeMode.GrowAndShrink;
        MinimumSize = new Size(560, 0);

        _grid.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
        _grid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
        Row(new Label { Text = _ins.Hint, AutoSize = true, MaximumSize = new Size(520, 0), Margin = new Padding(3, 3, 3, 10) }, span: true);

        switch (_ins.Kind)
        {
            case InsertKind.PlaceBox: BuildPlaceBox(); break;
            case InsertKind.DropFolder: BuildDropFolder(); break;
            case InsertKind.FileArg: BuildFileArg(); break;
            case InsertKind.Flags: BuildFlags(); break;
        }

        var ok = new Button { Text = _ins.Kind == InsertKind.DropFolder ? "Run harvester now" : "Run", AutoSize = true, DialogResult = DialogResult.OK };
        var cancel = new Button { Text = "Cancel", AutoSize = true, DialogResult = DialogResult.Cancel };
        ok.Click += (_, e) => { if (!Collect()) DialogResult = DialogResult.None; };
        var buttons = new FlowLayoutPanel { FlowDirection = FlowDirection.RightToLeft, Dock = DockStyle.Bottom, AutoSize = true, Padding = new Padding(8) };
        buttons.Controls.AddRange([cancel, ok]);
        AcceptButton = ok; CancelButton = cancel;
        Controls.Add(_grid);
        Controls.Add(buttons);
    }

    void Row(Control c, bool span = false, string? label = null)
    {
        var r = _grid.RowCount++;
        _grid.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        if (span) { _grid.Controls.Add(c, 0, r); _grid.SetColumnSpan(c, 2); return; }
        _grid.Controls.Add(new Label { Text = label, AutoSize = true, Anchor = AnchorStyles.Left, Margin = new Padding(3, 7, 8, 3) }, 0, r);
        c.Anchor = AnchorStyles.Left | AnchorStyles.Right;
        _grid.Controls.Add(c, 1, r);
    }

    TextBox Field(string key, string label, string placeholder)
    {
        var t = new TextBox { PlaceholderText = placeholder, Width = 380 };
        _fields[key] = t;
        Row(t, label: label);
        return t;
    }

    void BuildPlaceBox()
    {
        if (_ins.Choices is { Length: > 0 })
        {
            _choice = new ComboBox { DropDownStyle = ComboBoxStyle.DropDownList, Width = 200 };
            _choice.Items.Add("(a place by its box)");
            _choice.Items.AddRange(_ins.Choices);
            _choice.SelectedIndex = 0;
            Row(_choice, label: "Named window");
        }
        Field("name", "Place name", "haulover");
        Field("box", "Box lon0,lat0,lon1,lat1", "-80.22,25.82,-80.03,26.02");
        foreach (var extra in _ins.Extras ?? [])
        {
            var (label, hint) = extra switch
            {
                "--anchor" => ("Anchor lon,lat", "the point the chart is frozen at, e.g. -80.1225,25.903"),
                "--sea" => ("Open sea lon,lat", "a point in open water, e.g. -80.03,25.9"),
                "--near" => ("Near lon,lat", "optional: fetch the tiles nearest this first"),
                "--radius-km" => ("Radius km", "optional: widen the station search"),
                _ => (extra, ""),
            };
            Field(extra, label, hint);
        }
        if (_ins.Final)
        {
            _final = new CheckBox { Text = "--final  (write data/places/<name>.json, once; otherwise a draft in out/places/)", AutoSize = true };
            Row(_final, span: true);
        }
    }

    void BuildDropFolder()
    {
        var folder = Path.Combine(_repo.Root, _ins.Folder!.Replace('/', '\\'));
        Directory.CreateDirectory(folder);
        var have = Directory.GetFiles(folder, _ins.Pattern ?? "*").Length;
        Row(new Label { Text = $"{folder}\n{have} file(s) matching {_ins.Pattern} there now.", AutoSize = true, MaximumSize = new Size(520, 0) }, span: true);
        var open = new Button { Text = "Open the folder in Explorer", AutoSize = true };
        open.Click += (_, _) => Process.Start(new ProcessStartInfo("explorer.exe", folder) { UseShellExecute = true });
        Row(open, span: true);
    }

    void BuildFileArg()
    {
        _file = new TextBox { Width = 380 };
        var pick = new Button { Text = "Browse...", AutoSize = true };
        pick.Click += (_, _) =>
        {
            using var d = new OpenFileDialog { Filter = _ins.Pattern ?? "All files|*.*" };
            if (d.ShowDialog(this) == DialogResult.OK) _file.Text = d.FileName;
        };
        var p = new FlowLayoutPanel { AutoSize = true, Margin = Padding.Empty };
        p.Controls.AddRange([_file, pick]);
        Row(p, label: "File");
        Field("--bbox", "Box lon0,lat0,lon1,lat1", "-70.90,42.78,-70.78,42.86");
        Field("--out", "Output (from the repo root)", "optional: data/buildings/<name>");
    }

    void BuildFlags()
    {
        foreach (var f in _ins.Flags ?? [])
        {
            var cb = new CheckBox { Text = f, AutoSize = true };
            _flags.Add(cb);
            Row(cb, span: true);
        }
    }

    static bool LonLat(string s, int pairs) =>
        s.Split(',').Length == 2 * pairs && s.Split(',').All(x => double.TryParse(x.Trim(), out _));

    bool Collect()
    {
        Args.Clear();
        string V(string k) => _fields.TryGetValue(k, out var c) ? c.Text.Trim() : "";
        switch (_ins.Kind)
        {
            case InsertKind.PlaceBox:
                if (_choice is { SelectedIndex: > 0 })
                {
                    Args.AddRange([_ins.ChoiceFlag!, (string)_choice.SelectedItem!]);
                    return true;
                }
                if (V("name").Length == 0 || !LonLat(V("box"), 2))
                    return Fail("A place needs a name and a box of four numbers: lon0,lat0,lon1,lat1.");
                Args.AddRange(["--place", V("name"), "--box", V("box")]);
                foreach (var extra in _ins.Extras ?? [])
                {
                    var v = V(extra);
                    if (v.Length == 0) continue;
                    if (extra != "--radius-km" && !LonLat(v, 1)) return Fail($"{extra} wants lon,lat.");
                    Args.AddRange([extra, v]);
                }
                if (_final?.Checked == true) Args.Add("--final");
                return true;

            case InsertKind.DropFolder:
                return true;   // the files are in the folder; a plain run sweeps them

            case InsertKind.FileArg:
                if (_file is null || !File.Exists(_file.Text)) return Fail("Pick the input file.");
                Args.Add(_file.Text);
                if (V("--bbox").Length > 0)
                {
                    if (!LonLat(V("--bbox"), 2)) return Fail("The box is four numbers: lon0,lat0,lon1,lat1.");
                    Args.AddRange(["--bbox", V("--bbox")]);
                }
                if (V("--out").Length > 0) Args.AddRange(["--out", V("--out")]);
                return true;

            case InsertKind.Flags:
                Args.AddRange(_flags.Where(c => c.Checked).Select(c => c.Text));
                return true;
        }
        return false;
    }

    bool Fail(string why)
    {
        MessageBox.Show(this, why, Text, MessageBoxButtons.OK, MessageBoxIcon.Information);
        return false;
    }
}
