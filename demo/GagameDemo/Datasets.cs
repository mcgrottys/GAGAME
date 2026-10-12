namespace GagameDemo;

/// How a dataset takes a custom insert: what the dialog asks for and how it becomes arguments.
public enum InsertKind
{
    /// A named place and its box: `--place NAME --box lon0,lat0,lon1,lat1` (+ the extras named).
    PlaceBox,
    /// Files dropped into a cache folder the harvester sweeps on its next run.
    DropFolder,
    /// A file the harvester takes as its first argument, plus a box.
    FileArg,
    /// Flags to tick (skip sections, add an optional product).
    Flags,
}

public sealed record Insert(
    InsertKind Kind,
    string Hint,
    string[]? Choices = null,       // PlaceBox: named windows offered instead of a box
    string? ChoiceFlag = null,      // PlaceBox: the flag a choice goes under (--window)
    string[]? Extras = null,        // PlaceBox: extra lon,lat flags asked for (--anchor, --sea, --near)
    string? Folder = null,          // DropFolder: the folder, from the repo root
    string? Pattern = null,         // DropFolder / FileArg: the files it takes
    string[]? Flags = null,         // Flags: each flag offered
    bool Final = false);            // PlaceBox: offer --final (the place manifest, written once)

/// One line of the install form's data list: a product the engine reads, the harvester that writes
/// it, where it comes from, and the custom insert it supports. Presence is judged by `Outputs`.
public sealed record Dataset(
    string Name,
    string What,
    string? Script,
    string[] DefaultArgs,
    string[] Outputs,
    (string label, string url)[] Sources,
    Insert? Insert = null,
    string? Note = null)
{
    public bool Present(Repo repo) =>
        Outputs.Length > 0 && Outputs.All(o => File.Exists(Path.Combine(repo.Root, o)));

    public string Status(Repo repo)
    {
        if (Script is null) return string.IsNullOrEmpty(Repo.ReadKey()) ? "no key" : "keyed";
        if (Outputs.Length == 0) return "(see log)";
        var n = Outputs.Count(o => File.Exists(Path.Combine(repo.Root, o)));
        return n == Outputs.Length ? "present" : n == 0 ? "missing" : $"{n}/{Outputs.Length}";
    }
}

/// The catalog, in the order the README's milestones brought each in. Every entry names the
/// harvester as docs/LAUNCH.md (launch-data) runs it and the primary source SOURCES.md cites.
public static class Datasets
{
    static readonly string[] Lonlat = ["--anchor", "--sea"];

    public static readonly Dataset[] All =
    [
        new("Tides (CO-OPS)",
            "Harmonic constituents and a year of official predictions per station, fit at the exact harcon frequencies.",
            "harvest_tides.py", [], ["data/tides/stations.json"],
            [("NOAA CO-OPS API", "https://api.tidesandcurrents.noaa.gov/api/prod/"),
             ("CO-OPS metadata API", "https://api.tidesandcurrents.noaa.gov/mdapi/prod/")],
            new Insert(InsertKind.PlaceBox, "A place's stations: every harmonic station within the box (--radius-km widens it).",
                Extras: ["--radius-km"])),

        new("Sea state (GFS-Wave + NDBC)",
            "GFS-Wave Hs/Tp/direction for the Gulf of Maine through the NOMADS grib filter, with NDBC buoy ground truth.",
            "harvest_waves.py", [], ["data/sea/seastate.json"],
            [("NOMADS GFS-Wave", "https://nomads.ncep.noaa.gov/"),
             ("NDBC realtime2", "https://www.ndbc.noaa.gov/data/realtime2/")],
            new Insert(InsertKind.PlaceBox, "A place's sea: the wave box, with the open-sea point the buoys are matched at.",
                Extras: Lonlat)),

        new("Currents (CO-OPS + GoMOFS)",
            "Tidal current predictions at the entrance stations, buoy ADCP, and the GoMOFS surface field.",
            "harvest_currents.py", [], ["data/currents/currents.json"],
            [("CO-OPS currents", "https://tidesandcurrents.noaa.gov/currents/"),
             ("GoMOFS", "https://tidesandcurrents.noaa.gov/ofs/gomofs/gomofs.html")],
            new Insert(InsertKind.PlaceBox, "A place's current stations within the box.", Extras: ["--radius-km"])),

        new("River (USGS)",
            "Merrimack discharge and stage at Lowell and Lawrence, the solver's upstream boundary.",
            "harvest_river.py", [], ["data/river/river.json"],
            [("USGS NWIS instantaneous values", "https://waterservices.usgs.gov/nwis/iv/")]),

        new("Bathymetry (CUDEM 1/9\")",
            "NOAA CUDEM topobathy for a window: jetties, beach and channel in one surface, metres NAVD88.",
            "harvest_bathy.py", ["--window", "merrimack"], ["data/bathy/merrimack.json", "data/bathy/merrimack.f32"],
            [("NOAA CUDEM", "https://coast.noaa.gov/htdata/raster2/elevation/"),
             ("NODD coastal lidar bucket", "https://registry.opendata.aws/noaa-coastal-lidar/")],
            new Insert(InsertKind.PlaceBox, "Another named window, or any place by its box (--near picks the tile order).",
                Choices: ["merrimack", "capeann", "boston"], ChoiceFlag: "--window", Extras: ["--near"])),

        new("HQ bed (BlueTopo + eHydro)",
            "The 1.5 m inlet seafloor from the primary surveys, the bed the wave solve and bedforms read.",
            "harvest_hqbathy.py", [], ["data/bathy/inlethq.f32"],
            [("NOAA BlueTopo", "https://registry.opendata.aws/noaa-ocs-nationalbathymetry/"),
             ("USACE eHydro", "https://navigation.usace.army.mil/Survey/Hydro")]),

        new("Globe (ETOPO + live sea)",
            "ETOPO 2022 relief decimated to 8192 x 4096, today's GFS-Wave Hs and wind, ocean colour, ice, clouds, Mars.",
            "harvest_globe.py", [], ["data/globe/globe.json", "data/globe/etopo_8192.i16"],
            [("ETOPO 2022", "https://www.ncei.noaa.gov/products/etopo-global-relief-model"),
             ("GFS-Wave on AWS", "https://registry.opendata.aws/noaa-gfs-bdp-pds/")],
            new Insert(InsertKind.Flags, "Refresh only some sections: tick what to SKIP.",
                Flags: ["--skip-etopo", "--skip-ne", "--skip-mars", "--skip-windvec", "--skip-clouds", "--skip-ocean", "--skip-ice", "--skip-waves"])),

        new("Shorelines (GSHHG)",
            "Full-resolution New England shoreline and rivers plus a low-resolution global coast, for the stencil.",
            "harvest_gis.py", [], ["data/gis/gis.json"],
            [("GSHHG", "https://www.ngdc.noaa.gov/mgg/shorelines/")]),

        new("Survey (CUSP + NHD)",
            "NGS CUSP coastline and USGS NHD HR waterbodies into the survey order; rebakes the land mask.",
            "harvest_survey.py", [], ["data/gis/survey.json"],
            [("NOAA CUSP", "https://geodesy.noaa.gov/dist_shoreline/"),
             ("USGS NHD HR", "https://www.usgs.gov/national-hydrography/national-hydrography-dataset")],
            Note: "Run Vectors after it to refresh the vpack."),

        new("Vectors (vpack)",
            "Every vector file as a lossless LOD manuscript: the GSHHG bins, the hand survey edits, any shapefile.",
            "harvest_vectors.py", [], ["data/vectors/vectors.vpack"],
            [("OSM ODbL (survey edits)", "https://www.openstreetmap.org/copyright")],
            new Insert(InsertKind.DropFolder, "Drop ESRI shapefiles (.shp with its .dbf) here; the next run folds them in.",
                Folder: "cache/vectors", Pattern: "*.shp")),

        new("Aerial orthos (MassGIS 2023)",
            "15 cm leaf-off orthos for the Merrimack inlet as RGB mip chains, the first independent high-res source.",
            "harvest_aerial.py", [], ["data/aerial/aerial.json"],
            [("MassGIS 2023 aerial imagery", "https://www.mass.gov/info-details/massgis-data-2023-aerial-imagery")],
            new Insert(InsertKind.DropFolder, "Drop MassGIS USNG tile zips (19T*.zip) here; needs Pillow for the JP2 decode.",
                Folder: "cache/aerial", Pattern: "19T*.zip")),

        new("Overlay (your GeoTIFFs)",
            "Any GeoTIFF as a compositor plane, alpha as fibre; with none present a demo highlights overlay is made.",
            "harvest_overlay.py", [], ["data/overlay/overlay.json"],
            [("GeoTIFF spec", "https://www.ogc.org/standard/geotiff/")],
            new Insert(InsertKind.DropFolder, "Drop GeoTIFFs (RGB or RGBA, UTM 19N) here; georeferencing is read from the tags.",
                Folder: "cache/overlay", Pattern: "*.tif")),

        new("Water atlas registry",
            "Every water source near the focus: tide and current stations, buoys, gauges, and the EOT20 global tide.",
            "harvest_water.py", [], ["data/water/registry.json"],
            [("EOT20 (SEANOE, CC BY 4.0)", "https://doi.org/10.17882/79489"),
             ("NDBC station table", "https://www.ndbc.noaa.gov/data/stations/station_table.txt")],
            new Insert(InsertKind.Flags, "Optional products.", Flags: ["--eot20"]),
            Note: "--eot20 fetches the 2.3 GB constituent grids."),

        new("Aids to navigation (ENC Direct)",
            "The charted S-57 buoys and lights from the live NOAA ENC cells.",
            "harvest_aton.py", [], ["data/gis/aton_ne.json"],
            [("NOAA ENC Direct to GIS", "https://encdirect.noaa.gov/")]),

        new("AIS lane (route prior)",
            "The vqview-inlet traffic lane georeferenced into engine coordinates: the fleet's route and speeds.",
            "harvest_route.py", [], ["data/gis/route_merrimack.json"],
            [("aisstream.io (the named live upgrade)", "https://aisstream.io/")],
            Note: "Reads the vqview-inlet lane file from its own checkout; not a download."),

        new("Buildings (OSM solids)",
            "Footprints with the heights OSM tagged, as prisms, from a .osm.pbf extract.",
            "harvest_buildings.py", [], [],
            [("Geofabrik extracts", "https://download.geofabrik.de/"),
             ("Overture Maps (heights)", "https://overturemaps.org/")],
            new Insert(InsertKind.FileArg, "Pick the .osm.pbf extract and the box (lon0,lat0,lon1,lat1) to keep.",
                Pattern: "OSM extracts (*.osm.pbf)|*.osm.pbf|All files|*.*"),
            Note: "No default download: pick an extract with Insert."),

        new("A whole place",
            "One name, one box, every source the Merrimack has: tides, currents or OFS, waves, bathy, survey, manifest.",
            "harvest_place.py", ["--place", "haulover"], ["data/places/haulover.json"],
            [("Place manifests (data/places)", "https://github.com/mcgrottys/GAGAME")],
            new Insert(InsertKind.PlaceBox, "Any place: its box, the anchor the chart is frozen at, and an open-sea point.",
                Extras: Lonlat, Final: true),
            Note: "Download = Haulover (the shipped demos' second place). Chesapeake is also named."),

        new("Google Map Tiles (imagery)",
            "Satellite imagery fetched at run time under your key and the day caps set above; without a key the Earth keeps its procedural look.",
            null, [], [],
            [("Get a Map Tiles API key", "https://developers.google.com/maps/documentation/tile/get-api-key"),
             ("Map Tiles API terms", "https://developers.google.com/maps/documentation/tile/policies")]),
    ];
}
