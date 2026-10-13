namespace Gagame.Harvest;

/// The harvesters' shared geodesy: distances on the sphere and the two projections the
/// Merrimack data arrives in (UTM 19N and Massachusetts State Plane Mainland), Snyder on GRS80
/// exactly as src/compose/Projections.h and the Python harvesters spelled them.
public static class Geo
{
    /// CPython's math.radians / math.degrees and numpy's: ONE multiply by a precomputed constant.
    public const double DegToRad = Math.PI / 180.0;
    public const double RadToDeg = 180.0 / Math.PI;

    public static double Haversine_km(double lat0, double lon0, double lat1, double lon1)
    {
        double rl0 = lat0 * DegToRad, rl1 = lat1 * DegToRad;
        var dlat = rl1 - rl0;
        var dlon = (lon1 - lon0) * DegToRad;
        var a = Math.Sin(dlat / 2) * Math.Sin(dlat / 2) + Math.Cos(rl0) * Math.Cos(rl1) * Math.Sin(dlon / 2) * Math.Sin(dlon / 2);
        return 2 * 6371.0 * Math.Asin(Math.Sqrt(a));
    }

    /// Distance (km) from a point to a lon/lat box; 0 inside (harvest_tides.box_km).
    public static double BoxKm(double lat, double lon, Box box)
    {
        var clat = Math.Min(Math.Max(lat, box.Lat0), box.Lat1);
        var clon = Math.Min(Math.Max(lon, box.Lon0), box.Lon1);
        return Haversine_km(lat, lon, clat, clon);
    }

    /// The flat-earth km of harvest_currents._km / harvest_place.km / harvest_bathy.km_to_cell.
    public static double FlatKm(double lon0, double lat0, double lon1, double lat1) =>
        Math.Sqrt(Sq((lon1 - lon0) * 111.32 * Math.Cos((lat0 + lat1) / 2 * DegToRad)) + Sq((lat1 - lat0) * 110.57));

    static double Sq(double x) => x * x;

    const double A = 6378137.0, F = 1 / 298.257222101, K0 = 0.9996;

    /// UTM 19N -> (lat, lon) degrees, GRS80: harvest_aerial.py's hand inverse, verbatim.
    public static (double lat, double lon) Utm19Inverse(double e, double n)
    {
        var e2 = F * (2 - F);
        var ep2 = e2 / (1 - e2);
        var e1 = (1 - Math.Sqrt(1 - e2)) / (1 + Math.Sqrt(1 - e2));
        var m = n / K0;
        var mu = m / (A * (1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256));
        var p1 = mu + (3 * e1 / 2 - 27 * e1 * e1 * e1 / 32) * Math.Sin(2 * mu)
                 + (21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32) * Math.Sin(4 * mu)
                 + (151 * e1 * e1 * e1 / 96) * Math.Sin(6 * mu);
        double sin1 = Math.Sin(p1), cos1 = Math.Cos(p1), tan1 = Math.Tan(p1);
        var c1 = ep2 * cos1 * cos1;
        var t1 = tan1 * tan1;
        var n1 = A / Math.Sqrt(1 - e2 * sin1 * sin1);
        var r1 = A * (1 - e2) / Math.Pow(1 - e2 * sin1 * sin1, 1.5);
        var d = (e - 500000.0) / (n1 * K0);
        var lat = p1 - (n1 * tan1 / r1) * (d * d / 2 - (5 + 3 * t1 + 10 * c1 - 4 * c1 * c1 - 9 * ep2) * Math.Pow(d, 4) / 24
                                           + (61 + 90 * t1 + 298 * c1 + 45 * t1 * t1 - 252 * ep2 - 3 * c1 * c1) * Math.Pow(d, 6) / 720);
        var lon = -69.0 * DegToRad + (d - (1 + 2 * t1 + c1) * Math.Pow(d, 3) / 6
                                           + (5 - 2 * c1 + 28 * t1 - 3 * c1 * c1 + 8 * ep2 + 24 * t1 * t1) * Math.Pow(d, 5) / 120) / cos1;
        return (lat * RadToDeg, lon * RadToDeg);
    }

    /// harvest_overlay.py's shorter inverse (fewer series terms): kept distinct so the overlay
    /// json's bbox comes out as it did.
    public static (double lat, double lon) Utm19InverseShort(double e, double n)
    {
        var e2 = F * (2 - F);
        var ep2 = e2 / (1 - e2);
        var e1 = (1 - Math.Sqrt(1 - e2)) / (1 + Math.Sqrt(1 - e2));
        var mu = (n / K0) / (A * (1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256));
        var p1 = mu + (3 * e1 / 2 - 27 * e1 * e1 * e1 / 32) * Math.Sin(2 * mu)
                 + (21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32) * Math.Sin(4 * mu)
                 + (151 * e1 * e1 * e1 / 96) * Math.Sin(6 * mu);
        double s1 = Math.Sin(p1), c1 = Math.Cos(p1), t1 = Math.Tan(p1);
        double cc = ep2 * c1 * c1, tt = t1 * t1;
        var n1 = A / Math.Sqrt(1 - e2 * s1 * s1);
        var r1 = A * (1 - e2) / Math.Pow(1 - e2 * s1 * s1, 1.5);
        var d = (e - 500000.0) / (n1 * K0);
        var lat = p1 - (n1 * t1 / r1) * (d * d / 2 - (5 + 3 * tt + 10 * cc - 4 * cc * cc - 9 * ep2) * Math.Pow(d, 4) / 24);
        var lon = -69.0 * DegToRad + (d - (1 + 2 * tt + cc) * Math.Pow(d, 3) / 6) / c1;
        return (lat * RadToDeg, lon * RadToDeg);
    }

    /// (lat, lon) degrees -> UTM 19N easting/northing: Projections.h::TransverseMercator.
    public static (double e, double n) Tm19Forward(double latDeg, double lonDeg)
    {
        var e2 = F * (2.0 - F);
        var ep2 = e2 / (1.0 - e2);
        var lat = latDeg * DegToRad;
        var lon = lonDeg * DegToRad;
        double s = Math.Sin(lat), c = Math.Cos(lat), t = Math.Tan(lat);
        var n_ = A / Math.Sqrt(1.0 - e2 * s * s);
        var T = t * t;
        var C = ep2 * c * c;
        var Aa = c * (lon - (-69.0 * DegToRad));
        var m = A * ((1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256) * lat
                     - (3 * e2 / 8 + 3 * e2 * e2 / 32 + 45 * e2 * e2 * e2 / 1024) * Math.Sin(2 * lat)
                     + (15 * e2 * e2 / 256 + 45 * e2 * e2 * e2 / 1024) * Math.Sin(4 * lat)
                     - (35 * e2 * e2 * e2 / 3072) * Math.Sin(6 * lat));
        var a2 = Aa * Aa;
        double a3 = a2 * Aa, a4 = a2 * a2, a5 = a4 * Aa, a6 = a4 * a2;
        var east = 500000.0 + K0 * n_ * (Aa + (1 - T + C) * a3 / 6.0 + (5 - 18 * T + T * T + 72 * C - 58 * ep2) * a5 / 120.0);
        var north = K0 * (m + n_ * t * (a2 / 2.0 + (5 - T + 9 * C + 4 * C * C) * a4 / 24.0
                                        + (61 - 58 * T + T * T + 600 * C - 330 * ep2) * a6 / 720.0));
        return (east, north);
    }

    /// State Plane MA Mainland METERS -> (lat, lon) degrees: the inverse LCC 2SP cone of
    /// Projections.h::MassMainland (sp 41d43' / 42d41', lat0 41, lon0 -71.5, FE 200000, FN 750000).
    public static (double lat, double lon) LccMassInverse(double xM, double yM)
    {
        var e = Math.Sqrt(F * (2.0 - F));
        double sp1 = 41.71666666666667 * DegToRad, sp2 = 42.68333333333333 * DegToRad;
        double lat0 = 41.0 * DegToRad, lon0 = -71.5 * DegToRad;
        double M(double p) => Math.Cos(p) / Math.Sqrt(1.0 - e * e * Math.Sin(p) * Math.Sin(p));
        double T(double p) => Math.Tan(Math.PI / 4.0 - p / 2.0) / Math.Pow((1.0 - e * Math.Sin(p)) / (1.0 + e * Math.Sin(p)), e / 2.0);
        var n = (Math.Log(M(sp1)) - Math.Log(M(sp2))) / (Math.Log(T(sp1)) - Math.Log(T(sp2)));
        var aF = A * M(sp1) / (n * Math.Pow(T(sp1), n));
        var rho0 = aF * Math.Pow(T(lat0), n);
        var xp = xM - 200000.0;
        var yp = yM - 750000.0;
        var rho = Math.CopySign(1.0, n) * Math.Sqrt(xp * xp + (rho0 - yp) * (rho0 - yp));
        var theta = Math.Atan2(xp, rho0 - yp);
        var tt = Math.Pow(rho / aF, 1.0 / n);
        var lat = Math.PI / 2.0 - 2.0 * Math.Atan(tt);
        for (var i = 0; i < 8; i++)
        {
            var es = e * Math.Sin(lat);
            lat = Math.PI / 2.0 - 2.0 * Math.Atan(tt * Math.Pow((1.0 - es) / (1.0 + es), e / 2.0));
        }
        return (lat * RadToDeg, (theta / n + lon0) * RadToDeg);
    }
}
