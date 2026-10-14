namespace GagameDemo;

/// A rotor of Cl(3): R = S + B23 e23 + B31 e31 + B12 e12, |R| = 1. It turns a vector by the
/// sandwich v -> R v ~R. The painter's view orientation is one rotor (planet frame -> view frame);
/// a drag multiplies a rotor on the left, so there is no axis order and no gimbal.
public readonly record struct Rotor(double S, double B23, double B31, double B12)
{
    public static readonly Rotor Identity = new(1, 0, 0, 0);

    /// The geometric product of two rotors: scalar S1 S2 - b1.b2, bivector S1 b2 + S2 b1 - b1 x b2
    /// (e23 e31 = -e12 and its cycle give the minus on the cross term).
    public static Rotor operator *(Rotor a, Rotor b) => new(
        a.S * b.S - (a.B23 * b.B23 + a.B31 * b.B31 + a.B12 * b.B12),
        a.S * b.B23 + b.S * a.B23 - (a.B31 * b.B12 - a.B12 * b.B31),
        a.S * b.B31 + b.S * a.B31 - (a.B12 * b.B23 - a.B23 * b.B12),
        a.S * b.B12 + b.S * a.B12 - (a.B23 * b.B31 - a.B31 * b.B23));

    public Rotor Reverse => new(S, -B23, -B31, -B12);

    public Rotor Normalized()
    {
        double n = Math.Sqrt(S * S + B23 * B23 + B31 * B31 + B12 * B12);
        return new(S / n, B23 / n, B31 / n, B12 / n);
    }

    /// R v ~R. Written through the dual vector u = -(B23, B31, B12): v + 2S (u x v) + 2 u x (u x v),
    /// which is the sandwich expanded once.
    public void Apply(double x, double y, double z, out double ox, out double oy, out double oz)
    {
        double ux = -B23, uy = -B31, uz = -B12;
        double cx = uy * z - uz * y, cy = uz * x - ux * z, cz = ux * y - uy * x;
        double dx = uy * cz - uz * cy, dy = uz * cx - ux * cz, dz = ux * cy - uy * cx;
        ox = x + 2 * (S * cx + dx);
        oy = y + 2 * (S * cy + dy);
        oz = z + 2 * (S * cz + dz);
    }

    /// The rotor turning unit vector a onto unit vector b in their plane: (1 + b a) normalized,
    /// that is 1 + a.b - a^b. Antipodes have no plane; they take a half turn about any normal.
    public static Rotor Between(double[] a, double[] b)
    {
        double dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        // a ^ b on e23, e31, e12
        double w23 = a[1] * b[2] - a[2] * b[1], w31 = a[2] * b[0] - a[0] * b[2], w12 = a[0] * b[1] - a[1] * b[0];
        if (1 + dot < 1e-12)
        {
            // a half turn in the plane of a and any vector off it
            double[] n = Math.Abs(a[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
            w23 = a[1] * n[2] - a[2] * n[1]; w31 = a[2] * n[0] - a[0] * n[2]; w12 = a[0] * n[1] - a[1] * n[0];
            return new Rotor(0, -w23, -w31, -w12).Normalized();
        }
        return new Rotor(1 + dot, -w23, -w31, -w12).Normalized();
    }
}
