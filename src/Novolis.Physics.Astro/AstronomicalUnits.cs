using System.Numerics;

namespace Novolis.Physics.Astro;

/// <summary>SI conversion helpers for light-years, parsecs, and astronomical units.</summary>
public static class AstronomicalUnits
{
    /// <summary>Meters in one light-year (IAU conventional).</summary>
    public const double MetersPerLy = 9.4607304725808e15;

    /// <summary>Meters in one parsec.</summary>
    public const double MetersPerPc = 3.0856775814913673e16;

    /// <summary>Meters in one astronomical unit.</summary>
    public const double MetersPerAu = 1.495978707e11;

    /// <summary>Converts light-years to meters.</summary>
    public static double LyToMeters(double ly) => ly * MetersPerLy;

    /// <summary>Converts meters to light-years.</summary>
    public static double MetersToLy(double meters) => meters / MetersPerLy;

    /// <summary>Converts parsecs to meters.</summary>
    public static double PcToMeters(double pc) => pc * MetersPerPc;

    /// <summary>Converts meters to parsecs.</summary>
    public static double MetersToPc(double meters) => meters / MetersPerPc;

    /// <summary>Converts astronomical units to meters.</summary>
    public static double AuToMeters(double au) => au * MetersPerAu;

    /// <summary>Converts meters to astronomical units.</summary>
    public static double MetersToAu(double meters) => meters / MetersPerAu;

    /// <summary>Maps light-year XYZ into a <see cref="Vector3"/> in meters (float precision).</summary>
    public static Vector3 LightYearsToVector3(double xLy, double yLy, double zLy) =>
        new((float)LyToMeters(xLy), (float)LyToMeters(yLy), (float)LyToMeters(zLy));
}
