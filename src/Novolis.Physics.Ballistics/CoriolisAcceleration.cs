using System.Numerics;

namespace Novolis.Physics.Ballistics;

/// <summary>
/// Coriolis acceleration for a local East-Up-North frame: +X east, +Y up, +Z north.
/// Derived from the ENU form a = −2 ω × v with ω = (0, Ω cos φ, Ω sin φ), then swapped into this frame.
/// A northward shot in the northern hemisphere accelerates east. An eastward shot accelerates south and up (Eötvös).
/// </summary>
public static class CoriolisAcceleration
{
    /// <summary>Sidereal Earth rotation rate, rad/s.</summary>
    public const double EarthRotationRadiansPerSecond = 7.292115e-5;

    /// <summary>Acceleration in m/s² for <paramref name="groundVelocityMetersPerSecond"/> at <paramref name="latitudeDegrees"/>.</summary>
    public static Vector3 EastUpNorth(
        double latitudeDegrees,
        Vector3 groundVelocityMetersPerSecond,
        double rotationRadiansPerSecond = EarthRotationRadiansPerSecond)
    {
        var phi = global::System.Math.Clamp(latitudeDegrees, -90.0, 90.0) * (global::System.Math.PI / 180.0);
        var sin = global::System.Math.Sin(phi);
        var cos = global::System.Math.Cos(phi);
        var east = groundVelocityMetersPerSecond.X;
        var up = groundVelocityMetersPerSecond.Y;
        var north = groundVelocityMetersPerSecond.Z;
        var twoOmega = 2.0 * rotationRadiansPerSecond;
        return new Vector3(
            (float)(twoOmega * (sin * north - cos * up)),
            (float)(twoOmega * cos * east),
            (float)(-twoOmega * sin * east));
    }
}
