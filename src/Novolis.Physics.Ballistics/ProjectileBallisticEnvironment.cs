using System.Numerics;

namespace Novolis.Physics.Ballistics;

/// <summary>
/// Uniform downward gravity (−Y), optional air density for quadratic drag, and optional wind for drag relative to air.
/// <see cref="GravityMetersPerSecondSquared"/> is a positive magnitude; acceleration is <c>(0, −g, 0)</c> m/s².
/// Drag uses velocity relative to <see cref="WindMetersPerSecond"/> when density is non-zero.
/// <see cref="TemperatureKelvin"/> is the static temperature used for the speed of sound when a Mach drag law is attached.
/// <see cref="LatitudeDegrees"/> enables Coriolis in the East-Up-North frame when it is finite. NaN leaves Coriolis off.
/// </summary>
public readonly record struct ProjectileBallisticEnvironment(
    double GravityMetersPerSecondSquared,
    double AirDensityKgPerM3,
    Vector3 WindMetersPerSecond = default,
    double TemperatureKelvin = 288.15,
    double LatitudeDegrees = double.NaN);
