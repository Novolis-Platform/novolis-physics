using System.Numerics;

namespace Novolis.Physics.Ballistics;

/// <summary>
/// Uniform downward gravity (−Y), optional air density for quadratic drag, and optional wind for drag relative to air.
/// <see cref="GravityMetersPerSecondSquared"/> is a positive magnitude; acceleration is <c>(0, −g, 0)</c> m/s².
/// Drag uses velocity relative to <see cref="WindMetersPerSecond"/> when density is non-zero.
/// </summary>
public readonly record struct ProjectileBallisticEnvironment(
    double GravityMetersPerSecondSquared,
    double AirDensityKgPerM3,
    Vector3 WindMetersPerSecond = default);
