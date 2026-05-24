using System.Numerics;

namespace Novolis.Physics.Ballistics;

/// <summary>Air density and wind for ballistic drag (caller supplies ρ from atmosphere or constants).</summary>
public readonly struct ProjectileDragEnvironment
{
    /// <summary>ProjectileDragEnvironment operation.</summary>
    public ProjectileDragEnvironment(double airDensityKgPerM3, Vector3 windMetersPerSecond = default)
    {
        AirDensityKgPerM3 = airDensityKgPerM3;
        WindMetersPerSecond = windMetersPerSecond;
    }
/// <summary>AirDensityKgPerM3.</summary>

    public double AirDensityKgPerM3 { get; }
    /// <summary>WindMetersPerSecond.</summary>
    public Vector3 WindMetersPerSecond { get; }
}
