using System.Numerics;

namespace Novolis.Physics.Ballistics;

/// <summary>Air density and wind for ballistic drag (caller supplies ρ from atmosphere or constants).</summary>
public readonly struct ProjectileDragEnvironment
{
    public ProjectileDragEnvironment(double airDensityKgPerM3, Vector3 windMetersPerSecond = default)
    {
        AirDensityKgPerM3 = airDensityKgPerM3;
        WindMetersPerSecond = windMetersPerSecond;
    }

    public double AirDensityKgPerM3 { get; }
    public Vector3 WindMetersPerSecond { get; }
}
