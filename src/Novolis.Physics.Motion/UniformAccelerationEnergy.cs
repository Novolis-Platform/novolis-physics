using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Motion;

/// <summary>
/// Mechanical energy for a point mass in a uniform acceleration field (constant <c>g</c> such that
/// <c>F = m g</c> in an inertial frame). Uses <c>PE = −m (g · r)</c> so <c>E = KE + PE</c> is conserved
/// along ballistic segments with no drag.
/// </summary>
public static class UniformAccelerationEnergy
{
    /// <summary>Kinetic energy <c>½ m |v|²</c> in joules.</summary>
    public static double KineticEnergyJ(double massKg, in Vector3 velocityMps)
    {
        ArgumentOutOfRangeException.ThrowIfLessThanOrEqual(massKg, 0.0);
        var v2 = Vector3.Dot(velocityMps, velocityMps);
        return 0.5 * massKg * v2;
    }

    /// <summary>Potential energy <c>−m (g · r)</c> in joules (reference-dependent).</summary>
    public static double PotentialEnergyJ(double massKg, in Vector3 uniformAccelerationMps2, in Vector3 positionM)
    {
        ArgumentOutOfRangeException.ThrowIfLessThanOrEqual(massKg, 0.0);
        return -massKg * Vector3.Dot(uniformAccelerationMps2, positionM);
    }

    /// <summary><see cref="KineticEnergyJ"/> + <see cref="PotentialEnergyJ"/>.</summary>
    public static double MechanicalEnergyJ(
        double massKg,
        in Vector3 velocityMps,
        in Vector3 uniformAccelerationMps2,
        in Vector3 positionM) =>
        KineticEnergyJ(massKg, velocityMps) + PotentialEnergyJ(massKg, uniformAccelerationMps2, positionM);
}
