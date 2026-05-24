using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Orbits;

/// <summary>Two-body Newtonian point mass in 3D: <c>a = −μ r / |r|³</c> (reduces to planar when Z and Vz are zero).</summary>
public static class OrbitalMath
{
    /// <summary>Newtonian central acceleration at <paramref name="position"/> with gravitational parameter <paramref name="mu"/>.</summary>
    public static Vector3 CentralAcceleration(Vector3 position, double mu)
    {
        var r2 = position.LengthSquared();
        if (r2 < 1e-24)
            return Vector3.Zero;

        var invR = 1.0 / global::System.Math.Sqrt(r2);
        var invR3 = invR / r2;
        return position.Multiply(-mu * invR3);
    }

    /// <summary>Specific orbital energy ε = v²/2 − μ/r (J/kg).</summary>
    public static double SpecificOrbitalEnergy(Vector3 position, Vector3 velocity, double mu) =>
        0.5 * velocity.LengthSquared() - mu / position.Length();

    /// <summary>Specific angular momentum vector h = r × v (m²/s).</summary>
    public static Vector3 SpecificAngularMomentumVector(Vector3 position, Vector3 velocity) =>
        Vector3.Cross(position, velocity);
}
