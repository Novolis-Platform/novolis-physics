using Novolis.Physics.TestSupport;
using System.Numerics;
using Novolis.Math.Geometry;
using Novolis.Physics.Orbits;

namespace Novolis.Physics.TestSupport.Orbits;

/// <summary>Canonical initial conditions for the elliptical Earth test scenario (periapsis on +X, velocity along +Y).</summary>
public static class OrbitalTestState
{
    public static OrbitState CreatePeriapsisState()
    {
        var rp = OrbitalTestConstants.PeriapsisRadius;
        var vp = OrbitalTestConstants.PeriapsisSpeed;
        return new OrbitState(PhysicsTestVectors.V((float)rp, 0f, 0f), PhysicsTestVectors.V(0f, (float)vp, 0f));
    }
}
