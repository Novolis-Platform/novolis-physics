using Novolis.Physics.Abstractions;
using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Aerodynamics;

/// <summary>Quadratic drag plus a crude lift term along (forward × relative velocity).</summary>
/// <remarks>Time-invariant; simulation time passed to <see cref="Evaluate"/> is ignored.</remarks>
public sealed class SimpleLiftDragModel : IForceModel<RigidBodyState, SimpleAeroEnvironment>
{
    public ForceSample Evaluate(RigidBodyState body, SimpleAeroEnvironment environment, double timeSeconds)
    {
        var rho = environment.Atmosphere.DensityAtAltitude(environment.AltitudeMeters);
        var v = body.Velocity - environment.WindWorld;
        var speed = v.Length();
        if (speed < 1e-6 || rho < 1e-30)
        {
            return ForceSample.Zero;
        }

        var q = 0.5 * rho * environment.ReferenceAreaM2 * speed * speed;
        var drag = (v / speed).Multiply(-environment.DragCoefficient * q);
        var liftAxis = Vector3.Cross(environment.LiftReferenceForwardWorld, v);
        var liftMag = liftAxis.Length();
        var lift = liftMag > 1e-12f
            ? liftAxis.Divide(liftMag).Multiply(environment.LiftCoefficient * q)
            : Vector3.Zero;
        return new ForceSample(drag + lift, Vector3.Zero);
    }
}
