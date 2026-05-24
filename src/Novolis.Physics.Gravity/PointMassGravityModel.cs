using Novolis.Physics.Abstractions;
using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Gravity;

/// <summary>Newtonian gravity from one or more point masses (GM per source in <see cref="PointMassField"/>).</summary>
/// <remarks>Time-invariant; simulation time passed to <see cref="Evaluate"/> is ignored.</remarks>
public sealed class PointMassGravityModel : IForceModel<RigidBodyState, PointMassField>
{
    /// <summary>Evaluate operation.</summary>
    public ForceSample Evaluate(RigidBodyState body, PointMassField environment, double timeSeconds)
    {
        var total = Vector3.Zero;
        var span = environment.Sources.Span;
        for (var i = 0; i < span.Length; i++)
        {
            var (pos, gm) = span[i];
            var r = pos - body.Position;
            var distSq = r.LengthSquared();
            if (distSq < 1e-12)
            {
                continue;
            }

            var dist = System.Math.Sqrt(distSq);
            var dir = r.Divide(dist);
            var magnitude = gm / distSq;
            total += dir.Multiply(magnitude);
        }

        return new ForceSample(total.Multiply(body.Mass), Vector3.Zero);
    }
}
