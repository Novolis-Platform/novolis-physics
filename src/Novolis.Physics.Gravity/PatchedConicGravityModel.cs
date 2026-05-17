using Novolis.Physics.Abstractions;
using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Gravity;

/// <summary>Two-body patched conic: primary inside SOI, otherwise secondary point mass.</summary>
/// <remarks>Time-invariant; simulation time passed to <see cref="Evaluate"/> is ignored.</remarks>
public sealed class PatchedConicGravityModel : IForceModel<RigidBodyState, PatchedConicPairField>
{
    public ForceSample Evaluate(RigidBodyState body, PatchedConicPairField environment, double timeSeconds)
    {
        var toPrimary = body.Position - environment.PrimaryPosition;
        var insideSoi = toPrimary.Length() <= environment.PrimarySphereOfInfluenceRadius;
        var source = insideSoi ? environment.PrimaryPosition : environment.SecondaryPosition;
        var gm = insideSoi ? environment.PrimaryGm : environment.SecondaryGm;
        var r = source - body.Position;
        var distSq = r.LengthSquared();
        if (distSq < 1e-12)
        {
            return ForceSample.Zero;
        }

        var dist = System.Math.Sqrt(distSq);
        var dir = r.Divide(dist);
        var f = dir.Multiply(gm / distSq * body.Mass);
        return new ForceSample(f, Vector3.Zero);
    }
}
