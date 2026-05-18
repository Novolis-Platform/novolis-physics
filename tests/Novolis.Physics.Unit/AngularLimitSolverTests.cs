using System.Numerics;
using Novolis.Physics.Collision.Simple;
using Novolis.Physics.Joints;

namespace Novolis.Physics.Unit;

public sealed class AngularLimitSolverTests
{
    [Test]
    public async Task SolveHinge_ClampsBackwardBend()
    {
        var parent = new Vector3(0f, 1f, 0f);
        var childPos = new Vector3(0.5f, 1.4f, 0f);
        var spheres = new List<SphereState>
        {
            new(parent, Vector3.Zero),
            new(childPos, Vector3.Zero),
        };

        var rest = Vector3.Normalize(childPos - parent);
        var axis = Vector3.UnitZ;
        HingeLimit[] limits =
        [
            new(0, 1, axis, rest, minRadians: 0f, maxRadians: 0.4f, stiffness: 1f),
        ];

        var corrections = AngularLimitSolver.Solve(ReadOnlySpan<SwingLimit>.Empty, limits, spheres, iterations: 8);

        await Assert.That(corrections).IsGreaterThan(0);
        var bone = Vector3.Normalize(spheres[1].Position - spheres[0].Position);
        var angle = SignedAngle(rest, bone, axis);
        await Assert.That(angle).IsLessThanOrEqualTo(0.42f);
    }

    private static float SignedAngle(Vector3 from, Vector3 to, Vector3 axis)
    {
        var sin = Vector3.Dot(Vector3.Cross(from, to), axis);
        var cos = Vector3.Dot(from, to);
        return MathF.Atan2(sin, cos);
    }
}
