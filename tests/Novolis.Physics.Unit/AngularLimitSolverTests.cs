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
        var childPos = new Vector3(0.45f, 1.35f, 0f);
        var spheres = new List<SphereState>
        {
            new(parent, Vector3.Zero),
            new(childPos, Vector3.Zero),
        };

        var rest = new Vector3(0.6f, -0.8f, 0f);
        rest = Vector3.Normalize(rest);
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

    [Test]
    public async Task SolveHinge_LocalFrame_FollowsRotatedTorso()
    {
        var hip = new Vector3(0f, 1f, 0f);
        var chest = new Vector3(0f, 1.5f, 0f);
        var knee = new Vector3(0.4f, 0.55f, 0.1f);
        var spheres = new List<SphereState>
        {
            new(hip, Vector3.Zero),
            new(chest, Vector3.Zero),
            new(knee, Vector3.Zero),
        };

        BoneFrame.TryCreate(hip, chest, out var frame);
        var restWorld = Vector3.Normalize(knee - hip);
        var restLocal = frame.WorldToLocal(restWorld);
        var axisLocal = frame.WorldToLocal(Vector3.UnitX);

        HingeLimit[] limits =
        [
            HingeLimit.CreateLocal(0, 2, frameReferenceSphere: 1, axisLocal, restLocal, minRadians: 0f, maxRadians: 0.35f, stiffness: 1f),
        ];

        spheres[2] = new SphereState(new Vector3(-0.5f, 1.35f, -0.1f), Vector3.Zero);

        var corrections = AngularLimitSolver.Solve(ReadOnlySpan<SwingLimit>.Empty, limits, spheres, iterations: 12);

        await Assert.That(corrections).IsGreaterThan(0);
        if (!BoneFrame.TryCreate(spheres[0].Position, spheres[1].Position, out var frameAfter))
            return;

        var restAfter = Vector3.Normalize(frameAfter.LocalToWorld(restLocal));
        var axisAfter = Vector3.Normalize(frameAfter.LocalToWorld(axisLocal));
        var bone = Vector3.Normalize(spheres[2].Position - spheres[0].Position);
        var angle = SignedAngle(
            Vector3.Normalize(ProjectOnPlane(restAfter, axisAfter)),
            Vector3.Normalize(ProjectOnPlane(bone, axisAfter)),
            axisAfter);
        await Assert.That(angle).IsLessThanOrEqualTo(0.4f);
    }

    [Test]
    public async Task SolveHinge_HipKnee_BlocksBackwardFold()
    {
        var hip = new Vector3(0f, 1f, 0f);
        var chest = new Vector3(0f, 1.5f, 0f);
        var knee = new Vector3(-0.2f, 0.5f, 0.06f);
        var spheres = new List<SphereState>
        {
            new(hip, Vector3.Zero),
            new(chest, Vector3.Zero),
            new(knee, Vector3.Zero),
        };

        BoneFrame.TryCreate(hip, chest, out var frame);
        var restLocal = frame.WorldToLocal(Vector3.Normalize(knee - hip));
        var axisLocal = new Vector3(1f, 0f, 0f);

        HingeLimit[] limits =
        [
            HingeLimit.CreateLocal(0, 2, 1, axisLocal, restLocal, minRadians: -0.1f, maxRadians: 1.65f, stiffness: 1f),
        ];

        spheres[2] = new SphereState(new Vector3(-0.15f, 1.35f, -0.2f), Vector3.Zero);

        AngularLimitSolver.Solve(ReadOnlySpan<SwingLimit>.Empty, limits, spheres, iterations: 16);

        var bone = Vector3.Normalize(spheres[2].Position - spheres[0].Position);
        await Assert.That(bone.Y).IsLessThan(0.25f);
    }

    private static float SignedAngle(Vector3 from, Vector3 to, Vector3 axis)
    {
        var sin = Vector3.Dot(Vector3.Cross(from, to), axis);
        var cos = Vector3.Dot(from, to);
        return MathF.Atan2(sin, cos);
    }

    private static Vector3 ProjectOnPlane(Vector3 v, Vector3 planeNormal) =>
        v - planeNormal * Vector3.Dot(v, planeNormal);
}
