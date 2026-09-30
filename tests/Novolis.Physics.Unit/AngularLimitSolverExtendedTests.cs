using Novolis.Physics.Collision.Simple;
using Novolis.Physics.Joints;
using System.Numerics;
using TUnit.Core;

namespace Novolis.Physics.Unit;

public sealed class AngularLimitSolverExtendedTests
{
    [Test]
    public async Task SolveSwing_WithFrameReference_UsesBoneFrame()
    {
        var hip = new Vector3(0f, 0f, 0f);
        var chest = new Vector3(0f, 1f, 0f);
        var hand = new Vector3(0.6f, 1.2f, 0f);
        var spheres = new List<SphereState>
        {
            new(hip, Vector3.Zero),
            new(chest, Vector3.Zero),
            new(hand, Vector3.Zero),
        };
        BoneFrame.TryCreate(hip, chest, out var frame);
        var restLocal = frame.WorldToLocal(Vector3.Normalize(hand - hip));
        SwingLimit[] limits =
        [
            SwingLimit.CreateLocal(0, 2, frameReferenceSphere: 1, restLocal, maxRadians: 0.35f, stiffness: 1f),
        ];
        spheres[2] = new SphereState(new Vector3(-0.5f, 1.5f, -0.3f), Vector3.Zero);
        var corrections = AngularLimitSolver.Solve(limits, ReadOnlySpan<HingeLimit>.Empty, spheres, iterations: 12);
        await Assert.That(corrections).IsGreaterThan(0);
    }

    [Test]
    public async Task SolveHinge_InvalidAxis_ReturnsZero()
    {
        var spheres = new List<SphereState>
        {
            new(Vector3.Zero, Vector3.Zero),
            new(new Vector3(0f, 1f, 0f), Vector3.Zero),
        };
        HingeLimit[] limits =
        [
            new(0, 1, Vector3.Zero, Vector3.Zero, minRadians: 0f, maxRadians: 1f, stiffness: 1f),
        ];
        var corrections = AngularLimitSolver.Solve(ReadOnlySpan<SwingLimit>.Empty, limits, spheres, iterations: 4);
        await Assert.That(corrections).IsEqualTo(0);
    }
}
