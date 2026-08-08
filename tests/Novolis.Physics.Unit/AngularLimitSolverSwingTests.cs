using System.Numerics;
using Novolis.Physics.Joints;
using Novolis.Physics.Collision.Simple;
using TUnit.Core;

namespace Novolis.Physics.Unit;

public sealed class AngularLimitSolverSwingTests
{
    [Test]
    public async Task SolveSwing_ClampsBoneOutsideCone()
    {
        var parent = new Vector3(0f, 0f, 0f);
        var childPos = new Vector3(0f, 0.5f, 1.5f);
        var spheres = new List<SphereState>
        {
            new(parent, Vector3.Zero),
            new(childPos, Vector3.Zero),
        };
        SwingLimit[] limits =
        [
            new(0, 1, Vector3.UnitY, maxRadians: 0.4f, stiffness: 1f),
        ];

        var corrections = AngularLimitSolver.Solve(limits, ReadOnlySpan<HingeLimit>.Empty, spheres, iterations: 10);
        await Assert.That(corrections).IsGreaterThan(0);
        var bone = Vector3.Normalize(spheres[1].Position - spheres[0].Position);
        await Assert.That(Vector3.Dot(bone, Vector3.UnitY)).IsGreaterThan(0.5f);
    }

    [Test]
    public async Task SolveSwing_InvalidIndices_ReturnZero()
    {
        var spheres = new List<SphereState> { new(Vector3.Zero, Vector3.Zero) };
        var corrections = AngularLimitSolver.Solve(
            [new SwingLimit(0, 5, Vector3.UnitY, 0.5f)],
            ReadOnlySpan<HingeLimit>.Empty,
            spheres,
            iterations: 2);
        await Assert.That(corrections).IsEqualTo(0);
    }
}
