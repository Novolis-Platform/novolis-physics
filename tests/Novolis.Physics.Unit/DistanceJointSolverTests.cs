using System.Numerics;
using Novolis.Physics.Collision.Simple;
using Novolis.Physics.Joints;

namespace Novolis.Physics.Unit;

[NotInParallel(NovolisPhysicsTestTrace.NotInParallelKey)]
public sealed class DistanceJointSolverTests
{
    [Test]
    public async Task Solve_PullsSpheresToRestLength()
    {
        var spheres = new List<SphereState>
        {
            new(new Vector3(0f, 0f, 0f), Vector3.Zero),
            new(new Vector3(3f, 0f, 0f), Vector3.Zero),
        };
        DistanceJoint[] joints = [new(0, 1, 2f)];

        var corrections = DistanceJointSolver.Solve(joints, spheres, iterations: 12);

        await Assert.That(corrections).IsGreaterThan(0);
        var dist = Vector3.Distance(spheres[0].Position, spheres[1].Position);
        await Assert.That(dist).IsEqualTo(2f).Within(0.02f);
    }
}
