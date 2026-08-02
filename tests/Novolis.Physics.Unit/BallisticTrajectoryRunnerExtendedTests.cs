using Novolis.Physics.Abstractions;
using Novolis.Physics.Ballistics;
using Novolis.Physics.TestSupport;
using System.Numerics;
using TUnit.Core;

namespace Novolis.Physics.Unit;

[NotInParallel(NovolisPhysicsTestTrace.NotInParallelKey)]
public sealed class BallisticTrajectoryRunnerExtendedTests
{
    [Test]
    public async Task Reset_ReturnsToReadyAndClearsTrail()
    {
        var runner = new BallisticTrajectoryRunner();
        runner.Begin(new ProjectileState(new Vector3(0, 5, 0), new Vector3(0, -1, 0), 1, 0));
        runner.Reset();
        await Assert.That(runner.Phase).IsEqualTo(BallisticTrajectoryPhase.Ready);
        await Assert.That(runner.Trail.Count).IsEqualTo(0);
        await Assert.That(runner.Impact).IsNull();
    }

    [Test]
    public async Task AdvanceWithBudget_StopsAtMaxSteps()
    {
        var runner = new BallisticTrajectoryRunner(new BallisticTrajectoryRunnerOptions
        {
            Step = new ProjectileTerrainStepOptions { DtSeconds = 0.1, MaxSteps = 3, ProjectileRadius = 0.1f },
            RecordTrail = false,
        });
        var terrain = new OpenTerrain(heightAtZero: false);
        var sim = new ProjectileBallisticSimulation();
        var env = new ProjectileBallisticEnvironment(0, 0);
        runner.Begin(new ProjectileState(Vector3.Zero, new Vector3(1, 0, 0), 1, 0));

        runner.AdvanceWithBudget(sim, env, collisionWorld: null, terrain, Vector3.Zero, maxPhysicsSteps: 10);
        await Assert.That(runner.Phase).IsEqualTo(BallisticTrajectoryPhase.Impacted);
        await Assert.That(runner.Impact?.Reason).IsEqualTo(ProjectileTerrainImpactReason.MaxSteps);
    }

    [Test]
    public async Task BuildPreview_StopsAtHeightfieldOrRange()
    {
        var terrain = new OpenTerrain(heightAtZero: true, extent: 50f);
        var sim = new ProjectileBallisticSimulation();
        var env = new ProjectileBallisticEnvironment(9.80665, 0);
        var start = new ProjectileState(new Vector3(10, 20, 10), new Vector3(0, -5, 0), 1, 0);
        var points = BallisticTrajectoryRunner.BuildPreview(sim, env, terrain, start, dtSeconds: 0.05, maxTimeSeconds: 5, maxPoints: 200);
        await Assert.That(points.Count).IsGreaterThan(1);
        await Assert.That(points[^1].Y).IsLessThanOrEqualTo(20f);
    }

    [Test]
    public async Task BuildPreview_LeavesRange_ProjectsBoundary()
    {
        var terrain = new OpenTerrain(heightAtZero: false, extent: 20f);
        var sim = new ProjectileBallisticSimulation();
        var env = new ProjectileBallisticEnvironment(0, 0);
        var start = new ProjectileState(new Vector3(15, 5, 10), new Vector3(50, 0, 0), 1, 0);
        var points = BallisticTrajectoryRunner.BuildPreview(sim, env, terrain, start, 0.1, 2, 100);
        await Assert.That(points[^1].X).IsLessThanOrEqualTo(20.5f);
    }

    private sealed class OpenTerrain(bool heightAtZero = true, float extent = 200f) : IProjectileTerrainContact
    {
        private readonly AxisAlignedRangeBox _box = new(extent);

        public bool IsInside(float x, float z) => _box.IsInside(x, z);

        public bool TryHeightfieldContact(Vector3 position, float radius) =>
            heightAtZero && position.Y <= radius + 0.08f;

        public Vector3 ProjectOntoSurface(Vector3 position, float surfaceEpsilon = 0.05f) =>
            new(
                System.Math.Clamp(position.X, 0f, extent),
                surfaceEpsilon,
                System.Math.Clamp(position.Z, 0f, extent));

        public bool TrySegmentLeavesRange(Vector3 from, Vector3 to, out Vector3 hitPoint, out float fractionAlongSegment) =>
            _box.TrySegmentLeavesRange(from, to, out hitPoint, out fractionAlongSegment);
    }
}
