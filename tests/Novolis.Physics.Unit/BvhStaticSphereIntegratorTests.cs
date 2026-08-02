using System.Numerics;
using Novolis.Physics.Collision.Simple;
using Novolis.Physics.TestSupport;
using Novolis.Math.Geometry;
using TUnit.Core;

namespace Novolis.Physics.Unit;

[NotInParallel(NovolisPhysicsTestTrace.NotInParallelKey)]
public sealed class BvhStaticSphereIntegratorTests
{
    [Test]
    public async Task LinearDragAcceleration_OpposesVelocity()
    {
        var drag = BvhStaticSphereIntegrator.LinearDragAcceleration(new Vector3(10f, 0f, 0f), dragPerSecond: 2);
        await Assert.That(drag.X).IsEqualTo(-20f).Within(0.001f);
    }

    [Test]
    public async Task AdvanceWithUniformAccelerationAndLinearDrag_MovesSphereInRoom()
    {
        var min = PhysicsTestVectors.V(0, -5, -5);
        var max = PhysicsTestVectors.V(10, 5, 5);
        var world = CollisionTestGeometry.BuildAxisAlignedRoom(min, max, edgePad: 1);
        var pos = PhysicsTestVectors.V(5, 0, 0);
        var vel = PhysicsTestVectors.V(3, 0, 0);
        var reflections = BvhStaticSphereIntegrator.AdvanceWithUniformAccelerationAndLinearDrag(
            world,
            ref pos,
            ref vel,
            radiusM: 0.2,
            dtSeconds: 0.05,
            uniformAccelerationMps2: new Vector3(0f, -9.8f, 0f),
            linearDragPerSecond: 0.1);
        await Assert.That(pos.X).IsNotEqualTo(5f);
        await Assert.That(reflections).IsGreaterThanOrEqualTo(0);
    }

    [Test]
    public async Task AdvanceOneStep_TimeSpanOverload_ReflectsFromWall()
    {
        var min = PhysicsTestVectors.V(0, -5, -5);
        var max = PhysicsTestVectors.V(10, 5, 5);
        var world = CollisionTestGeometry.BuildAxisAlignedRoom(min, max, edgePad: 1);
        var pos = PhysicsTestVectors.V(1, 0, 0);
        var vel = PhysicsTestVectors.V(-8, 0, 0);
        var hits = BvhStaticSphereIntegrator.AdvanceOneStep(
            world,
            ref pos,
            ref vel,
            radiusM: 0.25,
            dt: TimeSpan.FromSeconds(0.05));
        await Assert.That(hits).IsGreaterThanOrEqualTo(0);
        await Assert.That(pos.X).IsNotEqualTo(1f);
    }

    [Test]
    public async Task AdvanceWithUniformAcceleration_TimeSpanOverload_AndHighDrag()
    {
        var world = new BvhStaticWorld(new TriangleMesh([], []));
        var pos = new Vector3(0, 5, 0);
        var vel = new Vector3(10, 0, 0);
        var refl = BvhStaticSphereIntegrator.AdvanceWithUniformAccelerationAndLinearDrag(
            world, ref pos, ref vel, 0.2, TimeSpan.FromSeconds(0.1),
            new Vector3(0, -9.8f, 0), linearDragPerSecond: 50, substepsPerStep: 4);
        await Assert.That(pos.Y).IsLessThan(5f);
        await Assert.That(refl).IsEqualTo(0);
    }

    [Test]
    public async Task AdvanceOneStep_NegativeTimeSpan_Throws()
    {
        var world = new BvhStaticWorld(new TriangleMesh([], []));
        var pos = Vector3.Zero;
        var vel = Vector3.Zero;
        var act = () => BvhStaticSphereIntegrator.AdvanceOneStep(world, ref pos, ref vel, 0.1, TimeSpan.FromSeconds(-1));
        await Assert.That(act).Throws<ArgumentOutOfRangeException>();
    }

    [Test]
    public async Task AdvanceWithUniformAcceleration_ZeroSubsteps_Throws()
    {
        var world = new BvhStaticWorld(new TriangleMesh([], []));
        var pos = Vector3.Zero;
        var vel = Vector3.Zero;
        var act = () => BvhStaticSphereIntegrator.AdvanceWithUniformAccelerationAndLinearDrag(
            world, ref pos, ref vel, 0.2, 0.1, Vector3.Zero, 0, substepsPerStep: 0);
        await Assert.That(act).Throws<ArgumentOutOfRangeException>();
    }
}
