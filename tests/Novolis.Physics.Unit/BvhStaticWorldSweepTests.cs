using Novolis.Physics.Abstractions;
using Novolis.Physics.Ballistics;
using Novolis.Physics.Collision.Simple;
using Novolis.Physics.TestSupport;
using System.Numerics;
using Novolis.Math.Geometry;
using TUnit.Core;

namespace Novolis.Physics.Unit;

public sealed class BvhStaticWorldSweepTests
{
    [Test]
    public async Task SweepSphere_HitsGroundBeforeFullTravel()
    {
        var mesh = new TriangleMesh(
            [new Vector3(0, 0, 0), new Vector3(20, 0, 0), new Vector3(0, 0, 20)],
            [0, 1, 2]);
        var world = new BvhStaticWorld(mesh);
        var sphere = new Sphere(new Vector3(2, 1, 2), radius: 0.5f);
        var hit = world.SweepSphere(in sphere, new Vector3(0, -3, 0), out var info);
        await Assert.That(hit).IsTrue();
        await Assert.That(info.Distance).IsLessThan(3.0);
    }

    [Test]
    public async Task SweepCapsule_HitsUsingEndpointSpheres()
    {
        var mesh = new TriangleMesh(
            [new Vector3(-5, 0, -5), new Vector3(5, 0, -5), new Vector3(-5, 0, 5), new Vector3(5, 0, 5)],
            [0, 1, 2, 1, 3, 2]);
        var world = new BvhStaticWorld(mesh);
        var cap = new Capsule(new Vector3(0, 2, 0), new Vector3(0, 4, 0), radius: 0.4f);
        var hit = world.SweepCapsule(in cap, new Vector3(0, -5, 0), out var info);
        await Assert.That(hit).IsTrue();
        await Assert.That(info.Distance).IsGreaterThan(0);
    }

    [Test]
    public async Task EmptyMesh_RaycastAndSweepReturnFalse()
    {
        var world = new BvhStaticWorld(new TriangleMesh([], []));
        await Assert.That(world.Raycast(new Ray(Vector3.Zero, Vector3.UnitY), 10, out _)).IsFalse();
        await Assert.That(world.SweepSphere(new Sphere(Vector3.Zero, 1f), Vector3.UnitX, out _)).IsFalse();
    }

    [Test]
    public async Task SweepSphere_GrazingOverlap_StillReportsHit()
    {
        var mesh = new TriangleMesh(
            [new Vector3(0, 0, 0), new Vector3(10, 0, 0), new Vector3(0, 0, 10)],
            [0, 1, 2]);
        var world = new BvhStaticWorld(mesh);
        var sphere = new Sphere(new Vector3(1, 0.4f, 1), 0.5f);
        var hit = world.SweepSphere(in sphere, new Vector3(0, -0.05f, 0), out _);
        await Assert.That(hit).IsTrue();
    }

    [Test]
    public async Task SweepSphere_DeepPenetration_ReturnsFalse()
    {
        var world = new BvhStaticWorld(new TriangleMesh(
            [new Vector3(-10, 0, -10), new Vector3(10, 0, -10), new Vector3(-10, 0, 10)],
            [0, 1, 2]));
        var sphere = new Sphere(new Vector3(0, -0.5f, 0), 0.5f);
        await Assert.That(world.SweepSphere(in sphere, new Vector3(0, -0.01f, 0), out _)).IsFalse();
    }
}
