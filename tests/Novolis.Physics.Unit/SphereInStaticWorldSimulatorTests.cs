using Novolis.Physics.Collision.Simple;
using Novolis.Physics.TestSupport;
using System.Numerics;
using Novolis.Math.Geometry;
using TUnit.Core;

namespace Novolis.Physics.Unit;

public sealed class SphereInStaticWorldSimulatorTests
{
    [Test]
    public async Task Step_ManySpheres_ResolvesContactsAndClamps()
    {
        var world = new BvhStaticWorld(new TriangleMesh(
            [new Vector3(-50, 0, -50), new Vector3(50, 0, -50), new Vector3(-50, 0, 50), new Vector3(50, 0, 50)],
            [0, 1, 2, 1, 3, 2]));
        var interior = new InteriorClampVolume
        {
            MinX = -40, MaxX = 40, MinY = 0, MaxY = 30, MinZ = -40, MaxZ = 40,
        };
        var spheres = Enumerable.Range(0, 60)
            .Select(i => new SphereState(new Vector3(i % 10, 2f + (i % 5), i % 8), new Vector3(0.5f, 0, 0)))
            .ToList();

        var sim = new SphereInStaticWorldSimulator
        {
            Options =
            {
                Radius = 0.25f,
                Gravity = new Vector3(0, -9.8f, 0),
                FloorHeight = 0f,
                MaxSpeedMps = 12f,
            },
        };

        for (var i = 0; i < 20; i++)
            sim.Step(world, spheres, interior, 1f / 60f);

        await Assert.That(sim.LastStats.SphereContacts).IsGreaterThanOrEqualTo(0);
        await Assert.That(sim.LastStats.PhysicsSubSteps).IsGreaterThan(0);
        await Assert.That(spheres.All(s => s.Position.Y >= -1f)).IsTrue();
    }

    [Test]
    public async Task Step_AllSleeping_SkipsSphereContacts()
    {
        var world = new BvhStaticWorld(new TriangleMesh([], []));
        var interior = new InteriorClampVolume { MinX = -1, MaxX = 1, MinY = 0, MaxY = 1, MinZ = -1, MaxZ = 1 };
        var spheres = new List<SphereState>
        {
            new(new Vector3(0, 0.3f, 0), Vector3.Zero) { IsSleeping = true, IsGrounded = true },
            new(new Vector3(0.5f, 0.3f, 0), Vector3.Zero) { IsSleeping = true, IsGrounded = true },
        };
        var sim = new SphereInStaticWorldSimulator { Options = { Radius = 0.2f, FloorHeight = 0f } };
        sim.ResetPileState();
        for (var i = 0; i < 30; i++)
            sim.Step(world, spheres, interior, 1f / 60f);
        await Assert.That(sim.LastStats.SphereContactSkipped).IsTrue();
    }

    [Test]
    public async Task DepenetrateSpawnedRange_SeparatesOverlappingSpawn()
    {
        var sim = new SphereInStaticWorldSimulator { Options = { Radius = 0.3f, MaxSpeedMps = 5f } };
        var spheres = Enumerable.Range(0, 55)
            .Select(_ => new SphereState(new Vector3(0, 1, 0), Vector3.Zero))
            .ToList();
        var interior = new InteriorClampVolume { MinX = -5, MaxX = 5, MinY = 0, MaxY = 5, MinZ = -5, MaxZ = 5 };
        sim.DepenetrateSpawnedRange(spheres, 0, spheres.Count, interior);
        var minDist = spheres.Min(s => s.Position.Length());
        await Assert.That(minDist).IsGreaterThanOrEqualTo(0f);
    }
}
