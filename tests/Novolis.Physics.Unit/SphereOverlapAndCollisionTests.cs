using System.Numerics;
using Novolis.Physics.Abstractions;
using Novolis.Physics.Ballistics;
using Novolis.Physics.Collision.Simple;
using Novolis.Physics.Joints;
using Novolis.Physics.TestSupport;
using Novolis.Math.Geometry;
using TUnit.Core;

namespace Novolis.Physics.Unit;

[NotInParallel(NovolisPhysicsTestTrace.NotInParallelKey)]
public sealed class SphereOverlapAndCollisionTests
{
    [Test]
    public async Task Separate_PushesOverlappingSpheresApart()
    {
        var a = new Vector3(0f, 0f, 0f);
        var b = new Vector3(0.5f, 0f, 0f);
        var moved = SphereOverlapResolution.Separate(ref a, ref b, radius: 0.5f);
        await Assert.That(moved).IsTrue();
        var dist = Vector3.Distance(a, b);
        await Assert.That(dist).IsGreaterThanOrEqualTo(0.99f);
    }

    [Test]
    public async Task Separate_CoincidentCenters_UsesFallbackNormal()
    {
        var a = Vector3.Zero;
        var b = Vector3.Zero;
        var moved = SphereOverlapResolution.Separate(ref a, ref b, radius: 0.5f);
        await Assert.That(moved).IsTrue();
        await Assert.That(a.X).IsLessThan(0f);
        await Assert.That(b.X).IsGreaterThan(0f);
    }

    [Test]
    public async Task SeparateWithImpulse_AppliesRestitution()
    {
        var a = new Vector3(0f, 0f, 0f);
        var b = new Vector3(0.5f, 0f, 0f);
        var va = new Vector3(1f, 0f, 0f);
        var vb = new Vector3(-1f, 0f, 0f);
        var hit = SphereOverlapResolution.SeparateWithImpulse(ref a, ref b, ref va, ref vb, radius: 0.5f, restitution: 0.5f);
        await Assert.That(hit).IsTrue();
        await Assert.That(va.X).IsLessThan(1f);
        await Assert.That(vb.X).IsGreaterThan(-1f);
    }

    [Test]
    public async Task RagdollBodyCollision_ResolvesOverlapAndBuildsSkipPairs()
    {
        var spheres = new List<SphereState>
        {
            new(new Vector3(0f, 0f, 0f), Vector3.Zero),
            new(new Vector3(0.2f, 0f, 0f), Vector3.Zero),
            new(new Vector3(2f, 0f, 0f), Vector3.Zero),
        };
        var fixes = RagdollBodyCollision.ResolveOverlaps(spheres, radius: 0.5f, iterations: 4);
        await Assert.That(fixes).IsGreaterThan(0);
        await Assert.That(Vector3.Distance(spheres[0].Position, spheres[1].Position)).IsGreaterThanOrEqualTo(0.99f);

        DistanceJoint[] joints = [new(0, 1, 1f), new(1, 2, 1f)];
        var skip = RagdollBodyCollision.BuildAdjacentSkipPairs(joints);
        await Assert.That(skip.Length).IsEqualTo(2);
        var skippedFixes = RagdollBodyCollision.ResolveOverlaps(spheres, 0.5f, 4, 1.02f, skip);
        await Assert.That(skippedFixes).IsGreaterThanOrEqualTo(0);
    }

    [Test]
    public async Task EmptyStaticWorld_NeverHits()
    {
        var world = new EmptyStaticWorld();
        var ray = new Ray(Vector3.Zero, Vector3.UnitX);
        await Assert.That(world.Raycast(ray, 100, out _)).IsFalse();
        await Assert.That(world.SweepSphere(new Sphere(Vector3.Zero, 1f), Vector3.UnitX, out _)).IsFalse();
        await Assert.That(world.SweepCapsule(new Capsule(Vector3.Zero, Vector3.UnitY, 0.5f), Vector3.UnitX, out _)).IsFalse();
    }

    [Test]
    public async Task StandardAtmosphere_DensityAndPressure()
    {
        var rho = StandardAtmosphere.DensityKgPerM3(101325, 288.15, relativeHumidity01: 0.5);
        await Assert.That(rho).IsGreaterThan(1.0);
        await Assert.That(StandardAtmosphere.DensityKgPerM3(101325, 100, 0.5)).IsEqualTo(0);

        var p0 = StandardAtmosphere.PressureAtAltitude(101325, altitudeMeters: 0, referenceTemperatureKelvin: 288.15);
        var pHigh = StandardAtmosphere.PressureAtAltitude(101325, altitudeMeters: 1000, referenceTemperatureKelvin: 288.15);
        await Assert.That(p0).IsEqualTo(101325).Within(1);
        await Assert.That(pHigh).IsLessThan(p0);

        var iso = StandardAtmosphere.PressureAtAltitude(101325, 500, 288.15, lapseRateKelvinPerMeter: 0);
        await Assert.That(iso).IsLessThan(p0).And.IsGreaterThan(90000);
    }

    [Test]
    public async Task HeightfieldContact_DetectsGroundAndProjects()
    {
        IHeightSampler sampler = new FlatHeight(0f);
        var range = new AxisAlignedRangeBox(100f);
        var contact = HeightfieldContact.TryContact(sampler, range, new Vector3(10f, 0.05f, 10f), radius: 0.1f);
        await Assert.That(contact).IsTrue();
        await Assert.That(HeightfieldContact.TryContact(sampler, range, new Vector3(200f, 0f, 0f), 0.1f)).IsFalse();

        var projected = HeightfieldContact.ProjectOntoSurface(sampler, range, new Vector3(150f, 5f, -5f));
        await Assert.That(projected.X).IsEqualTo(100f).Within(0.001f);
        await Assert.That(projected.Z).IsEqualTo(0f).Within(0.001f);
        await Assert.That(projected.Y).IsGreaterThan(0f);
    }

    private sealed class FlatHeight(float y) : IHeightSampler
    {
        public float SampleHeight(float x, float z) => y;
    }
}
