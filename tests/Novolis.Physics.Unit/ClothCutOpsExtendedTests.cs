using System.Numerics;
using Novolis.Physics.Cloth;
using Novolis.Physics.Collision.Simple;
using Novolis.Physics.Joints;
using TUnit.Core;

namespace Novolis.Physics.Unit;

[NotInParallel(NovolisPhysicsTestTrace.NotInParallelKey)]
public sealed class ClothCutOpsExtendedTests
{
    [Test]
    public async Task SeverWhere_DropsInvalidJointIndices()
    {
        var joints = new List<DistanceJoint> { new(0, 99, 1f, 1f) };
        var spheres = new List<SphereState> { new(Vector3.Zero, Vector3.Zero) };
        var result = ClothCutOps.SeverWhere(joints, spheres, (_, _, _) => false);
        await Assert.That(result.SeveredJointCount).IsEqualTo(1);
        await Assert.That(joints.Count).IsEqualTo(0);
    }

    [Test]
    public async Task SeverWhere_CompactsWhenPredicateFalse()
    {
        var joints = new List<DistanceJoint>
        {
            new(0, 1, 1f, 1f),
            new(0, 1, 1f, 1f),
        };
        var spheres = new List<SphereState>
        {
            new(Vector3.Zero, Vector3.Zero),
            new(Vector3.UnitX, Vector3.Zero),
        };
        var result = ClothCutOps.SeverWhere(joints, spheres, (_, _, _) => false);
        await Assert.That(result.SeveredJointCount).IsEqualTo(0);
        await Assert.That(joints.Count).IsEqualTo(2);
    }

    [Test]
    public async Task SegmentSegmentDistanceSquared_CoversDegenerateCases()
    {
        var pointPoint = ClothCutOps.SegmentSegmentDistanceSquared(Vector3.Zero, Vector3.Zero, Vector3.UnitX, Vector3.UnitX);
        await Assert.That(pointPoint).IsEqualTo(1f).Within(0.01f);

        var parallel = ClothCutOps.SegmentSegmentDistanceSquared(
            Vector3.Zero, Vector3.UnitX,
            new Vector3(0f, 1f, 0f), new Vector3(1f, 1f, 0f));
        await Assert.That(parallel).IsEqualTo(1f).Within(0.01f);

        var skew = ClothCutOps.SegmentSegmentDistanceSquared(
            Vector3.Zero, Vector3.UnitX,
            Vector3.UnitY, Vector3.One);
        await Assert.That(skew).IsGreaterThanOrEqualTo(0f);
    }

    [Test]
    public async Task ApplyBlastImpulse_SkipsPinnedAndZeroSpeed()
    {
        var center = new Vector3(1f, 1f, 0f);
        var spheres = new List<SphereState>
        {
            new(center, Vector3.Zero),
            new(center + new Vector3(0.1f, 0f, 0f), Vector3.Zero),
        };
        var zero = ClothCutOps.ApplyBlastImpulse(spheres, new ClothBlast(center, 0.5f, 0f));
        await Assert.That(zero).IsEqualTo(0);

        spheres[1].Velocity = Vector3.Zero;
        var pinned = ClothCutOps.ApplyBlastImpulse(spheres, new ClothBlast(center, 0.5f, 5f), pinnedIndices: [0]);
        await Assert.That(pinned).IsEqualTo(1);
        await Assert.That(spheres[1].Velocity.Length()).IsGreaterThan(0f);
    }

    [Test]
    public async Task ApplyBlastImpulse_AtEpicenter_UsesUnitYFallback()
    {
        var center = new Vector3(2f, 0f, 0f);
        var spheres = new List<SphereState> { new(center, Vector3.Zero) };
        var affected = ClothCutOps.ApplyBlastImpulse(spheres, new ClothBlast(center, 0.5f, 3f));
        await Assert.That(affected).IsEqualTo(1);
        await Assert.That(spheres[0].Velocity.Y).IsGreaterThan(0f);
    }

    [Test]
    public async Task SeverWhere_NullArguments_Throw()
    {
        var joints = new List<DistanceJoint>();
        var spheres = new List<SphereState>();
        var actJ = () => ClothCutOps.SeverWhere(null!, spheres, (_, _, _) => false);
        var actS = () => ClothCutOps.SeverWhere(joints, null!, (_, _, _) => false);
        var actP = () => ClothCutOps.SeverWhere(joints, spheres, null!);
        await Assert.That(actJ).Throws<ArgumentNullException>();
        await Assert.That(actS).Throws<ArgumentNullException>();
        await Assert.That(actP).Throws<ArgumentNullException>();
    }

    [Test]
    public async Task SegmentSegmentDistanceSquared_ClampedParameterBranches()
    {
        var d = ClothCutOps.SegmentSegmentDistanceSquared(
            Vector3.Zero, new Vector3(10f, 0f, 0f),
            new Vector3(5f, 8f, 0f), new Vector3(5f, 9f, 0f));
        await Assert.That(d).IsGreaterThan(0f);

        var pointSeg = ClothCutOps.SegmentSegmentDistanceSquared(
            Vector3.Zero, Vector3.Zero,
            new Vector3(0f, 2f, 0f), new Vector3(1f, 2f, 0f));
        await Assert.That(pointSeg).IsEqualTo(4f).Within(0.01f);
    }
}
