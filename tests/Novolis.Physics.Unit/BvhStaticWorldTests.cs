using Novolis.Physics.TestSupport;
using Novolis.Physics.Collision.Simple;
using System.Numerics;
using Novolis.Math.Geometry;
using TUnit.Core;

namespace Novolis.Physics.Unit;

[NotInParallel(NovolisPhysicsTestTrace.NotInParallelKey)]
public sealed class BvhStaticWorldTests
{
    [Test]
    public async Task BvhStaticWorld_Raycast_HitsGroundTriangle()
    {
        var verts = new[]
        {
            PhysicsTestVectors.V(0, 0, 0),
            PhysicsTestVectors.V(10, 0, 0),
            PhysicsTestVectors.V(0, 0, 10),
        };
        var indices = new[] { 0, 1, 2 };
        var mesh = new TriangleMesh(verts, indices);
        var world = new BvhStaticWorld(mesh);
        var ray = new Ray(PhysicsTestVectors.V(1, 2, 1), PhysicsTestVectors.V(0, -1, 0).Normalized());
        var hit = world.Raycast(in ray, maxDistance: 10, out var info);

        var o = NovolisPhysicsTestTrace.Out;
        o.Section("BvhStaticWorld raycast (compact)");

        o.Results("BvhStaticWorld raycast - mesh and outcome");
        o.Table(
            new[]
            {
                new TriangleVertexRow("V0", verts[0].X, verts[0].Y, verts[0].Z),
                new TriangleVertexRow("V1", verts[1].X, verts[1].Y, verts[1].Z),
                new TriangleVertexRow("V2", verts[2].X, verts[2].Y, verts[2].Z),
            },
            new TableOptions { RightAlignNumericColumns = true },
            caption: "Ground triangle (m)");
        o.Table(
            new[]
            {
                new RaycastOutcomeRow(
                    "ray",
                    ray.Origin.X,
                    ray.Origin.Y,
                    ray.Origin.Z,
                    ray.Direction.X,
                    ray.Direction.Y,
                    ray.Direction.Z,
                    MaxDist: 10,
                    Hit: null,
                    Distance: null,
                    PrimitiveIndex: null),
                new RaycastOutcomeRow(
                    "result",
                    ray.Origin.X,
                    ray.Origin.Y,
                    ray.Origin.Z,
                    ray.Direction.X,
                    ray.Direction.Y,
                    ray.Direction.Z,
                    MaxDist: null,
                    Hit: hit,
                    Distance: info.Distance,
                    PrimitiveIndex: info.PrimitiveIndex),
            },
            new TableOptions { RightAlignNumericColumns = true },
            caption: "Ray (origin m, unit dir, maxDistance) then hit");

        await Assert.That(hit).IsTrue();
        await Assert.That(info.Distance).IsGreaterThan((float)(1.9)).And.IsLessThan((float)(2.1));
    }

    private sealed record TriangleVertexRow(string Id, double X, double Y, double Z);

    private sealed record RaycastOutcomeRow(
        string Stage,
        double Ox,
        double Oy,
        double Oz,
        double Dx,
        double Dy,
        double Dz,
        double? MaxDist,
        bool? Hit,
        double? Distance,
        int? PrimitiveIndex);
}
