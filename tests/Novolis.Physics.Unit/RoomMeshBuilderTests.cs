using Novolis.Physics.Collision.Simple;
using Novolis.Physics.Numerics;
using Novolis.Physics.TestSupport;
using TUnit.Core;

namespace Novolis.Physics.Unit;

[NotInParallel(NovolisPhysicsTestTrace.NotInParallelKey)]
public sealed class RoomMeshBuilderTests
{
    [Test]
    public async Task FromWallGrid_BuildsMeshWithTriangles()
    {
        Span<byte> cells = [0, 1, 0, 1];
        var world = RoomMeshBuilder.FromWallGrid(2, 2, cells);

        var ray = new Ray3d(new Vector3d(1.5, 1.0, -1.0), new Vector3d(0, 0, 1).Normalized());
        var hit = world.Raycast(in ray, maxDistance: 10.0, out var info);

        await Assert.That(hit).IsTrue();
        await Assert.That(info.Distance).IsGreaterThan(0);
    }
}
