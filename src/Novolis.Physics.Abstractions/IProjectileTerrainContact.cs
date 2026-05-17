using System.Numerics;

namespace Novolis.Physics.Abstractions;

/// <summary>Heightfield and range-box queries for ballistic terrain contact.</summary>
public interface IProjectileTerrainContact
{
    bool IsInside(float x, float z);

    bool TryHeightfieldContact(Vector3 position, float radius);

    Vector3 ProjectOntoSurface(Vector3 position, float surfaceEpsilon = 0.05f);

    bool TrySegmentLeavesRange(Vector3 from, Vector3 to, out Vector3 hitPoint, out float fractionAlongSegment);
}
