using System.Numerics;

namespace Novolis.Physics.Abstractions;

/// <summary>Heightfield and range-box queries for ballistic terrain contact.</summary>
public interface IProjectileTerrainContact
{
    /// <summary>Returns whether world XZ lies inside the playable range.</summary>
    bool IsInside(float x, float z);

    /// <summary>Tests sphere–heightfield penetration at <paramref name="position"/> with <paramref name="radius"/>.</summary>
    bool TryHeightfieldContact(Vector3 position, float radius);

    /// <summary>Projects <paramref name="position"/> onto the terrain surface within <paramref name="surfaceEpsilon"/>.</summary>
    Vector3 ProjectOntoSurface(Vector3 position, float surfaceEpsilon = 0.05f);

    /// <summary>First exit of a segment from the terrain range or heightfield bounds.</summary>
    bool TrySegmentLeavesRange(Vector3 from, Vector3 to, out Vector3 hitPoint, out float fractionAlongSegment);
}
