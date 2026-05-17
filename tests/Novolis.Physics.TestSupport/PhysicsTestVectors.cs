using System.Numerics;

namespace Novolis.Physics.TestSupport;

/// <summary>Test helpers for BCL <see cref="Vector3"/> from double literals.</summary>
public static class PhysicsTestVectors
{
    public static Vector3 V(double x, double y, double z) => new((float)x, (float)y, (float)z);
}
