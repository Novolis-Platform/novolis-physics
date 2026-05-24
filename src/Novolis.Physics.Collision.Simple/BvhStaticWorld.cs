using Novolis.Physics.Abstractions;
using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Collision.Simple;

/// <summary>Binary BVH over triangle indices; immutable after construction.</summary>
public sealed class BvhStaticWorld : IStaticWorld
{
    private readonly TriangleBvh _bvh;

    /// <summary>Builds a BVH over the static mesh triangles.</summary>
    /// <param name="mesh">Indexed triangle mesh.</param>
    public BvhStaticWorld(StaticTriangleMesh mesh) =>
        _bvh = TriangleBvhBuilder.Build(mesh.Vertices, mesh.TriangleIndices);

    /// <summary>Raycast operation.</summary>
    public bool Raycast(in Ray ray, double maxDistance, out HitInfo hit)
    {
        hit = default;
        if (_bvh.RootIndex < 0)
        {
            return false;
        }

        if (!_bvh.Raycast(in ray, (float)maxDistance, out var t, out var point, out var normal, out var tri))
        {
            return false;
        }

        hit = new HitInfo((double)t, point, normal, tri);
        return true;
    }

    /// <summary>Approximate swept sphere vs BVH static mesh.</summary>
    public bool SweepSphere(in Sphere sphere, Vector3 displacement, out HitInfo hit)
    {
        hit = default;
        var len = displacement.Length();
        if (len < 1e-30)
        {
            return false;
        }

        var dir = displacement / len;
        var ray = new Ray(sphere.Center, dir);
        if (!Raycast(in ray, len + sphere.Radius, out var raw))
        {
            return false;
        }

        var adjusted = raw.Distance - sphere.Radius;
        if (adjusted > len)
        {
            return false;
        }

        if (adjusted < 0)
        {
            if (adjusted < -sphere.Radius * 0.35)
            {
                return false;
            }

            adjusted = System.Math.Min(len * 1e-5, len * 0.5);
            if (adjusted < 1e-14)
            {
                adjusted = 1e-14;
            }
        }

        var point = ray.PointAt((float)adjusted);
        hit = new HitInfo(adjusted, point, raw.Normal, raw.PrimitiveIndex);
        return true;
    }

    /// <summary>Conservative capsule sweep using endpoint sphere sweeps.</summary>
    public bool SweepCapsule(in Capsule capsule, Vector3 displacement, out HitInfo hit)
    {
        var s0 = new Sphere(capsule.A, capsule.Radius);
        var s1 = new Sphere(capsule.B, capsule.Radius);
        var h0 = SweepSphere(in s0, displacement, out var hit0);
        var h1 = SweepSphere(in s1, displacement, out var hit1);
        if (h0 && h1)
        {
            hit = hit0.Distance <= hit1.Distance ? hit0 : hit1;
            return true;
        }

        if (h0)
        {
            hit = hit0;
            return true;
        }

        if (h1)
        {
            hit = hit1;
            return true;
        }

        hit = default;
        return false;
    }
}
