using System.Numerics;
using Novolis.Physics.Collision.Simple;
using Novolis.Physics.Joints;

namespace Novolis.Physics.Cloth;

/// <summary>
/// Mesh-cutting for cloth distance-joint graphs. All cut kinds (blade, blast, custom)
/// go through the same sever path so explosion fragmentation can reuse sword logic.
/// </summary>
/// <remarks>
/// Architecture for advanced damage:
/// <list type="number">
/// <item><description>Topology edit = remove <see cref="DistanceJoint"/> entries (never invent a second constraint stack).</description></item>
/// <item><description>Sword / knife = <see cref="CutWithBlade"/> (segment–segment capture).</description></item>
/// <item><description>Explosion = <see cref="CutWithBlast"/> (+ optional <see cref="ApplyBlastImpulse"/>); call repeatedly or with multiple blasts for multi-fragment shatter.</description></item>
/// <item><description>Future islands / separate pieces = connected-component labeling over remaining joints (not required for a two-piece sword split).</description></item>
/// </list>
/// </remarks>
public static class ClothCutOps
{
    /// <summary>
    /// Removes joints for which <paramref name="shouldSever"/> returns true.
    /// Predicate receives the joint and the current endpoints.
    /// </summary>
    public static ClothCutResult SeverWhere(
        IList<DistanceJoint> joints,
        IList<SphereState> spheres,
        Func<DistanceJoint, Vector3, Vector3, bool> shouldSever)
    {
        ArgumentNullException.ThrowIfNull(joints);
        ArgumentNullException.ThrowIfNull(spheres);
        ArgumentNullException.ThrowIfNull(shouldSever);

        var write = 0;
        var severed = 0;
        for (var read = 0; read < joints.Count; read++)
        {
            var joint = joints[read];
            if ((uint)joint.SphereA >= (uint)spheres.Count || (uint)joint.SphereB >= (uint)spheres.Count)
            {
                severed++;
                continue;
            }

            var a = spheres[joint.SphereA].Position;
            var b = spheres[joint.SphereB].Position;
            if (shouldSever(joint, a, b))
            {
                severed++;
                continue;
            }

            if (write != read)
                joints[write] = joint;
            write++;
        }

        while (joints.Count > write)
            joints.RemoveAt(joints.Count - 1);

        return new ClothCutResult(severed, joints.Count);
    }

    /// <summary>Severs every joint whose particle-pair segment comes within the blade volume.</summary>
    public static ClothCutResult CutWithBlade(
        IList<DistanceJoint> joints,
        IList<SphereState> spheres,
        in ClothBlade blade)
    {
        var heel = blade.Heel;
        var tip = blade.Tip;
        var thickness = blade.HalfThickness;
        var thicknessSq = thickness * thickness;

        return SeverWhere(joints, spheres, (_, a, b) =>
            SegmentSegmentDistanceSquared(a, b, heel, tip) <= thicknessSq);
    }

    /// <summary>
    /// Severs joints whose midpoint lies inside the blast radius.
    /// Scales to explosions: invoke once for a grenade, or many overlapping blasts for shatter.
    /// </summary>
    public static ClothCutResult CutWithBlast(
        IList<DistanceJoint> joints,
        IList<SphereState> spheres,
        in ClothBlast blast)
    {
        var center = blast.Epicenter;
        var radiusSq = blast.Radius * blast.Radius;

        return SeverWhere(joints, spheres, (_, a, b) =>
        {
            var mid = (a + b) * 0.5f;
            return (mid - center).LengthSquared() <= radiusSq;
        });
    }

    /// <summary>
    /// Adds an outward velocity kick to free particles near the blast epicenter.
    /// Call after <see cref="CutWithBlast"/> for an explosion that both tears and flings fabric.
    /// </summary>
    public static int ApplyBlastImpulse(
        IList<SphereState> spheres,
        in ClothBlast blast,
        ReadOnlySpan<int> pinnedIndices = default)
    {
        ArgumentNullException.ThrowIfNull(spheres);
        if (blast.ImpulseSpeed <= 1e-6f)
            return 0;

        var affected = 0;
        var radius = blast.Radius;
        var radiusSq = radius * radius;
        for (var i = 0; i < spheres.Count; i++)
        {
            if (IsPinned(i, pinnedIndices))
                continue;

            var delta = spheres[i].Position - blast.Epicenter;
            var dSq = delta.LengthSquared();
            if (dSq > radiusSq)
                continue;

            Vector3 direction;
            float falloff;
            if (dSq < 1e-10f)
            {
                direction = Vector3.UnitY;
                falloff = 1f;
            }
            else
            {
                var dist = MathF.Sqrt(dSq);
                direction = delta / dist;
                falloff = 1f - dist / radius;
            }

            spheres[i].Velocity += direction * (blast.ImpulseSpeed * falloff);
            spheres[i].IsSleeping = false;
            affected++;
        }

        return affected;
    }

    /// <summary>Minimum distance between two finite segments, squared.</summary>
    public static float SegmentSegmentDistanceSquared(Vector3 p1, Vector3 q1, Vector3 p2, Vector3 q2)
    {
        const float eps = 1e-8f;
        var d1 = q1 - p1;
        var d2 = q2 - p2;
        var r = p1 - p2;
        var a = Vector3.Dot(d1, d1);
        var e = Vector3.Dot(d2, d2);
        var f = Vector3.Dot(d2, r);

        float s, t;
        if (a <= eps && e <= eps)
            return r.LengthSquared();

        if (a <= eps)
        {
            s = 0f;
            t = System.Math.Clamp(f / e, 0f, 1f);
        }
        else
        {
            var c = Vector3.Dot(d1, r);
            if (e <= eps)
            {
                t = 0f;
                s = System.Math.Clamp(-c / a, 0f, 1f);
            }
            else
            {
                var b = Vector3.Dot(d1, d2);
                var denom = a * e - b * b;
                s = denom != 0f ? System.Math.Clamp((b * f - c * e) / denom, 0f, 1f) : 0f;
                t = (b * s + f) / e;
                if (t < 0f)
                {
                    t = 0f;
                    s = System.Math.Clamp(-c / a, 0f, 1f);
                }
                else if (t > 1f)
                {
                    t = 1f;
                    s = System.Math.Clamp((b - c) / a, 0f, 1f);
                }
            }
        }

        var c1 = p1 + d1 * s;
        var c2 = p2 + d2 * t;
        return (c1 - c2).LengthSquared();
    }

    private static bool IsPinned(int index, ReadOnlySpan<int> pinned)
    {
        foreach (var p in pinned)
        {
            if (p == index)
                return true;
        }

        return false;
    }
}
