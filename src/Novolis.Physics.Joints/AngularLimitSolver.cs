using System.Numerics;
using Novolis.Physics.Collision.Simple;

namespace Novolis.Physics.Joints;

/// <summary>Position projections for swing cones and hinge arcs on sphere chains.</summary>
public static class AngularLimitSolver
{
    /// <summary>Iteratively applies swing and hinge limits to sphere positions.</summary>
    public static int Solve(
        ReadOnlySpan<SwingLimit> swingLimits,
        ReadOnlySpan<HingeLimit> hingeLimits,
        IList<SphereState> spheres,
        int iterations = 4)
    {
        if (spheres.Count == 0 || iterations <= 0)
            return 0;

        var corrections = 0;
        for (var iter = 0; iter < iterations; iter++)
        {
            foreach (var limit in swingLimits)
                corrections += SolveSwing(limit, spheres);

            foreach (var limit in hingeLimits)
                corrections += SolveHinge(limit, spheres);
        }

        return corrections;
    }

    /// <summary>Enforces one swing cone limit between parent and child spheres.</summary>
    public static int SolveSwing(SwingLimit limit, IList<SphereState> spheres)
    {
        if ((uint)limit.ParentSphere >= (uint)spheres.Count || (uint)limit.ChildSphere >= (uint)spheres.Count)
            return 0;

        var parent = spheres[limit.ParentSphere];
        var child = spheres[limit.ChildSphere];
        var delta = child.Position - parent.Position;
        var lenSq = delta.LengthSquared();
        if (lenSq < 1e-10f)
            return 0;

        var len = MathF.Sqrt(lenSq);
        var bone = delta / len;
        var rest = ResolveSwingRest(limit, spheres);
        var maxRadians = MathF.Max(limit.MaxRadians, 0f);
        var cosMax = MathF.Cos(maxRadians);
        var dot = Vector3.Dot(bone, rest);
        if (dot >= cosMax)
            return 0;

        var clampedBone = ClampToCone(bone, rest, cosMax);
        return ApplyBoneCorrection(ref parent, ref child, clampedBone, len, limit.Stiffness, spheres, limit.ParentSphere, limit.ChildSphere);
    }

    /// <summary>Enforces one hinge arc limit between parent and child spheres.</summary>
    public static int SolveHinge(HingeLimit limit, IList<SphereState> spheres)
    {
        if ((uint)limit.ParentSphere >= (uint)spheres.Count || (uint)limit.ChildSphere >= (uint)spheres.Count)
            return 0;

        if (!TryResolveHingeAxisAndRest(limit, spheres, out var axis, out var rest))
            return 0;

        var parent = spheres[limit.ParentSphere];
        var child = spheres[limit.ChildSphere];
        var delta = child.Position - parent.Position;
        var lenSq = delta.LengthSquared();
        if (lenSq < 1e-10f)
            return 0;

        var len = MathF.Sqrt(lenSq);
        var bone = delta / len;
        var restPlane = ProjectOnPlane(rest, axis);
        if (restPlane.LengthSquared() < 1e-10f)
            return 0;

        restPlane = Vector3.Normalize(restPlane);

        var bonePlane = ProjectOnPlane(bone, axis);
        if (bonePlane.LengthSquared() < 1e-10f)
            bonePlane = restPlane;
        else
            bonePlane = Vector3.Normalize(bonePlane);

        var angle = SignedAngle(restPlane, bonePlane, axis);
        var clamped = System.Math.Clamp(angle, limit.MinRadians, limit.MaxRadians);
        if (MathF.Abs(angle - clamped) < 1e-5f)
            return 0;

        var clampedBone = RotateAroundAxis(restPlane, axis, clamped);
        return ApplyBoneCorrection(ref parent, ref child, clampedBone, len, limit.Stiffness, spheres, limit.ParentSphere, limit.ChildSphere);
    }

    private static Vector3 ResolveSwingRest(SwingLimit limit, IList<SphereState> spheres)
    {
        if (limit.FrameReferenceSphere < 0)
            return Vector3.Normalize(limit.RestDirection);

        if ((uint)limit.FrameReferenceSphere >= (uint)spheres.Count)
            return Vector3.Normalize(limit.RestDirection);

        if (!BoneFrame.TryCreate(
                spheres[limit.ParentSphere].Position,
                spheres[limit.FrameReferenceSphere].Position,
                out var frame))
            return Vector3.Normalize(limit.RestDirection);

        var world = frame.LocalToWorld(limit.RestDirectionLocal);
        var lenSq = world.LengthSquared();
        return lenSq < 1e-10f ? Vector3.Normalize(limit.RestDirection) : world / MathF.Sqrt(lenSq);
    }

    private static bool TryResolveHingeAxisAndRest(
        HingeLimit limit,
        IList<SphereState> spheres,
        out Vector3 axis,
        out Vector3 rest)
    {
        if (limit.FrameReferenceSphere < 0)
        {
            axis = limit.HingeAxis;
            rest = limit.RestDirection;
        }
        else
        {
            if ((uint)limit.FrameReferenceSphere >= (uint)spheres.Count)
            {
                axis = limit.HingeAxis;
                rest = limit.RestDirection;
            }
            else if (!BoneFrame.TryCreate(
                         spheres[limit.ParentSphere].Position,
                         spheres[limit.FrameReferenceSphere].Position,
                         out var frame))
            {
                axis = limit.HingeAxis;
                rest = limit.RestDirection;
            }
            else
            {
                axis = frame.LocalToWorld(limit.HingeAxisLocal);
                rest = frame.LocalToWorld(limit.RestDirectionLocal);
            }
        }

        var axisLenSq = axis.LengthSquared();
        if (axisLenSq < 1e-10f)
        {
            axis = Vector3.UnitX;
            rest = Vector3.UnitY;
            return false;
        }

        axis /= MathF.Sqrt(axisLenSq);
        var restLenSq = rest.LengthSquared();
        if (restLenSq < 1e-10f)
        {
            rest = Vector3.UnitY;
            return false;
        }

        rest /= MathF.Sqrt(restLenSq);
        return true;
    }

    private static int ApplyBoneCorrection(
        ref SphereState parent,
        ref SphereState child,
        Vector3 targetBone,
        float length,
        float stiffness,
        IList<SphereState> spheres,
        int parentIndex,
        int childIndex)
    {
        var w = System.Math.Clamp(stiffness, 0f, 1f);
        if (w <= 0f)
            return 0;

        var targetChild = parent.Position + targetBone * length;
        var correction = (targetChild - child.Position) * w;
        child.Position += correction;
        parent.Position -= correction * 0.35f;
        spheres[parentIndex] = parent;
        spheres[childIndex] = child;
        return 1;
    }

    private static Vector3 ClampToCone(Vector3 bone, Vector3 rest, float cosMax)
    {
        var axis = Vector3.Cross(bone, rest);
        var axisLenSq = axis.LengthSquared();
        if (axisLenSq < 1e-10f)
            return rest;

        axis /= MathF.Sqrt(axisLenSq);
        var angle = MathF.Acos(System.Math.Clamp(Vector3.Dot(bone, rest), -1f, 1f));
        var maxAngle = MathF.Acos(System.Math.Clamp(cosMax, -1f, 1f));
        return RotateAroundAxis(bone, axis, maxAngle - angle);
    }

    private static Vector3 ProjectOnPlane(Vector3 v, Vector3 planeNormal) =>
        v - planeNormal * Vector3.Dot(v, planeNormal);

    private static float SignedAngle(Vector3 from, Vector3 to, Vector3 axis)
    {
        var sin = Vector3.Dot(Vector3.Cross(from, to), axis);
        var cos = Vector3.Dot(from, to);
        return MathF.Atan2(sin, cos);
    }

    private static Vector3 RotateAroundAxis(Vector3 v, Vector3 axis, float angle)
    {
        var c = MathF.Cos(angle);
        var s = MathF.Sin(angle);
        return v * c + Vector3.Cross(axis, v) * s + axis * Vector3.Dot(axis, v) * (1f - c);
    }
}
