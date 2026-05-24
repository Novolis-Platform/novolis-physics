using System.Numerics;

namespace Novolis.Physics.Joints;

/// <summary>Orthonormal bone basis: right, up (parent→reference), forward.</summary>
public readonly struct BoneFrame
{
    /// <summary>Right axis of the bone frame.</summary>
    public Vector3 Right { get; }

    /// <summary>Up axis (parent toward reference).</summary>
    public Vector3 Up { get; }

    /// <summary>Forward axis completing the right-handed basis.</summary>
    public Vector3 Forward { get; }

    /// <summary>Initializes a bone frame from orthonormal axes.</summary>
    public BoneFrame(Vector3 right, Vector3 up, Vector3 forward)
    {
        Right = right;
        Up = up;
        Forward = forward;
    }

    /// <summary>Builds a frame from parent and reference positions when they are separated.</summary>
    public static bool TryCreate(Vector3 parentPosition, Vector3 referencePosition, out BoneFrame frame)
    {
        var up = referencePosition - parentPosition;
        var upLenSq = up.LengthSquared();
        if (upLenSq < 1e-10f)
        {
            frame = default;
            return false;
        }

        up /= MathF.Sqrt(upLenSq);
        var worldForward = Vector3.UnitZ;
        var right = Vector3.Cross(up, worldForward);
        if (right.LengthSquared() < 1e-8f)
            right = Vector3.Cross(up, Vector3.UnitX);
        right = Vector3.Normalize(right);
        var forward = Vector3.Normalize(Vector3.Cross(right, up));
        frame = new BoneFrame(right, up, forward);
        return true;
    }

    /// <summary>Transforms a local vector to world space.</summary>
    public Vector3 LocalToWorld(Vector3 local) =>
        Right * local.X + Up * local.Y + Forward * local.Z;

    /// <summary>Transforms a world vector to local space.</summary>
    public Vector3 WorldToLocal(Vector3 world) =>
        new(Vector3.Dot(world, Right), Vector3.Dot(world, Up), Vector3.Dot(world, Forward));
}
