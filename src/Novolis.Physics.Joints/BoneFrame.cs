using System.Numerics;

namespace Novolis.Physics.Joints;

/// <summary>Orthonormal bone basis: right, up (parent→reference), forward.</summary>
public readonly struct BoneFrame
{
    public Vector3 Right { get; }
    public Vector3 Up { get; }
    public Vector3 Forward { get; }

    public BoneFrame(Vector3 right, Vector3 up, Vector3 forward)
    {
        Right = right;
        Up = up;
        Forward = forward;
    }

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

    public Vector3 LocalToWorld(Vector3 local) =>
        Right * local.X + Up * local.Y + Forward * local.Z;

    public Vector3 WorldToLocal(Vector3 world) =>
        new(Vector3.Dot(world, Right), Vector3.Dot(world, Up), Vector3.Dot(world, Forward));
}
