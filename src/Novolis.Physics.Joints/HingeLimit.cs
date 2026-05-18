using System.Numerics;

namespace Novolis.Physics.Joints;

/// <summary>Hinge limit in the plane perpendicular to hinge axis (signed angle from rest).</summary>
public readonly struct HingeLimit
{
    public int ParentSphere { get; }
    public int ChildSphere { get; }
    /// <summary>World axis when <see cref="FrameReferenceSphere"/> is -1.</summary>
    public Vector3 HingeAxis { get; }
    /// <summary>World rest when <see cref="FrameReferenceSphere"/> is -1.</summary>
    public Vector3 RestDirection { get; }
    public float MinRadians { get; }
    public float MaxRadians { get; }
    public float Stiffness { get; }
    public int FrameReferenceSphere { get; }
    public Vector3 RestDirectionLocal { get; }
    public Vector3 HingeAxisLocal { get; }

    public HingeLimit(
        int parentSphere,
        int childSphere,
        Vector3 hingeAxis,
        Vector3 restDirection,
        float minRadians,
        float maxRadians,
        float stiffness = 1f,
        int frameReferenceSphere = -1,
        Vector3 restDirectionLocal = default,
        Vector3 hingeAxisLocal = default)
    {
        ParentSphere = parentSphere;
        ChildSphere = childSphere;
        HingeAxis = hingeAxis;
        RestDirection = restDirection;
        MinRadians = minRadians;
        MaxRadians = maxRadians;
        Stiffness = stiffness;
        FrameReferenceSphere = frameReferenceSphere;
        RestDirectionLocal = restDirectionLocal;
        HingeAxisLocal = hingeAxisLocal;
    }

    public static HingeLimit CreateLocal(
        int parentSphere,
        int childSphere,
        int frameReferenceSphere,
        Vector3 hingeAxisLocal,
        Vector3 restDirectionLocal,
        float minRadians,
        float maxRadians,
        float stiffness = 1f) =>
        new(
            parentSphere,
            childSphere,
            Vector3.UnitX,
            Vector3.UnitY,
            minRadians,
            maxRadians,
            stiffness,
            frameReferenceSphere,
            restDirectionLocal,
            hingeAxisLocal);
}
