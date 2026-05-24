using System.Numerics;

namespace Novolis.Physics.Joints;

/// <summary>Hinge limit in the plane perpendicular to hinge axis (signed angle from rest).</summary>
public readonly struct HingeLimit
{
    /// <summary>Parent sphere index in the chain.</summary>
    public int ParentSphere { get; }

    /// <summary>Child sphere index in the chain.</summary>
    public int ChildSphere { get; }

    /// <summary>World axis when <see cref="FrameReferenceSphere"/> is -1.</summary>
    public Vector3 HingeAxis { get; }

    /// <summary>World rest when <see cref="FrameReferenceSphere"/> is -1.</summary>
    public Vector3 RestDirection { get; }

    /// <summary>Minimum signed hinge angle (radians).</summary>
    public float MinRadians { get; }

    /// <summary>Maximum signed hinge angle (radians).</summary>
    public float MaxRadians { get; }

    /// <summary>Constraint stiffness in 0..1.</summary>
    public float Stiffness { get; }

    /// <summary>Bone frame reference sphere index, or -1 for world axes.</summary>
    public int FrameReferenceSphere { get; }

    /// <summary>Rest direction in the reference bone frame.</summary>
    public Vector3 RestDirectionLocal { get; }

    /// <summary>Hinge axis in the reference bone frame.</summary>
    public Vector3 HingeAxisLocal { get; }

    /// <summary>Creates a hinge limit with optional local-frame parameters.</summary>
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

    /// <summary>Creates a hinge limit defined in a reference bone frame.</summary>
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
