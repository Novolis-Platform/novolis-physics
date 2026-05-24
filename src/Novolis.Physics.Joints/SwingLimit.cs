using System.Numerics;

namespace Novolis.Physics.Joints;

/// <summary>Ball-socket style limit: child direction from parent stays within a cone around rest.</summary>
public readonly struct SwingLimit
{
    /// <summary>Parent sphere index in the chain.</summary>
    public int ParentSphere { get; }

    /// <summary>Child sphere index in the chain.</summary>
    public int ChildSphere { get; }

    /// <summary>World rest when <see cref="FrameReferenceSphere"/> is -1.</summary>
    public Vector3 RestDirection { get; }

    /// <summary>Maximum angular deviation from rest (radians).</summary>
    public float MaxRadians { get; }

    /// <summary>Constraint stiffness in 0..1.</summary>
    public float Stiffness { get; }

    /// <summary>When &gt;= 0, <see cref="RestDirectionLocal"/> is resolved each solve via <see cref="BoneFrame"/>.</summary>
    public int FrameReferenceSphere { get; }

    /// <summary>Rest direction in the reference bone frame.</summary>
    public Vector3 RestDirectionLocal { get; }

    /// <summary>Creates a swing cone limit with optional local-frame rest direction.</summary>
    public SwingLimit(
        int parentSphere,
        int childSphere,
        Vector3 restDirection,
        float maxRadians,
        float stiffness = 1f,
        int frameReferenceSphere = -1,
        Vector3 restDirectionLocal = default)
    {
        ParentSphere = parentSphere;
        ChildSphere = childSphere;
        RestDirection = restDirection;
        MaxRadians = maxRadians;
        Stiffness = stiffness;
        FrameReferenceSphere = frameReferenceSphere;
        RestDirectionLocal = restDirectionLocal;
    }

    /// <summary>Creates a swing limit defined in a reference bone frame.</summary>
    public static SwingLimit CreateLocal(
        int parentSphere,
        int childSphere,
        int frameReferenceSphere,
        Vector3 restDirectionLocal,
        float maxRadians,
        float stiffness = 1f) =>
        new(
            parentSphere,
            childSphere,
            Vector3.UnitY,
            maxRadians,
            stiffness,
            frameReferenceSphere,
            restDirectionLocal);
}
