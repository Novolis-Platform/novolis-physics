using System.Numerics;

namespace Novolis.Physics.Joints;

/// <summary>Ball-socket style limit: child direction from parent stays within a cone around rest.</summary>
public readonly struct SwingLimit
{
    public int ParentSphere { get; }
    public int ChildSphere { get; }
    /// <summary>World rest when <see cref="FrameReferenceSphere"/> is -1.</summary>
    public Vector3 RestDirection { get; }
    public float MaxRadians { get; }
    public float Stiffness { get; }
    /// <summary>When &gt;= 0, <see cref="RestDirectionLocal"/> is resolved each solve via <see cref="BoneFrame"/>.</summary>
    public int FrameReferenceSphere { get; }
    public Vector3 RestDirectionLocal { get; }

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
