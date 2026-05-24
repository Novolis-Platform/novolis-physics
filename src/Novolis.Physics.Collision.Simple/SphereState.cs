using System.Numerics;

namespace Novolis.Physics.Collision.Simple;

/// <summary>Dynamic sphere state for static-world integration and pairwise contact resolution.</summary>
public sealed class SphereState
{
    /// <summary>Position.</summary>
    public Vector3 Position;
    /// <summary>IsGrounded.</summary>
    public Vector3 Velocity;
    /// <summary>IsGrounded.</summary>
    public bool IsGrounded;
    /// <summary>Speed.</summary>
    public bool IsSleeping;
/// <summary>SphereState operation.</summary>

    public float Speed => Velocity.Length();

    /// <summary>SphereState operation.</summary>
    public SphereState()
    {
    }
/// <summary>SphereState operation.</summary>

    public SphereState(Vector3 position, Vector3 velocity)
    {
        Position = position;
        Velocity = velocity;
    }
}
