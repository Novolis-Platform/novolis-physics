using System.Numerics;

namespace Novolis.Physics.Collision.Simple;

/// <summary>Dynamic sphere state for static-world integration and pairwise contact resolution.</summary>
public sealed class SphereState
{
    public Vector3 Position;
    public Vector3 Velocity;
    public bool IsGrounded;
    public bool IsSleeping;

    public float Speed => Velocity.Length();

    public SphereState()
    {
    }

    public SphereState(Vector3 position, Vector3 velocity)
    {
        Position = position;
        Velocity = velocity;
    }
}
