namespace Novolis.Physics.Collision.Simple;

/// <summary>Structure-of-arrays layout for many spheres (SIMD-friendly broad-phase).</summary>
public sealed class SphereSoA
{
    /// <summary>World X positions (meters).</summary>
    public float[] PosX = [];

    /// <summary>World Y positions (meters).</summary>
    public float[] PosY = [];

    /// <summary>World Z positions (meters).</summary>
    public float[] PosZ = [];

    /// <summary>World X velocities (m/s).</summary>
    public float[] VelX = [];

    /// <summary>World Y velocities (m/s).</summary>
    public float[] VelY = [];

    /// <summary>World Z velocities (m/s).</summary>
    public float[] VelZ = [];

    /// <summary>Per-sphere sleeping flags.</summary>
    public bool[] Sleeping = [];

    /// <summary>Number of active spheres in the arrays.</summary>
    public int Count { get; private set; }

    /// <summary>Grows internal buffers to hold at least <paramref name="count"/> spheres.</summary>
    public void Resize(int count)
    {
        Count = count;
        if (PosX.Length >= count)
            return;

        PosX = new float[count];
        PosY = new float[count];
        PosZ = new float[count];
        VelX = new float[count];
        VelY = new float[count];
        VelZ = new float[count];
        Sleeping = new bool[count];
    }

    /// <summary>Copies sphere state from <paramref name="spheres"/> into SoA buffers.</summary>
    public void SyncFrom(IList<SphereState> spheres)
    {
        Resize(spheres.Count);
        for (var i = 0; i < spheres.Count; i++)
        {
            var s = spheres[i];
            PosX[i] = s.Position.X;
            PosY[i] = s.Position.Y;
            PosZ[i] = s.Position.Z;
            VelX[i] = s.Velocity.X;
            VelY[i] = s.Velocity.Y;
            VelZ[i] = s.Velocity.Z;
            Sleeping[i] = s.IsSleeping;
        }
    }

    /// <summary>Writes SoA velocities and positions back into <paramref name="spheres"/>.</summary>
    public void SyncTo(IList<SphereState> spheres)
    {
        for (var i = 0; i < Count; i++)
        {
            var s = spheres[i];
            s.Position = new System.Numerics.Vector3(PosX[i], PosY[i], PosZ[i]);
            s.Velocity = new System.Numerics.Vector3(VelX[i], VelY[i], VelZ[i]);
        }
    }
}
