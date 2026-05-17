namespace Novolis.Physics.Collision.Simple;

/// <summary>Structure-of-arrays layout for many spheres (SIMD-friendly broad-phase).</summary>
public sealed class SphereSoA
{
    public float[] PosX = [];
    public float[] PosY = [];
    public float[] PosZ = [];
    public float[] VelX = [];
    public float[] VelY = [];
    public float[] VelZ = [];
    public bool[] Sleeping = [];

    public int Count { get; private set; }

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
