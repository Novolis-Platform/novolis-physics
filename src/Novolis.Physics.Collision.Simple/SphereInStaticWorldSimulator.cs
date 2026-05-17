using System.Numerics;

namespace Novolis.Physics.Collision.Simple;

/// <summary>Integrates many equal-radius spheres against a static BVH world with pairwise grid resolution.</summary>
public sealed class SphereInStaticWorldSimulator
{
    private readonly UniformGridSphereContactSolver _contactSolver = new();
    private readonly SphereSoA _soa = new();
    private bool _pileSettled;

    public SphereInStaticWorldOptions Options { get; set; } = new();

    public SphereSimulationStats LastStats { get; private set; }

    public void ResetPileState() => _pileSettled = false;

    public void MarkPileUnsettled() => _pileSettled = false;

    public void Step(
        BvhStaticWorld staticWorld,
        IList<SphereState> spheres,
        InteriorClampVolume interior,
        float deltaSeconds)
    {
        var stats = new SphereSimulationStats();
        if (deltaSeconds <= 0f || spheres.Count == 0)
        {
            LastStats = stats;
            return;
        }

        var opts = Options;
        var radius = opts.Radius;
        var gridCell = radius * opts.GridCellRadiusScale;
        var subSteps = SubStepsForCount(spheres.Count);
        var impulseIterations = SolveIterationsForCount(spheres.Count);
        stats.PhysicsSubSteps = subSteps;

        var activeCount = 0;
        var sleepingCount = 0;

        foreach (var sphere in spheres)
        {
            UpdateSleeping(sphere, opts);
            if (sphere.IsSleeping)
            {
                sleepingCount++;
                continue;
            }

            activeCount++;
            var pos = sphere.Position;
            var vel = sphere.Velocity;
            stats.IntegratorReflections += BvhStaticSphereIntegrator.AdvanceWithUniformAccelerationAndLinearDrag(
                staticWorld,
                ref pos,
                ref vel,
                radius,
                deltaSeconds,
                opts.Gravity,
                opts.LinearDragPerSecond,
                substepsPerStep: subSteps,
                normalRestitution: opts.StaticRestitution);
            sphere.Position = pos;
            sphere.Velocity = vel;
            ApplyGroundFriction(sphere, deltaSeconds, opts);
            UpdateSleeping(sphere, opts);
            if (sphere.IsSleeping)
            {
                sleepingCount++;
                activeCount--;
            }
        }

        stats.ActiveCount = activeCount;
        stats.SleepingCount = sleepingCount;

        if (spheres.Count > 1)
        {
            if (activeCount == 0 && _pileSettled)
            {
                stats.SphereContactSkipped = true;
            }
            else
            {
                ResolveSphereOverlaps(
                    spheres,
                    activeCount,
                    impulseIterations,
                    radius,
                    gridCell,
                    opts.SphereRestitution,
                    ref stats);
            }
        }

        stats.ClampedCount = 0;
        foreach (var sphere in spheres)
        {
            if (ClampToInterior(sphere, interior, opts.MaxSpeedMps))
                stats.ClampedCount++;
            ClampVelocity(sphere, opts.MaxSpeedMps);
            UpdateSleeping(sphere, opts);
        }

        LastStats = stats;
    }

    public void DepenetrateSpawnedRange(
        IList<SphereState> spheres,
        int startIndex,
        int endIndex,
        InteriorClampVolume interior)
    {
        var count = endIndex - startIndex;
        var iterations = count switch
        {
            > 50 => 12,
            > 10 => 8,
            _ => 6,
        };
        var radius = Options.Radius;

        for (var iter = 0; iter < iterations; iter++)
        {
            for (var i = startIndex; i < endIndex; i++)
            {
                for (var j = i + 1; j < endIndex; j++)
                {
                    var a = spheres[i].Position;
                    var b = spheres[j].Position;
                    SphereOverlapResolution.Separate(ref a, ref b, radius);
                    spheres[i].Position = a;
                    spheres[j].Position = b;
                }

                for (var j = 0; j < startIndex; j++)
                {
                    var a = spheres[i].Position;
                    var b = spheres[j].Position;
                    SphereOverlapResolution.Separate(ref a, ref b, radius);
                    spheres[i].Position = a;
                    spheres[j].Position = b;
                }
            }
        }

        var maxSpeed = Options.MaxSpeedMps;
        for (var i = startIndex; i < endIndex; i++)
        {
            ClampToInterior(spheres[i], interior, maxSpeed);
            ClampVelocity(spheres[i], maxSpeed);
        }
    }

    private void ResolveSphereOverlaps(
        IList<SphereState> spheres,
        int activeCount,
        int impulseIterations,
        float radius,
        float gridCell,
        float restitution,
        ref SphereSimulationStats stats)
    {
        var allSleeping = activeCount == 0;
        var positionIters = allSleeping ? 1 : DepenetrateIterationsForCount(spheres.Count);
        var impulseIters = activeCount > 1 ? impulseIterations : 0;
        stats.SphereContactIterations = positionIters + impulseIters;

        _soa.SyncFrom(spheres);
        var frameContacts = 0;

        for (var iter = 0; iter < positionIters; iter++)
        {
            var r = _contactSolver.Resolve(_soa, radius, gridCell, restitution, applyImpulses: false, awakePairsOnly: false);
            stats.SpherePairChecks += r.PairChecks;
            frameContacts += r.Contacts;
        }

        for (var iter = 0; iter < impulseIters; iter++)
        {
            var r = _contactSolver.Resolve(_soa, radius, gridCell, restitution, applyImpulses: true, awakePairsOnly: true);
            stats.SpherePairChecks += r.PairChecks;
            frameContacts += r.Contacts;
        }

        stats.SphereContacts = frameContacts;
        _soa.SyncTo(spheres);

        _pileSettled = allSleeping && frameContacts == 0;
        if (!allSleeping)
            _pileSettled = false;

        if (impulseIters > 0)
        {
            var maxSpeed = Options.MaxSpeedMps;
            foreach (var sphere in spheres)
                ClampVelocity(sphere, maxSpeed);
        }
    }

    private static void UpdateSleeping(SphereState sphere, SphereInStaticWorldOptions opts) =>
        sphere.IsSleeping = sphere.IsGrounded && sphere.Speed < opts.SleepSpeedThreshold;

    private static void ApplyGroundFriction(SphereState sphere, float deltaSeconds, SphereInStaticWorldOptions opts)
    {
        var floorContactY = opts.FloorHeight + opts.Radius;
        sphere.IsGrounded = sphere.Position.Y <= floorContactY + opts.GroundContactSlack && sphere.Velocity.Y <= 1.2f;
        if (!sphere.IsGrounded)
            return;

        if (sphere.Position.Y < floorContactY)
            sphere.Position = new Vector3(sphere.Position.X, floorContactY, sphere.Position.Z);

        var horizontalSpeed = MathF.Sqrt(sphere.Velocity.X * sphere.Velocity.X + sphere.Velocity.Z * sphere.Velocity.Z);
        if (horizontalSpeed > 1e-5f)
        {
            var scale = MathF.Max(0f, 1f - (float)(opts.GroundFrictionPerSecond * deltaSeconds));
            sphere.Velocity = new Vector3(sphere.Velocity.X * scale, sphere.Velocity.Y, sphere.Velocity.Z * scale);
        }

        if (MathF.Abs(sphere.Velocity.Y) < 0.6f)
            sphere.Velocity = new Vector3(sphere.Velocity.X, MathF.Min(sphere.Velocity.Y, 0f), sphere.Velocity.Z);
    }

    private static bool ClampToInterior(SphereState sphere, InteriorClampVolume interior, float maxSpeedMps)
    {
        var before = sphere.Position;
        sphere.Position = ClampPosition(sphere.Position, interior);
        sphere.Velocity = ClampVelocity(sphere.Velocity, maxSpeedMps);
        return (sphere.Position - before).LengthSquared() > 1e-8f;
    }

    private static Vector3 ClampPosition(Vector3 p, InteriorClampVolume b) =>
        new(
            System.Math.Clamp(p.X, b.MinX, b.MaxX),
            System.Math.Clamp(p.Y, b.MinY, b.MaxY),
            System.Math.Clamp(p.Z, b.MinZ, b.MaxZ));

    private static void ClampVelocity(SphereState sphere, float maxSpeedMps) =>
        sphere.Velocity = ClampVelocity(sphere.Velocity, maxSpeedMps);

    private static Vector3 ClampVelocity(Vector3 v, float maxSpeedMps)
    {
        var speed = v.Length();
        if (speed <= maxSpeedMps || speed < 1e-6f)
            return v;

        return v * (maxSpeedMps / speed);
    }

    private static int SubStepsForCount(int count) =>
        count switch
        {
            > 200 => 4,
            > 80 => 8,
            > 30 => 12,
            _ => 16,
        };

    private static int SolveIterationsForCount(int count) =>
        count switch
        {
            > 150 => 1,
            > 50 => 2,
            _ => 3,
        };

    private static int DepenetrateIterationsForCount(int count) =>
        count switch
        {
            > 1000 => 4,
            > 300 => 3,
            > 80 => 2,
            _ => 1,
        };
}

/// <summary>Configuration for <see cref="SphereInStaticWorldSimulator"/>.</summary>
public sealed class SphereInStaticWorldOptions
{
    public float Radius { get; set; } = 0.22f;
    public Vector3 Gravity { get; set; } = new(0f, -9.80665f, 0f);
    public double LinearDragPerSecond { get; set; } = 0.048;
    public double StaticRestitution { get; set; } = 0.82;
    public float SphereRestitution { get; set; } = 0.88f;
    public double GroundFrictionPerSecond { get; set; } = 9.5;
    public float FloorHeight { get; set; }
    public float GroundContactSlack { get; set; } = 0.05f;
    public float SleepSpeedThreshold { get; set; } = 0.12f;
    public float MaxSpeedMps { get; set; } = 18f;
    public float GridCellRadiusScale { get; set; } = 2.25f;
}

/// <summary>Per-frame counters from <see cref="SphereInStaticWorldSimulator.Step"/>.</summary>
public struct SphereSimulationStats
{
    public int ActiveCount;
    public int SleepingCount;
    public int SphereContacts;
    public int SpherePairChecks;
    public int IntegratorReflections;
    public int PhysicsSubSteps;
    public int SphereContactIterations;
    public int ClampedCount;
    public bool SphereContactSkipped;
}
