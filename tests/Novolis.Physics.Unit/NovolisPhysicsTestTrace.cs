using Novolis.Physics.TestSupport;

namespace Novolis.Physics.Unit;

/// <summary>Shared console trace for Novolis.Physics unit tests (TestOutput lock is enough for parallel runs).</summary>
internal static class NovolisPhysicsTestTrace
{
    internal const string NotInParallelKey = TraceParallelismKeys.NovolisPhysicsBallistics;

    internal static readonly TestOutput Out = TestOutput.ForScope("Novolis.Physics", useSharedConsoleLock: true);
}
