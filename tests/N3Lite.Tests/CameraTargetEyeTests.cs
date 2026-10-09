using N3Lite;
using Xunit;

namespace N3Lite.Tests;

public class CameraTargetEyeTests
{
    static readonly Vec3 Standing = new Vec3(0.1f, 1.8f, 0.05f);

    [Fact]
    public void FirstReadCachesAllAxesThenOnlyTheHeightFollows()
    {
        var eye = new CameraTargetEye();
        Vec3 first = eye.FromAttractor(Standing, 2f);
        Assert.Equal(new Vec3(0.2f, 3.6f, 0.1f), first);

        Vec3 later = eye.FromAttractor(new Vec3(5f, 1f, 7f), 1.5f);
        Assert.Equal(new Vec3(0.2f, 1.5f, 0.1f), later);
    }

    [Fact]
    public void ThirdPersonSnapsOnTheFirstUpdateThenEasesAFifthPerCall()
    {
        var eye = new CameraTargetEye();
        Vec3 current = new Vec3(0f, 2f, 0f);

        // Snap is set at construction: the attractor is taken as it is, and Snap clears.
        current = eye.Update(current, CameraViewMode.Lock, true, true, Standing, 1f);
        Assert.Equal(Standing, current);
        Assert.False(eye.Snap);

        // The swim lowers the attractor: a fifth of the gap per call.
        Vec3 swim = new Vec3(9f, 0.9f, 9f);
        Vec3 next = eye.Update(current, CameraViewMode.Lock, true, true, swim, 1f);
        Assert.Equal(1.8f * 0.8f + 0.9f * 0.2f, next.Y, 5);
        Assert.Equal(Standing.X, next.X, 5);
        Assert.Equal(Standing.Z, next.Z, 5);
    }

    [Fact]
    public void ThirdPersonTakesTheTargetWhenTheCurrentHeightIsZero()
    {
        var eye = new CameraTargetEye { Snap = false };
        eye.FromAttractor(Standing, 1f);
        Vec3 next = eye.Update(new Vec3(0f, 0f, 0f), CameraViewMode.Rubber, true, true, new Vec3(0f, 1.2f, 0f), 1f);
        Assert.Equal(1.2f, next.Y, 5);
    }

    [Fact]
    public void FirstPersonAlmostTakesTheTarget()
    {
        var eye = new CameraTargetEye { Snap = false };
        eye.FromAttractor(Standing, 1f);
        Vec3 current = new Vec3(Standing.X, 1.8f, Standing.Z);
        Vec3 next = eye.Update(current, CameraViewMode.FirstPerson, true, true, new Vec3(0f, 1.0f, 0f), 1f);
        Assert.Equal(1.0f * 0.999f + 1.8f * 0.0009999871f, next.Y, 5);
    }

    [Fact]
    public void NoBobHoldsUntilHalfAMetreOffThenEasesToATenth()
    {
        var eye = new CameraTargetEye { NoBob = true, Snap = false };
        eye.FromAttractor(Standing, 1f);
        Vec3 current = Standing;

        // 0.4 m of head bob: inside the 0.5 m start, held.
        Vec3 held = eye.Update(current, CameraViewMode.Lock, true, true, new Vec3(0f, 1.4f, 0f), 1f);
        Assert.Equal(current, held);

        // 0.9 m down: eases a hundredth per call, and keeps easing under 0.5 m.
        Vec3 target = new Vec3(0f, 0.9f, 0f);
        int calls = 0;
        while (current.Y - 0.9f >= 0.1f && calls < 10000)
        {
            Vec3 next = eye.Update(current, CameraViewMode.Lock, true, true, target, 1f);
            Assert.True(next.Y < current.Y);
            current = next;
            calls++;
        }
        Assert.True(calls < 10000);

        // Inside 0.1 m it stops.
        Vec3 stopped = eye.Update(current, CameraViewMode.Lock, true, true, target, 1f);
        Assert.Equal(current, stopped);
    }

    [Fact]
    public void AMissingAttractorSetsSnapAndTheHeightIsFloored()
    {
        var eye = new CameraTargetEye { Snap = false };
        Vec3 next = eye.Update(Standing, CameraViewMode.Lock, true, false, default, 1f);
        Assert.Equal(new Vec3(0f, 0.3f, 0f), next);
        Assert.False(eye.Snap);

        Vec3 noMesh = eye.Update(Standing, CameraViewMode.Lock, false, false, default, 1f);
        Assert.Equal(new Vec3(0f, 0.3f, 0f), noMesh);
    }

    [Fact]
    public void FixedThirdSetEyeStoresWithoutTheNudge()
    {
        var cam = new CameraVehicleFixedThirdSim();
        cam.ApplyDefaults();
        cam.HasSurface = false;
        cam.Position = new Vec3(1f, 2f, 3f);
        cam.SetEyeTargetLocalPos(new Vec3(0f, 1.5f, 0f));
        Assert.Equal(new Vec3(0f, 1.5f, 0f), cam.EyeTargetLocalPos);
        Assert.Equal(new Vec3(1f, 2f, 3f), cam.Position);
    }

    [Fact]
    public void FirstPersonSetEyePlacesTheCameraAtTheEye()
    {
        var cam = new CameraVehicleFirstPersonSim { HasSurface = false };
        cam.TargetPosition = new Vec3(10f, 0f, 10f);
        cam.SetEyeTargetLocalPos(new Vec3(0f, 1.6f, 0.2f));
        Assert.Equal(0.2f, cam.BackPush);
        Assert.Equal(new Vec3(10f, 1.6f, 10.2f), cam.Position);
    }
}

public class CameraZoomRequestTests
{
    [Fact]
    public void AReversedWheelTurnDropsThePendingZoom()
    {
        var cam = new CameraVehicleFixedThirdSim();
        cam.RequestZoom(-4f);
        cam.RequestZoom(-4f);
        Assert.Equal(-8f, cam.ZoomPending);

        cam.RequestZoom(4f);
        Assert.Equal(4f, cam.ZoomPending);

        cam.RequestZoom(4f);
        Assert.Equal(8f, cam.ZoomPending);
    }
}

public class CameraZoomWallTests
{
    // A floor at y 0 and a wall across the camera's back at z = -5.5.
    static N3Lite.Surfaces.TriangleMeshSurface Room()
    {
        var v = new[]
        {
            new Vec3(-50f, 0f, -50f), new Vec3(50f, 0f, -50f), new Vec3(50f, 0f, 50f), new Vec3(-50f, 0f, 50f),
            new Vec3(-50f, 0f, -5.5f), new Vec3(50f, 0f, -5.5f), new Vec3(50f, 20f, -5.5f), new Vec3(-50f, 20f, -5.5f),
        };
        return new N3Lite.Surfaces.TriangleMeshSurface(v, new[] { 0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7 });
    }

    static CameraVehicleFixedThirdSim LockCamera(N3Lite.Surfaces.ISurface surface)
    {
        var cam = new CameraVehicleFixedThirdSim { Frozen = true };
        cam.ApplyDefaults();
        cam.EyeTargetLocalPos = new Vec3(0f, 1.8f, 0f);
        cam.TargetPosition = Vec3.Zero;
        cam.RecalcOptimalPos();
        cam.Position = cam.GetOptimalPos();
        cam.Surface = surface;
        return cam;
    }

    [Fact]
    public void ZoomingOutIntoAWallPutsTheCameraBackAndDropsTheZoom()
    {
        CameraVehicleFixedThirdSim cam = LockCamera(Room());
        Vec3 before = cam.Position;
        float follow = cam.FollowDistance;

        cam.RequestZoom(-4f);
        cam.UpdateZoom(0.1f);

        Assert.Equal(0f, cam.ZoomPending);
        Assert.Equal(before, cam.Position);
        Assert.Equal(follow, cam.FollowDistance);
    }

    [Fact]
    public void WithoutASurfaceTheSameZoomGoesThrough()
    {
        CameraVehicleFixedThirdSim cam = LockCamera(null);
        cam.RequestZoom(-4f);
        cam.UpdateZoom(0.1f);

        Assert.True(cam.ZoomPending < 0f);
        Assert.True(cam.Position.Z < -5.5f);
    }
}
