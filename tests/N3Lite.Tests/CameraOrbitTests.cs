using System;
using LostEden.Vehicles;
using Xunit;

namespace LostEden.Vehicle.Tests;

public class CameraOrbitTests
{
    static CameraVehicleFixedThirdSim Camera(bool locked = false)
    {
        var cam = new CameraVehicleFixedThirdSim { Frozen = locked };
        cam.ApplyDefaults();
        cam.HasSurface = false;
        cam.EyeTargetLocalPos = new Vec3(0f, 1.8f, 0f);
        cam.TargetPosition = Vec3.Zero;
        cam.Position = new Vec3(0f, 1.8f, -4.743f);
        cam.RecalcOptimalPos();
        return cam;
    }

    static void Walk(CameraVehicleFixedThirdSim cam, float speed, int frames)
    {
        for (int i = 0; i < frames; i++)
        {
            cam.TargetPosition += new Vec3(0f, 0f, speed / 60f);
            cam.Tick(1f / 60f, 5f);
        }
    }

    static float Distance(CameraVehicleFixedThirdSim cam)
        => (cam.Position - cam.GetLookTargetPos()).Length;

    static void Orbit(CameraVehicleFixedThirdSim cam, float yawDegrees, bool locked)
    {
        Vec3 lookTarget = cam.GetLookTargetPos();
        Vec3 offset = locked && !cam.GetOptimalPos().IsZero
            ? cam.GetOptimalPos() - lookTarget
            : cam.Position - lookTarget;
        float distance = offset.Length;
        if (distance < 1e-4f)
            return;

        float r = yawDegrees * MathF.PI / 180f;
        float cos = MathF.Cos(r), sin = MathF.Sin(r);
        var rotated = new Vec3(
            offset.X * cos + offset.Z * sin,
            offset.Y,
            -offset.X * sin + offset.Z * cos);

        if (distance > CameraVehicleSim.MaxZoomDistance)
            rotated = rotated / distance * CameraVehicleSim.MaxZoomDistance;

        Vec3 target = lookTarget + rotated;
        cam.SetRelPos(target);

        if (locked)
        {
            cam.UpdateHeadingToPos(target, false);
            return;
        }

        if (cam.Speed < CameraVehicleSim.OrbitResyncSpeed)
        {
            cam.ResyncFollowDistance();
            cam.ForcedUpdate(false);
        }
    }

    [Fact]
    public void Orbit_WhileWalking_DoesNotChangeTheDistance()
    {
        var cam = Camera();
        Walk(cam, 4f, 400);

        float before = Distance(cam);
        Assert.True(before > cam.FollowDistance + 0.5f,
            $"the camera should be lagging for this test to mean anything: {before}");

        Orbit(cam, 15f, locked: false);

        Assert.Equal(before, Distance(cam), 3);
    }

    [Fact]
    public void Orbit_RepeatedWhileWalking_DoesNotRatchetTheDistanceOutward()
    {
        var cam = Camera();
        Walk(cam, 4f, 400);
        float baseline = Distance(cam);

        for (int drag = 0; drag < 8; drag++)
        {
            Orbit(cam, 15f, locked: false);
            Walk(cam, 4f, 120);
        }

        Assert.True(MathF.Abs(Distance(cam) - baseline) < 0.2f,
            $"distance drifted from {baseline} to {Distance(cam)} over 8 drags");
    }

    [Fact]
    public void Orbit_WhileWalking_ActuallyRotatesTheCamera()
    {
        var cam = Camera();
        Walk(cam, 4f, 400);
        Vec3 before = cam.Position - cam.GetLookTargetPos();

        Orbit(cam, 30f, locked: false);
        Vec3 after = cam.Position - cam.GetLookTargetPos();

        float dot = Vec3.Dot(before, after) / (before.Length * after.Length);
        float angle = MathF.Acos(Math.Clamp(dot, -1f, 1f)) * 180f / MathF.PI;
        Assert.True(angle > 20f, $"expected roughly a 30 degree swing, got {angle}");
    }

    [Fact]
    public void Orbit_WhileStopped_HoldsItsDistanceAcrossDrags()
    {
        var cam = Camera();
        Walk(cam, 4f, 300);
        Walk(cam, 0f, 400);

        Assert.True(cam.Speed < CameraVehicleSim.OrbitResyncSpeed,
            $"expected the camera to have stopped, speed {cam.Speed}");

        float baseline = Distance(cam);
        for (int drag = 0; drag < 4; drag++)
        {
            Orbit(cam, 20f, locked: false);
            Walk(cam, 0f, 60);
        }

        Assert.Equal(baseline, Distance(cam), 2);
    }

    [Fact]
    public void Orbit_InLockMode_AlsoHoldsItsDistance()
    {
        var cam = Camera(locked: true);
        Walk(cam, 4f, 300);
        float baseline = Distance(cam);

        for (int drag = 0; drag < 4; drag++)
        {
            Orbit(cam, 20f, locked: true);
            Walk(cam, 4f, 60);
        }

        Assert.True(MathF.Abs(Distance(cam) - baseline) < 0.2f,
            $"lock drifted from {baseline} to {Distance(cam)}");
    }

    static void Frame(CameraVehicleFixedThirdSim cam, Vec3 target, float yawDegrees, bool syncBeforeOrbit)
    {
        if (syncBeforeOrbit)
            cam.TargetPosition = target;

        Orbit(cam, yawDegrees, locked: true);

        if (!syncBeforeOrbit)
            cam.TargetPosition = target;

        cam.Tick(1f / 60f, 6f);
    }

    static float WorstUnitError(bool syncBeforeOrbit, float yawPerFrame)
    {
        var cam = Camera(locked: true);
        var target = Vec3.Zero;
        float worst = 0f;

        for (int f = 0; f < 600; f++)
        {
            target.Z += 6f / 60f;
            Frame(cam, target, yawPerFrame, syncBeforeOrbit);
            worst = MathF.Max(worst, MathF.Abs(cam.PreferredDirection.Length - 1f));
        }

        return worst;
    }

    [Theory]
    [InlineData(0f)]
    [InlineData(1.5f)]
    [InlineData(-3f)]
    public void Orbit_InLock_WhileWalking_KeepsThePreferredDirectionUnitLength(float yawPerFrame)
    {
        Assert.Equal(0f, WorstUnitError(syncBeforeOrbit: false, yawPerFrame), 4);
    }

    [Fact]
    public void Orbit_InLock_WhileWalking_NeverRatchetsOutward()
    {
        var cam = Camera(locked: true);
        var target = Vec3.Zero;
        float follow = cam.FollowDistance;

        for (int f = 0; f < 600; f++)
        {
            target.Z += 6f / 60f;
            Frame(cam, target, 1.5f, syncBeforeOrbit: false);
            Assert.True(Distance(cam) <= follow + 0.01f,
                $"frame {f}: camera pushed out to {Distance(cam)} from {follow}");
        }
    }

    [Fact]
    public void Orbit_InLock_ReturnsToItsFollowDistanceOnceTheDragStops()
    {
        var cam = Camera(locked: true);
        var target = Vec3.Zero;
        float follow = cam.FollowDistance;

        for (int f = 0; f < 300; f++)
        {
            target.Z += 6f / 60f;
            Frame(cam, target, 1.5f, syncBeforeOrbit: false);
        }

        for (int f = 0; f < 300; f++)
        {
            target.Z += 6f / 60f;
            cam.TargetPosition = target;
            cam.Tick(1f / 60f, 6f);
        }

        Assert.Equal(follow, Distance(cam), 2);
        Assert.Equal(1f, cam.PreferredDirection.Length, 4);
    }

    [Theory]
    [InlineData(0f)]
    [InlineData(1.5f)]
    public void TheOldLateUpdateOrderIsWhatRatchetedTheCameraOut(float yawPerFrame)
    {
        Assert.True(WorstUnitError(syncBeforeOrbit: true, yawPerFrame) > 0.5f);
    }

    const float PitchLimitY = 0.9999f;

    static Vec3 Step(Vec3 dir, float yawDegrees, float pitchDegrees)
    {
        if (yawDegrees != 0f)
        {
            float r = yawDegrees * MathF.PI / 180f;
            float c = MathF.Cos(r), sn = MathF.Sin(r);
            dir = new Vec3(dir.X * c + dir.Z * sn, dir.Y, -dir.X * sn + dir.Z * c);
        }

        if (pitchDegrees != 0f)
        {
            Vec3 axis = Vec3.Cross(Vec3.ReferenceUp, dir);
            if (axis.LengthSquared > 1e-8f)
            {
                Vec3 candidate = Quat.FromAxisAngle(
                    axis / axis.Length, pitchDegrees * MathF.PI / 180f) * dir;
                if (MathF.Abs(candidate.Y) <= PitchLimitY)
                    dir = candidate;
            }
        }

        return dir;
    }

    static float Elevation(Vec3 d) => MathF.Asin(Math.Clamp(d.Y, -1f, 1f)) * 180f / MathF.PI;
    static float Azimuth(Vec3 d) => MathF.Atan2(d.X, d.Z) * 180f / MathF.PI;

    static readonly Vec3 DefaultStart = new Vec3(0f, 0.3162f, -0.9487f);

    [Theory]
    [InlineData(-3f)]
    [InlineData(3f)]
    public void Pitch_CannotCrossThePole(float pitchPerStep)
    {
        Vec3 dir = DefaultStart;
        float startAzimuth = Azimuth(dir);

        for (int i = 0; i < 200; i++)
            dir = Step(dir, 0f, pitchPerStep);

        Assert.True(MathF.Abs(dir.Y) <= PitchLimitY,
            $"direction reached |y|={MathF.Abs(dir.Y)}, past the guard");
        Assert.True(MathF.Abs(Mathf_DeltaAngle(startAzimuth, Azimuth(dir))) < 1f,
            "azimuth flipped, so the direction wrapped over the top");
    }

    [Theory]
    [InlineData(-3f)]
    [InlineData(3f)]
    public void Pitch_CanAlwaysComeBackFromTheLimit(float pitchPerStep)
    {
        Vec3 dir = DefaultStart;
        for (int i = 0; i < 200; i++)
            dir = Step(dir, 0f, pitchPerStep);

        float atLimit = Elevation(dir);
        for (int i = 0; i < 10; i++)
            dir = Step(dir, 0f, -pitchPerStep);

        float moved = Elevation(dir) - atLimit;
        Assert.True(MathF.Abs(moved) > 25f,
            $"expected to pitch back ~30 degrees, moved {moved}");

        Assert.True(MathF.Abs(Elevation(dir)) < MathF.Abs(atLimit) - 25f,
            $"pitched the wrong way: {atLimit} -> {Elevation(dir)}");
    }

    [Theory]
    [InlineData(-3f)]
    [InlineData(3f)]
    public void Yaw_StillWorksAtThePitchLimit(float pitchPerStep)
    {
        Vec3 dir = DefaultStart;
        for (int i = 0; i < 200; i++)
            dir = Step(dir, 0f, pitchPerStep);

        float before = Azimuth(dir);
        Vec3 yawed = Step(dir, 15f, 0f);

        Assert.Equal(15f, Mathf_DeltaAngle(before, Azimuth(yawed)), 2);
    }

    [Fact]
    public void Pitch_NeverReachesThePoleUnderMixedInput()
    {
        Vec3 dir = DefaultStart;
        float worst = 0f;

        for (int i = 0; i < 500; i++)
        {
            dir = Step(dir, 7f, -5f);
            worst = MathF.Max(worst, MathF.Abs(dir.Y));

            Assert.True(Vec3.Cross(Vec3.ReferenceUp, dir).LengthSquared > 1e-8f,
                $"pitch axis degenerated at step {i}");
        }

        Assert.True(worst <= PitchLimitY, $"max |y| was {worst}");
    }

    static float Mathf_DeltaAngle(float a, float b)
    {
        float d = (b - a) % 360f;
        if (d > 180f) d -= 360f;
        if (d < -180f) d += 360f;
        return d;
    }
}
