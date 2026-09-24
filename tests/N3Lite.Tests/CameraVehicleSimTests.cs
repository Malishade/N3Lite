using System;
using System.Collections.Generic;
using LostEden.Vehicles;
using Xunit;

namespace LostEden.Vehicle.Tests;

public class CameraVehicleSimTests
{
    static CameraVehicleFixedThirdSim MakeCamera()
    {
        var cam = new CameraVehicleFixedThirdSim();
        cam.ApplyDefaults();
        return cam;
    }

    [Fact]
    public void DefaultSettings_MatchTheFactory()
    {
        var cam = MakeCamera();

        Assert.Equal(20f, cam.Mass);
        Assert.Equal(600f, cam.MaxForce);
        Assert.Equal(15f, cam.MaxVel);
        Assert.Equal(0.01f, cam.NearProbeOffset);
        Assert.Equal(0.05f, cam.MaxSubStep);
    }

    [Fact]
    public void DefaultSettings_TurnOffFallingAndSurfaceHug()
    {
        var cam = MakeCamera();

        Assert.False(cam.Airborne);
        Assert.False(cam.SurfaceHug);
    }

    [Fact]
    public void WithSurfaceHugOff_VerticalVelocityIsNotPinned()
    {
        var cam = MakeCamera();
        cam.SetVel(new Vec3(0f, 3f, 0f));
        cam.HasSurface = false;
        cam.Frozen = true;

        cam.Run(0.05f);

        Assert.True(cam.Position.Y > 0f, "camera should have risen, but surface hug pinned it");
    }

    [Fact]
    public void AssigningVelocityWithoutSetVelDoesNotMoveTheVehicle()
    {
        var cam = MakeCamera();
        cam.Velocity = new Vec3(0f, 3f, 0f);
        cam.HasSurface = false;
        cam.Frozen = true;

        cam.Run(0.05f);

        Assert.Equal(0f, cam.Position.Y, 6);
    }

    [Fact]
    public void DefaultOffset_IsStoredAsADirectionAndADistance()
    {
        var cam = MakeCamera();

        Assert.Equal(MathF.Sqrt(1.5f * 1.5f + 4.5f * 4.5f), cam.FollowDistance, 4);
        Assert.Equal(1f, cam.PreferredDirection.Length, 5);
        Assert.Equal(0f, cam.PreferredDirection.X, 5);
        Assert.True(cam.PreferredDirection.Z < 0f, "the camera sits behind the character");
        Assert.True(cam.PreferredDirection.Y > 0f, "and above it");
    }

    [Fact]
    public void LookTarget_IsTargetPositionPlusTheEyeOffsetRotatedIntoWorld()
    {
        var cam = MakeCamera();
        cam.TargetPosition = new Vec3(10f, 0f, 5f);
        cam.EyeTargetLocalPos = new Vec3(0f, 1.8f, 0f);
        cam.TargetRotation = Quat.FromAxisAngle(Vec3.ReferenceUp, MathF.PI / 2f);

        Vec3 look = cam.GetLookTargetPos();

        Assert.Equal(10f, look.X, 4);
        Assert.Equal(1.8f, look.Y, 4);
        Assert.Equal(5f, look.Z, 4);
    }

    [Fact]
    public void LookTarget_RotatesAHorizontalEyeOffsetWithTheTarget()
    {
        var cam = MakeCamera();
        cam.TargetPosition = Vec3.Zero;
        cam.EyeTargetLocalPos = new Vec3(0f, 0f, 1f);
        cam.TargetRotation = Quat.FromAxisAngle(Vec3.ReferenceUp, MathF.PI / 2f);

        Vec3 look = cam.GetLookTargetPos();

        Assert.Equal(1f, look.X, 4);
        Assert.Equal(0f, look.Z, 4);
    }

    [Fact]
    public void Resync_AdoptsTheMeasuredDistanceAsTheFollowDistance()
    {
        var cam = MakeCamera();
        cam.TargetPosition = Vec3.Zero;
        cam.EyeTargetLocalPos = Vec3.Zero;
        cam.Position = new Vec3(0f, 0f, -7f);

        cam.ResyncFollowDistance();

        Assert.Equal(7f, cam.FollowDistance, 4);
    }

    [Fact]
    public void Resync_ClampsTheFollowDistanceAtThePointNineMinimum()
    {
        var cam = MakeCamera();
        cam.TargetPosition = Vec3.Zero;
        cam.EyeTargetLocalPos = Vec3.Zero;
        cam.Position = new Vec3(0f, 0f, -0.2f);

        cam.ResyncFollowDistance();

        Assert.Equal(CameraVehicleSim.MinFollowDistance, cam.FollowDistance, 5);
    }

    [Fact]
    public void Resync_CalledEveryFrameWouldCollapseTheCameraOntoTheCharacter()
    {
        var cam = MakeCamera();
        cam.TargetPosition = Vec3.Zero;
        cam.EyeTargetLocalPos = new Vec3(0f, 1.8f, 0f);
        cam.HasSurface = false;
        cam.Position = new Vec3(0f, 1.8f, 0f);

        for (int i = 0; i < 300; i++)
        {
            cam.ResyncFollowDistance();
            cam.Tick(1f / 60f, 0f);
        }

        Assert.True((cam.Position - cam.GetLookTargetPos()).Length < 1.5f,
            "calling ResyncFollowDistance every frame should collapse the camera onto the character");
    }

    [Fact]
    public void MotionConstraints_DeriveForceAndSlowingDistanceFromThePointThreeReachTime()
    {
        var cam = MakeCamera();
        cam.TargetPosition = Vec3.Zero;
        cam.EyeTargetLocalPos = Vec3.Zero;

        cam.UpdateMotionConstraints(10f);

        Assert.Equal(10f, cam.MaxVel, 4);
        Assert.Equal(20f * 10f / 0.3f, cam.MaxForce, 2);
        Assert.Equal(10f * 0.3f, cam.SlowingDistance, 3);
    }

    [Fact]
    public void MotionConstraints_NeverGoBelowTheMinimumSpeed()
    {
        var cam = MakeCamera();
        cam.TargetPosition = Vec3.Zero;
        cam.EyeTargetLocalPos = Vec3.Zero;

        cam.UpdateMotionConstraints(0.1f);

        Assert.Equal(CameraVehicleSim.MinCatchUpSpeed, cam.MaxVel, 4);
    }

    [Fact]
    public void MotionConstraints_SpeedUpWhenTheCameraHasFallenBehind()
    {
        var cam = MakeCamera();
        cam.TargetPosition = Vec3.Zero;
        cam.EyeTargetLocalPos = Vec3.Zero;
        cam.Position = new Vec3(0f, 0f, -30f);

        cam.UpdateMotionConstraints(10f);

        Assert.Equal(60f, cam.MaxVel, 3);
    }

    [Fact]
    public void MotionConstraints_DoNotSpeedUpWhenTheCameraIsWhereItShouldBe()
    {
        var cam = MakeCamera();
        cam.TargetPosition = Vec3.Zero;
        cam.EyeTargetLocalPos = Vec3.Zero;
        cam.Position = new Vec3(0f, 0f, -4.74f);

        cam.UpdateMotionConstraints(10f);

        Assert.Equal(10f, cam.MaxVel, 3);
    }

    [Fact]
    public void MotionConstraints_CapTheCatchUpSpeed()
    {
        var cam = MakeCamera();
        cam.TargetPosition = Vec3.Zero;
        cam.EyeTargetLocalPos = Vec3.Zero;
        cam.Position = new Vec3(0f, 0f, -500f);

        cam.UpdateMotionConstraints(15f);

        Assert.Equal(CameraVehicleSim.MaxCatchUpSpeed, cam.MaxVel, 3);
    }

    [Fact]
    public void Tick_UsesCharacterMaxSpeedSoAStandingPlayerStillGetsAFastCamera()
    {
        var cam = MakeCamera();
        cam.TargetPosition = Vec3.Zero;
        cam.EyeTargetLocalPos = Vec3.Zero;
        cam.Position = new Vec3(0f, 0f, -4.743f);
        cam.HasSurface = false;

        cam.Tick(1f / 60f, 5f);

        Assert.Equal(CameraVehicleSim.MaxBaseSpeed, cam.MaxVel, 3);
    }

    [Fact]
    public void Orbit_MovesTheCameraImmediatelyNotOverTime()
    {
        var cam = MakeCamera();
        cam.TargetPosition = Vec3.Zero;
        cam.EyeTargetLocalPos = Vec3.Zero;
        cam.HasSurface = false;
        cam.StayBehind = false;
        cam.SetPreferredOffset(new Vec3(0f, 0f, -4.743f));
        cam.Position = new Vec3(0f, 0f, -4.743f);

        for (int i = 0; i < 120; i++)
            cam.Tick(1f / 60f, 5f);

        var swung = new Vec3(-4.743f, 0f, 0f);
        cam.SetRelPos(swung);
        cam.ForcedUpdate(false);

        Assert.Equal(-4.743f, cam.Position.X, 3);
        Assert.Equal(0f, cam.Position.Z, 3);

        for (int i = 0; i < 120; i++)
            cam.Tick(1f / 60f, 5f);

        Assert.True((cam.Position - swung).Length < 0.3f,
            $"expected to hold the new angle, drifted to {cam.Position}");
    }

    [Fact]
    public void OptimalPos_IsTheLookTargetPlusTheDirectionTimesTheFollowDistance()
    {
        var cam = MakeCamera();
        cam.TargetPosition = Vec3.Zero;
        cam.EyeTargetLocalPos = Vec3.Zero;
        cam.StayBehind = false;
        cam.SetPreferredOffset(new Vec3(0f, 0f, -5f));

        cam.RecalcOptimalPos();

        Assert.Equal(0f, cam.OptimalPos.X, 4);
        Assert.Equal(-5f, cam.OptimalPos.Z, 4);
    }

    [Fact]
    public void OptimalPos_StayBehindRotatesTheOffsetWithTheTarget()
    {
        var cam = MakeCamera();
        cam.TargetPosition = Vec3.Zero;
        cam.EyeTargetLocalPos = Vec3.Zero;
        cam.StayBehind = true;
        cam.SetPreferredOffset(new Vec3(0f, 0f, -5f));
        cam.TargetRotation = Quat.FromAxisAngle(Vec3.ReferenceUp, MathF.PI / 2f);

        cam.RecalcOptimalPos();

        Assert.Equal(-5f, cam.OptimalPos.X, 3);
        Assert.Equal(0f, cam.OptimalPos.Z, 3);
    }

    [Fact]
    public void Occlusion_ClearViewLeavesTheSentinelZero()
    {
        var cam = MakeCamera();
        cam.LineOfSight = (a, b) => true;

        cam.RecalcOptimalPos();

        Assert.True(cam.AdjustedPos.IsZero);
        Assert.Equal(0, cam.LastOcclusionIterations);
    }

    [Fact]
    public void Occlusion_BinarySearchesTheFurthestVisiblePointTowardsTheHead()
    {
        var cam = MakeCamera();
        cam.TargetPosition = Vec3.Zero;
        cam.EyeTargetLocalPos = Vec3.Zero;
        cam.StayBehind = false;
        cam.SetPreferredOffset(new Vec3(0f, 0f, -10f));

        cam.LineOfSight = (from, to) => to.Length <= 3f;

        cam.RecalcOptimalPos();

        Assert.False(cam.AdjustedPos.IsZero);
        Assert.True(cam.LastOcclusionIterations > 0);
        Assert.True(cam.LastOcclusionIterations <= CameraVehicleFixedThirdSim.OcclusionSearchMaxIterations);

        float distance = cam.AdjustedPos.Length;
        float expected = 3f - cam.NearProbeOffset;
        Assert.True(Math.Abs(distance - expected) < 0.2f,
            $"expected to settle near {expected} m (3 m wall minus the probe offset), got {distance}");
        Assert.True(cam.AdjustedPos.X == 0f && cam.AdjustedPos.Y == 0f,
            "the solve must stay on the line to the head, not slide sideways");
    }

    [Fact]
    public void Occlusion_FullyBlockedCollapsesTowardsTheLowEndOfTheBracket()
    {
        var cam = MakeCamera();
        cam.TargetPosition = Vec3.Zero;
        cam.EyeTargetLocalPos = Vec3.Zero;
        cam.StayBehind = false;
        cam.SetPreferredOffset(new Vec3(0f, 0f, -10f));
        cam.LineOfSight = (from, to) => false;

        cam.RecalcOptimalPos();

        Assert.True(cam.AdjustedPos.Length < 0.3f,
            $"expected to collapse onto the head, got {cam.AdjustedPos.Length}");
    }

    [Fact]
    public void Occlusion_IsSkippedEntirelyWithoutASurface()
    {
        var cam = MakeCamera();
        cam.HasSurface = false;
        cam.LineOfSight = (a, b) => throw new InvalidOperationException("must not be queried");

        cam.RecalcOptimalPos();

        Assert.Equal(0, cam.LastOcclusionIterations);
    }

    [Fact]
    public void UpdateHeadingToPos_TurnsAWorldPositionIntoAPreferredDirection()
    {
        var cam = MakeCamera();
        cam.TargetPosition = Vec3.Zero;
        cam.EyeTargetLocalPos = Vec3.Zero;
        cam.StayBehind = false;
        cam.HasSurface = false;
        cam.FollowDistance = 5f;

        cam.UpdateHeadingToPos(new Vec3(5f, 0f, 0f), recalcDistance: false);

        Assert.Equal(1f, cam.PreferredDirection.X, 4);
        Assert.Equal(0f, cam.PreferredDirection.Z, 4);
        Assert.Equal(5f, cam.FollowDistance, 4);
        Assert.Equal(5f, cam.OptimalPos.X, 4);
    }

    [Fact]
    public void UpdateHeadingToPos_CanAdoptTheDistanceToo()
    {
        var cam = MakeCamera();
        cam.TargetPosition = Vec3.Zero;
        cam.EyeTargetLocalPos = Vec3.Zero;
        cam.StayBehind = false;
        cam.HasSurface = false;
        cam.FollowDistance = 5f;

        cam.UpdateHeadingToPos(new Vec3(0f, 0f, -12f), recalcDistance: true);

        Assert.Equal(12f, cam.FollowDistance, 4);
        Assert.Equal(1f, cam.PreferredDirection.Length, 4);
    }

    [Fact]
    public void UpdateHeadingToPos_StoresTheDirectionInTheTargetsFrame()
    {
        var cam = MakeCamera();
        cam.TargetPosition = Vec3.Zero;
        cam.EyeTargetLocalPos = Vec3.Zero;
        cam.StayBehind = true;
        cam.HasSurface = false;
        cam.FollowDistance = 5f;
        cam.TargetRotation = Quat.FromAxisAngle(Vec3.ReferenceUp, MathF.PI / 2f);

        cam.UpdateHeadingToPos(new Vec3(-5f, 0f, 0f), recalcDistance: false);

        Assert.Equal(-1f, cam.PreferredDirection.Z, 3);
        Assert.Equal(0f, cam.PreferredDirection.X, 3);

        cam.TargetRotation = Quat.FromAxisAngle(Vec3.ReferenceUp, -MathF.PI / 2f);
        cam.RecalcOptimalPos();

        Assert.Equal(5f, cam.OptimalPos.X, 3);
    }

    [Fact]
    public void UpdateHeadingToPos_ClearsTheOcclusionSentinel()
    {
        var cam = MakeCamera();
        cam.TargetPosition = Vec3.Zero;
        cam.EyeTargetLocalPos = Vec3.Zero;
        cam.StayBehind = false;
        cam.HasSurface = true;
        cam.LineOfSight = (a, b) => true;

        cam.UpdateHeadingToPos(new Vec3(0f, 0f, -5f), recalcDistance: false);

        Assert.True(cam.AdjustedPos.IsZero);
    }

    static CameraVehicleFixedThirdSim ZoomRig(float startDistance)
    {
        var cam = MakeCamera();
        cam.TargetPosition = Vec3.Zero;
        cam.EyeTargetLocalPos = Vec3.Zero;
        cam.HasSurface = false;
        cam.StayBehind = false;
        cam.SetPreferredOffset(new Vec3(0f, 0f, -startDistance));
        cam.Position = new Vec3(0f, 0f, -startDistance);
        return cam;
    }

    [Fact]
    public void ZoomSteer_PositiveRateSeeksPastTheHeadSoTheCameraClosesIn()
    {
        var cam = ZoomRig(10f);

        SteeringResult r = cam.ZoomSteer(1f, out Vec3 steer);

        Assert.Equal(SteeringResult.Force, r);
        Assert.True(steer.Z > 0f, $"expected a pull toward +Z (the head), got {steer}");
    }

    [Fact]
    public void ZoomSteer_NegativeRatePushesTheCameraOut()
    {
        var cam = ZoomRig(10f);

        SteeringResult r = cam.ZoomSteer(-1f, out Vec3 steer);

        Assert.Equal(SteeringResult.Force, r);
        Assert.True(steer.Z < 0f, $"expected a push toward -Z (away), got {steer}");
    }

    [Fact]
    public void ZoomSteer_RefusesToZoomInsideTheMinimumDistance()
    {
        var cam = ZoomRig(0.5f);

        Assert.Equal(SteeringResult.Halt, cam.ZoomSteer(1f, out _));
    }

    [Fact]
    public void ZoomSteer_RefusesToZoomOutBeyondTheMaximumDistance()
    {
        var cam = ZoomRig(30f);

        Assert.Equal(SteeringResult.Halt, cam.ZoomSteer(-1f, out _));
    }

    [Fact]
    public void ZoomSteer_StillAllowsZoomingBackInFromBeyondTheMaximum()
    {
        var cam = ZoomRig(30f);

        Assert.Equal(SteeringResult.Force, cam.ZoomSteer(1f, out _));
    }

    [Fact]
    public void Zoom_QueuedDistanceIsSpentAndThenCommitted()
    {
        var cam = ZoomRig(10f);

        cam.RequestZoom(4f);
        for (int i = 0; i < 300; i++)
            cam.Tick(1f / 60f, 0f);

        Assert.Equal(0f, cam.ZoomPending);
        Assert.Equal(0f, cam.ZoomRate);

        float distance = (cam.Position - cam.GetLookTargetPos()).Length;
        Assert.InRange(distance, 5.5f, 7f);
        Assert.Equal(distance, cam.FollowDistance, 1);
    }

    [Fact]
    public void Zoom_OutwardAlsoCommits()
    {
        var cam = ZoomRig(5f);

        cam.RequestZoom(-5f);
        for (int i = 0; i < 300; i++)
            cam.Tick(1f / 60f, 0f);

        float distance = (cam.Position - cam.GetLookTargetPos()).Length;
        Assert.True(distance > 7f, $"expected to have backed off, distance {distance}");
        Assert.Equal(distance, cam.FollowDistance, 1);
    }

    [Fact]
    public void Zoom_StopsAtTheMinimumDistanceHowMuchEverIsQueued()
    {
        var cam = ZoomRig(5f);

        cam.RequestZoom(50f);
        for (int i = 0; i < 600; i++)
            cam.Tick(1f / 60f, 0f);

        float distance = (cam.Position - cam.GetLookTargetPos()).Length;
        Assert.True(distance >= CameraVehicleSim.MinZoomDistance - 0.05f,
            $"expected to stop at the 0.7 m floor, got {distance}");
    }

    [Fact]
    public void Zoom_StopsAtTheMaximumDistanceHowMuchEverIsQueued()
    {
        var cam = ZoomRig(5f);

        cam.RequestZoom(-100f);
        for (int i = 0; i < 900; i++)
            cam.Tick(1f / 60f, 0f);

        float distance = (cam.Position - cam.GetLookTargetPos()).Length;
        Assert.True(distance <= CameraVehicleSim.MaxZoomDistance + 1.5f,
            $"expected to stop near the 25 m ceiling, got {distance}");
    }

    [Fact]
    public void VetoForward_PointsAtTheLookTarget()
    {
        var cam = MakeCamera();
        cam.TargetPosition = Vec3.Zero;
        cam.EyeTargetLocalPos = Vec3.Zero;
        cam.Position = new Vec3(0f, 0f, -5f);

        cam.VetoForward(out Vec3 forward);

        Assert.Equal(1f, forward.Z, 4);
        Assert.Equal(1f, forward.Length, 4);
    }

    [Fact]
    public void VetoUpAlignment_IsWorldUpOrthogonalisedAgainstTheForward()
    {
        var cam = MakeCamera();
        cam.TargetPosition = Vec3.Zero;
        cam.EyeTargetLocalPos = Vec3.Zero;
        cam.Position = new Vec3(0f, 5f, -5f);

        cam.VetoForward(out Vec3 forward);
        cam.VetoUpAlignment(out Vec3 up);

        Assert.Equal(0f, Vec3.Dot(forward, up), 4);
        Assert.Equal(1f, up.Length, 4);
        Assert.True(up.Y > 0f, "up should still point broadly upward");
    }

    [Fact]
    public void VetoUpAlignment_HandlesLookingStraightDown()
    {
        var cam = MakeCamera();
        cam.TargetPosition = Vec3.Zero;
        cam.EyeTargetLocalPos = Vec3.Zero;
        cam.Position = new Vec3(0f, 5f, 0f);

        cam.VetoForward(out Vec3 forward);
        cam.VetoUpAlignment(out Vec3 up);

        Assert.Equal(1f, up.Length, 4);
        Assert.Equal(0f, Vec3.Dot(forward, up), 4);
    }

    [Fact]
    public void VetoForward_BlendsTowardTheCharacterFacingWhenCrowded()
    {
        var cam = MakeCamera();
        cam.TargetPosition = Vec3.Zero;
        cam.EyeTargetLocalPos = Vec3.Zero;
        cam.TargetRotation = Quat.Identity;

        cam.Position = new Vec3(0f, 0f, -5f);
        cam.VetoForward(out Vec3 far);
        Assert.Equal(1f, far.Z, 4);

        cam.Position = new Vec3(0.3f, 0f, 0f);
        cam.VetoForward(out Vec3 crowded);
        Assert.Equal(1f, crowded.Z, 3);
        Assert.Equal(0f, crowded.X, 3);
    }

    [Fact]
    public void VetoForward_IsUnblendedBeyondThePointNineThreshold()
    {
        var cam = MakeCamera();
        cam.TargetPosition = Vec3.Zero;
        cam.EyeTargetLocalPos = Vec3.Zero;
        cam.TargetRotation = Quat.Identity;
        cam.Position = new Vec3(1f, 0f, 0f);

        cam.VetoForward(out Vec3 forward);

        Assert.Equal(-1f, forward.X, 4);
    }

    [Fact]
    public void CamArrive_HaltsWhenTheStepJumpsTenfold()
    {
        var cam = MakeCamera();
        cam.TargetPosition = Vec3.Zero;
        cam.EyeTargetLocalPos = Vec3.Zero;
        cam.HasSurface = false;
        cam.Position = new Vec3(0f, 0f, -1f);

        for (int i = 0; i < 20; i++)
            cam.Run(1f / 300f);

        Vec3 before = cam.Position;
        cam.Run(0.5f);

        Assert.Equal(before.X, cam.Position.X, 4);
        Assert.Equal(before.Z, cam.Position.Z, 4);
    }

    [Fact]
    public void CamArrive_DoesNotHaltOnAnOrdinaryFrameRateDrop()
    {
        var cam = MakeCamera();
        cam.TargetPosition = Vec3.Zero;
        cam.EyeTargetLocalPos = Vec3.Zero;
        cam.HasSurface = false;
        cam.Position = new Vec3(0f, 0f, -1f);

        for (int i = 0; i < 20; i++)
            cam.Tick(1f / 60f, 0f);

        Vec3 before = cam.Position;
        cam.Tick(0.2f, 0f);

        Assert.NotEqual(before.Z, cam.Position.Z, 4);
    }

    [Fact]
    public void Camera_SettlesAtItsPreferredOffsetBehindTheCharacter()
    {
        var cam = MakeCamera();
        cam.TargetPosition = Vec3.Zero;
        cam.EyeTargetLocalPos = new Vec3(0f, 1.8f, 0f);
        cam.HasSurface = false;
        cam.Position = new Vec3(0f, 1.8f, 0f);

        for (int i = 0; i < 300; i++)
            cam.Tick(1f / 60f, 0f);

        Vec3 look = cam.GetLookTargetPos();
        Vec3 offset = cam.Position - look;

        Assert.True(offset.Z < -3f, $"expected to settle behind the head, offset was {offset}");
        Assert.True(offset.Y > 0.5f, $"and above it, offset was {offset}");
        Assert.True(cam.Speed < 1f, $"and to have settled, speed was {cam.Speed}");
        Assert.Equal(cam.FollowDistance, offset.Length, 1);
    }

    static float WalkAndMeasure(float walkSpeed, float frameTime, float seconds)
    {
        var cam = MakeCamera();
        cam.EyeTargetLocalPos = new Vec3(0f, 1.8f, 0f);
        cam.HasSurface = false;
        cam.TargetPosition = Vec3.Zero;
        cam.Position = new Vec3(0f, 1.8f, -4.5f);

        int frames = (int)MathF.Round(seconds / frameTime);
        for (int i = 0; i < frames; i++)
        {
            cam.TargetPosition += new Vec3(0f, 0f, walkSpeed * frameTime);
            cam.Tick(frameTime, walkSpeed);
        }

        return (cam.Position - cam.GetLookTargetPos()).Length;
    }

    [Theory]
    [InlineData(2f)]
    [InlineData(4f)]
    [InlineData(8f)]
    public void Camera_StaysNearItsPreferredDistanceWhileFollowing(float walkSpeed)
    {
        float distance = WalkAndMeasure(walkSpeed, 1f / 60f, 10f);

        Assert.InRange(distance, 4f, 7.5f);
    }

    [Theory]
    [InlineData(2f)]
    [InlineData(4f)]
    [InlineData(8f)]
    public void Camera_HoldsASteadyDistanceWhileWalking(float walkSpeed)
    {
        var cam = MakeCamera();
        cam.EyeTargetLocalPos = new Vec3(0f, 1.8f, 0f);
        cam.HasSurface = false;
        cam.TargetPosition = Vec3.Zero;
        cam.Position = new Vec3(0f, 1.8f, -4.743f);

        float min = float.MaxValue, max = float.MinValue, previous = 0f, worstStep = 0f;

        for (int i = 0; i < 900; i++)
        {
            cam.TargetPosition += new Vec3(0f, 0f, walkSpeed / 60f);
            cam.Tick(1f / 60f, 5f);

            float d = (cam.Position - cam.GetLookTargetPos()).Length;
            if (i > 180)
            {
                if (d < min) min = d;
                if (d > max) max = d;
                worstStep = MathF.Max(worstStep, MathF.Abs(d - previous));
            }
            previous = d;
        }

        Assert.True(max - min < 0.05f, $"distance oscillated between {min} and {max}");
        Assert.True(worstStep < 0.01f, $"distance jumped {worstStep} m in one frame");
    }

    [Fact]
    public void Camera_TrailsFurtherTheFasterTheCharacterMoves()
    {
        float slow = WalkAndMeasure(2f, 1f / 60f, 10f);
        float fast = WalkAndMeasure(8f, 1f / 60f, 10f);

        Assert.True(fast > slow, $"expected more trail at speed: {slow} then {fast}");
        Assert.True(fast - slow < 2.5f, $"but not much more: {slow} then {fast}");
    }

    [Fact]
    public void Camera_FollowDistanceIsStableAcrossFrameRates()
    {
        float at30 = WalkAndMeasure(4f, 1f / 30f, 10f);
        float at60 = WalkAndMeasure(4f, 1f / 60f, 10f);
        float at144 = WalkAndMeasure(4f, 1f / 144f, 10f);

        Assert.True(MathF.Abs(at144 - at30) < 0.5f,
            $"30fps {at30}, 60fps {at60}, 144fps {at144}");
    }
}
