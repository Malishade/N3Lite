using System;
using N3Lite;
using Xunit;

namespace N3Lite.Tests
{
    public class CharVehicleSimTests
    {
        static CharVehicleSim Player() => new CharVehicleSim
        {
            Mass = 50f,
            MaxForce = 10f,
            MaxVel = 1f,
            NearProbeOffset = 0.5f,
            SlowingDistance = 1.5f,
            FallingEnabled = true,
            SurfaceHug = false,
        };

        [Theory]
        [InlineData(6f, 6f)]
        [InlineData(1.5f, 1.5f)]
        [InlineData(0.005f, 0.1f)]
        public void MaxVelIsTheGivenSpeed(float speed, float expected)
        {
            CharVehicleSim sim = Player();
            sim.UpdateMotionConstraints(speed);

            Assert.Equal(expected, sim.MaxVel, 3);
        }

        [Fact]
        public void MaxForceIsTwiceMassTimesSpeed()
        {
            CharVehicleSim sim = Player();
            sim.UpdateMotionConstraints(6f);

            Assert.Equal(6f, sim.MaxVel, 3);
            Assert.Equal(2f * 50f * 6f, sim.MaxForce, 2);
        }

        [Fact]
        public void TheBrakeDistanceIsAQuarterOfTheMaxSpeed()
        {
            foreach (float speed in new[] { 1f, 5f, 6f, 13f })
            {
                CharVehicleSim sim = Player();
                sim.UpdateMotionConstraints(speed);
                Assert.Equal(sim.MaxVel / 4f, sim.SlowingDistance, 3);
            }
        }

        [Fact]
        public void MaxForceIsCappedAtTenThousand()
        {
            CharVehicleSim sim = Player();
            sim.Mass = 500f;
            sim.UpdateMotionConstraints(13f);

            Assert.Equal(10000f, sim.MaxForce, 1);
        }

        [Fact]
        public void MassIsFlooredByItsSetterSoTheZeroGuardIsUnreachable()
        {
            CharVehicleSim sim = Player();

            sim.Mass = 0f;
            Assert.Equal(0.1f, sim.Mass, 4);

            sim.UpdateMotionConstraints(6f);
            Assert.Equal(0.1f, sim.Mass, 4);
        }

        [Fact]
        public void SetStrafeKeepsOnlyTheSignOfItsArgument()
        {
            CharVehicleSim sim = Player();
            sim.StrafeSpeed = 3f;

            sim.SetStrafe(0.001f);
            Assert.Equal(3f, sim.Strafe, 3);

            sim.SetStrafe(1000f);
            Assert.Equal(3f, sim.Strafe, 3);

            sim.SetStrafe(-0.001f);
            Assert.Equal(-3f, sim.Strafe, 3);

            sim.SetStrafe(0f);
            Assert.Equal(0f, sim.Strafe, 3);
        }

        [Fact]
        public void ALockedDriveRefusesLongitudinalSteering()
        {
            CharVehicleSim sim = Player();
            sim.DriveLocked = true;
            sim.SetForwardDrive(1f);
            sim.Run(1f / 60f);

            Assert.Equal(0f, sim.Speed, 4);
        }

        [Fact]
        public void APositiveDriveGoesForwardAndANegativeOneReverses()
        {
            CharVehicleSim forward = Player();
            forward.UpdateMotionConstraints(6f);
            forward.SetForwardDrive(1f);
            for (int i = 0; i < 60; i++)
                forward.Run(1f / 60f);

            CharVehicleSim back = Player();
            back.UpdateMotionConstraints(6f);
            back.SetForwardDrive(-1f);
            for (int i = 0; i < 60; i++)
                back.Run(1f / 60f);

            Assert.True(forward.Position.Z > 0.5f, $"forward went to {forward.Position}");
            Assert.True(back.Position.Z < -0.5f, $"reverse went to {back.Position}");
        }

        [Fact]
        public void AZeroDriveProducesNoLongitudinalSteering()
        {
            CharVehicleSim sim = Player();
            sim.UpdateMotionConstraints(6f);
            sim.SetForwardDrive(0f);

            for (int i = 0; i < 60; i++)
                sim.Run(1f / 60f);

            Assert.Equal(0f, sim.Speed, 4);
        }

        [Fact]
        public void StrafeMovesAlongBodyRight()
        {
            CharVehicleSim sim = Player();
            sim.UpdateMotionConstraints(6f);
            sim.StrafeSpeed = 3f;
            sim.SetStrafe(1f);

            sim.Run(1f);

            Assert.True(sim.Position.X > 2.5f, $"strafed to {sim.Position}");
            Assert.Equal(0f, sim.Position.Z, 3);
        }

        [Fact]
        public void TheVerticalAxisIsWorldUpNotBodyUp()
        {
            CharVehicleSim sim = Player();
            sim.UpdateMotionConstraints(6f);
            sim.DisableFalling();
            sim.BodyRotation = Quat.FromAxisAngle(new Vec3(0f, 0f, 1f), (float)(Math.PI / 4.0));
            sim.SetVertical(2f);

            sim.Run(1f);

            Assert.True(sim.Position.Y > 1.5f, $"expected to rise, got {sim.Position}");
            Assert.Equal(0f, sim.Position.X, 3);
            Assert.Equal(0f, sim.Position.Z, 3);
        }

        [Fact]
        public void NoStrafeAndNoVerticalMeansNoLateralSteering()
        {
            CharVehicleSim sim = Player();
            sim.UpdateMotionConstraints(6f);

            sim.Run(1f / 60f);

            Assert.Equal(0f, sim.Position.X, 5);
            Assert.Equal(0f, sim.Position.Z, 5);
        }

        [Fact]
        public void TurningWhileStandingStillRotatesTheBody()
        {
            CharVehicleSim sim = Player();
            sim.UpdateMotionConstraints(6f);
            sim.SetTurnRate(1f);

            sim.Run(0.5f);

            Vec3 forward = sim.GetBodyForward();
            Assert.Equal((float)Math.Sin(0.5), forward.X, 2);
            Assert.Equal((float)Math.Cos(0.5), forward.Z, 2);
        }

        [Fact]
        public void TurningWhileMovingRotatesTheVelocityNotTheBody()
        {
            CharVehicleSim sim = Player();
            sim.UpdateMotionConstraints(6f);
            sim.SetForwardDrive(1f);
            for (int i = 0; i < 60; i++)
                sim.Run(1f / 60f);

            Quat before = sim.BodyRotation;
            sim.SetTurnRate(1f);
            sim.Run(0.25f);

            Assert.Equal(before.X, sim.BodyRotation.X, 4);
            Assert.Equal(before.Y, sim.BodyRotation.Y, 4);
            Assert.Equal(before.Z, sim.BodyRotation.Z, 4);
            Assert.Equal(before.W, sim.BodyRotation.W, 4);

            Assert.True(Math.Abs(sim.Velocity.X) > 0.01f, $"velocity did not turn: {sim.Velocity}");
        }

        [Fact]
        public void AZeroTurnRateProducesNoTurnSteering()
        {
            CharVehicleSim sim = Player();
            sim.UpdateMotionConstraints(6f);
            sim.SetTurnRate(0f);

            sim.Run(0.5f);

            Assert.Equal(0f, sim.GetBodyForward().X, 5);
            Assert.Equal(1f, sim.GetBodyForward().Z, 5);
        }

        [Fact]
        public void BodyRightIsTheRotationAppliedToXUnit()
        {
            CharVehicleSim sim = Player();
            Assert.Equal(1f, sim.CalcBodyRight().X, 4);

            sim.BodyRotation = Quat.FromAxisAngle(Vec3.ReferenceUp, (float)(Math.PI / 2.0));
            Vec3 right = sim.CalcBodyRight();
            Assert.Equal(-1f, right.Z, 3);
        }
    }
}
