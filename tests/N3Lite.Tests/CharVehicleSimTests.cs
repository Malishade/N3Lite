using System;
using LostEden.Vehicles;
using Xunit;

namespace LostEden.Vehicle.Tests
{
    public class CharVehicleSimTests
    {
        static CharVehicleSim Player(int state = 3, float runStat = 0f) => new CharVehicleSim
        {
            Mass = 50f,
            MaxForce = 10f,
            MaxVel = 1f,
            NearProbeOffset = 0.5f,
            SlowingDistance = 1.5f,
            FallingEnabled = true,
            SurfaceHug = false,
            MovementState = state,
            RunSpeedStat = runStat,
        };

        [Theory]
        [InlineData(3, 1, 0f, 5f)]
        [InlineData(3, 1, 275f, 6f)]
        [InlineData(3, 1, 2200f, 13f)]
        [InlineData(3, 1, 10000f, 13f)]
        [InlineData(3, 2, 0f, 3f)]
        [InlineData(3, 2, 275f, 3.7f)]
        [InlineData(4, 1, 0f, 3f)]
        [InlineData(4, 1, 440f, 4f)]
        [InlineData(7, 1, 0f, 7f)]
        [InlineData(7, 1, 275f, 8f)]
        [InlineData(2, 1, 5000f, 1.5f)]
        [InlineData(5, 1, 5000f, 1f)]
        public void MaxVelFollowsTheStateCurve(int state, int direction, float stat, float expected)
        {
            CharVehicleSim sim = Player(state, stat);
            sim.CurveDirection = direction;

            sim.UpdateMotionConstraints();

            Assert.Equal(expected, sim.MaxVel, 3);
        }

        [Fact]
        public void MaxForceIsTwiceMassTimesSpeed()
        {
            CharVehicleSim sim = Player(3, 275f);
            sim.UpdateMotionConstraints();

            Assert.Equal(6f, sim.MaxVel, 3);
            Assert.Equal(2f * 50f * 6f, sim.MaxForce, 2);
        }

        [Fact]
        public void TheBrakeDistanceIsAQuarterOfTheMaxSpeed()
        {
            foreach (float stat in new[] { 0f, 275f, 1000f, 5000f })
            {
                CharVehicleSim sim = Player(3, stat);
                sim.UpdateMotionConstraints();
                Assert.Equal(sim.MaxVel / 4f, sim.SlowingDistance, 3);
            }
        }

        [Fact]
        public void MaxForceIsCappedAtTenThousand()
        {
            CharVehicleSim sim = Player(3, 5000f);
            sim.Mass = 500f;
            sim.UpdateMotionConstraints();

            Assert.Equal(10000f, sim.MaxForce, 1);
        }

        [Fact]
        public void MassIsFlooredByItsSetterSoTheZeroGuardIsUnreachable()
        {
            CharVehicleSim sim = Player();

            sim.Mass = 0f;
            Assert.Equal(0.1f, sim.Mass, 4);

            sim.UpdateMotionConstraints();
            Assert.Equal(0.1f, sim.Mass, 4);
        }

        [Fact]
        public void FlyTurnsGravityOff()
        {
            CharVehicleSim sim = Player(7);
            Assert.True(sim.FallingEnabled);

            sim.UpdateMotionConstraints();

            Assert.False(sim.FallingEnabled);
        }

        [Theory]
        [InlineData(3, 0f, 2.5f)]
        [InlineData(3, 275f, 3f)]
        [InlineData(3, 10000f, 6.5f)]
        [InlineData(7, 0f, 3.5f)]
        [InlineData(2, 0f, 1.5f)]
        public void StrafeSpeedIsHalfTheForwardCurve(int state, float stat, float expected)
        {
            CharVehicleSim sim = Player(state, stat);
            Assert.Equal(expected, sim.StrafeSpeed(state), 3);
        }

        [Fact]
        public void SetStrafeKeepsOnlyTheSignOfItsArgument()
        {
            CharVehicleSim sim = Player(3, 275f);

            sim.SetStrafe(0.001f);
            Assert.Equal(3f, sim.Strafe, 3);

            sim.SetStrafe(1000f);
            Assert.Equal(3f, sim.Strafe, 3);

            sim.SetStrafe(-0.001f);
            Assert.Equal(-3f, sim.Strafe, 3);

            sim.SetStrafe(0f);
            Assert.Equal(0f, sim.Strafe, 3);
        }

        [Theory]
        [InlineData(1)]
        [InlineData(8)]
        [InlineData(9)]
        public void StatesOneEightAndNineRefuseLongitudinalSteering(int state)
        {
            CharVehicleSim sim = Player(state);
            sim.SetForwardDrive(1f);
            sim.Run(1f / 60f);

            Assert.Equal(0f, sim.Speed, 4);
        }

        [Fact]
        public void APositiveDriveGoesForwardAndANegativeOneReverses()
        {
            CharVehicleSim forward = Player(3, 275f);
            forward.UpdateMotionConstraints();
            forward.SetForwardDrive(1f);
            for (int i = 0; i < 60; i++)
                forward.Run(1f / 60f);

            CharVehicleSim back = Player(3, 275f);
            back.UpdateMotionConstraints();
            back.SetForwardDrive(-1f);
            for (int i = 0; i < 60; i++)
                back.Run(1f / 60f);

            Assert.True(forward.Position.Z > 0.5f, $"forward went to {forward.Position}");
            Assert.True(back.Position.Z < -0.5f, $"reverse went to {back.Position}");
        }

        [Fact]
        public void AZeroDriveProducesNoLongitudinalSteering()
        {
            CharVehicleSim sim = Player(3, 275f);
            sim.UpdateMotionConstraints();
            sim.SetForwardDrive(0f);

            for (int i = 0; i < 60; i++)
                sim.Run(1f / 60f);

            Assert.Equal(0f, sim.Speed, 4);
        }

        [Fact]
        public void StrafeMovesAlongBodyRight()
        {
            CharVehicleSim sim = Player(3, 275f);
            sim.UpdateMotionConstraints();
            sim.SetStrafe(1f);

            sim.Run(1f);

            Assert.True(sim.Position.X > 2.5f, $"strafed to {sim.Position}");
            Assert.Equal(0f, sim.Position.Z, 3);
        }

        [Fact]
        public void TheVerticalAxisIsWorldUpNotBodyUp()
        {
            CharVehicleSim sim = Player(3, 275f);
            sim.UpdateMotionConstraints();
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
            CharVehicleSim sim = Player(3, 275f);
            sim.UpdateMotionConstraints();

            sim.Run(1f / 60f);

            Assert.Equal(0f, sim.Position.X, 5);
            Assert.Equal(0f, sim.Position.Z, 5);
        }

        [Fact]
        public void TurningWhileStandingStillRotatesTheBody()
        {
            CharVehicleSim sim = Player(3, 275f);
            sim.UpdateMotionConstraints();
            sim.SetTurnRate(1f);

            sim.Run(0.5f);

            Vec3 forward = sim.GetBodyForward();
            Assert.Equal((float)Math.Sin(0.5), forward.X, 2);
            Assert.Equal((float)Math.Cos(0.5), forward.Z, 2);
        }

        [Fact]
        public void TurningWhileMovingRotatesTheVelocityNotTheBody()
        {
            CharVehicleSim sim = Player(3, 275f);
            sim.UpdateMotionConstraints();
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
            CharVehicleSim sim = Player(3, 275f);
            sim.UpdateMotionConstraints();
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
