using LostEden.Vehicles;
using LostEden.Vehicles.Surfaces;
using Xunit;

namespace LostEden.Vehicle.Tests
{
    public class DriveHaltTests
    {
        sealed class Flat : ITileHeightSource
        {
            public int Width => 512;
            public int Height => 512;
            public float TileSize => 1f;
            public float SampleHeight(int x, int z) => 0f;
        }

        static CharVehicleSim Walker()
        {
            var sim = new CharVehicleSim
            {
                Mass = 50f,
                MaxForce = 10f,
                MaxVel = 1f,
                NearProbeOffset = 0.5f,
                SlowingDistance = 1.5f,
                MovementState = 3,
                RunSpeedStat = 275f,
                Surface = new TilemapSurface(new Flat()),
                Position = new Vec3(20f, 0.01f, 20f),
            };
            sim.EnableFalling();
            sim.DisableSurfaceHug();
            sim.UpdateMotionConstraints();
            return sim;
        }

        [Fact]
        public void WithoutAHaltAReleasedDriveWouldCoastForever()
        {
            CharVehicleSim sim = Walker();
            sim.SetForwardDrive(1f);
            for (int i = 0; i < 60; i++)
                sim.Run(1f / 60f);

            Assert.True(sim.Speed > 5f);

            sim.SetForwardDrive(0f);
            Vec3 a = sim.Position;
            for (int i = 0; i < 120; i++)
                sim.Run(1f / 60f);
            Vec3 b = sim.Position;

            Assert.True((b - a).Length > 5f,
                $"expected coasting without a halt, moved {(b - a).Length}");
        }

        [Fact]
        public void HaltingOnReleaseStopsTheBodyDead()
        {
            CharVehicleSim sim = Walker();
            sim.SetForwardDrive(1f);
            for (int i = 0; i < 60; i++)
                sim.Run(1f / 60f);

            sim.SetForwardDrive(0f);
            sim.Halt();

            Vec3 a = sim.Position;
            for (int i = 0; i < 120; i++)
                sim.Run(1f / 60f);
            Vec3 b = sim.Position;

            Assert.Equal(0f, sim.Speed, 4);
            Assert.True((b - a).Length < 0.01f,
                $"expected to stop dead, drifted {(b - a).Length}");
        }

        [Fact]
        public void HaltingOnAReversalMakesItStartFromRest()
        {
            CharVehicleSim sim = Walker();
            sim.SetForwardDrive(1f);
            for (int i = 0; i < 60; i++)
                sim.Run(1f / 60f);

            Assert.True(sim.Speed > 5f);

            sim.SetForwardDrive(-1f);
            sim.Halt();

            Assert.Equal(0f, sim.Speed, 4);
        }

        static void Drive(CharVehicleSim sim, float drive)
        {
            if (drive == sim.ForwardDrive)
                return;

            if (drive > 0f)
            {
                sim.SetDirection(1);
                sim.SetForwardDrive(drive);
            }
            else if (drive < 0f)
            {
                sim.SetForwardDrive(drive);
                sim.SetDirection(-1);
            }
            else
            {
                sim.SetForwardDrive(0f);
                sim.Halt();
                sim.SetDirection(1);
            }
        }

        static CharVehicleSim FacingPositiveX()
        {
            CharVehicleSim sim = Walker();
            sim.UseSurfaceNormal();
            sim.BodyRotation = Quat.FromAxisAngle(Vec3.ReferenceUp, (float)(System.Math.PI / 2.0));
            for (int i = 0; i < 20; i++)
                sim.Run(1f / 60f);
            return sim;
        }

        [Fact]
        public void ABackpedallingBodyActuallyTravelsBackwards()
        {
            CharVehicleSim sim = FacingPositiveX();
            float startX = sim.Position.X;

            Drive(sim, -1f);
            for (int i = 0; i < 90; i++)
                sim.Run(1f / 60f);

            Assert.True(sim.Position.X < startX - 5f,
                $"backpedalled only {startX - sim.Position.X} m (x {startX} -> {sim.Position.X})");
        }

        [Fact]
        public void ABackpedallingBodyKeepsFacingForwards()
        {
            CharVehicleSim sim = FacingPositiveX();

            Drive(sim, -1f);
            for (int i = 0; i < 90; i++)
                sim.Run(1f / 60f);

            Vec3 facing = sim.BodyRotation * Vec3.ReferenceForward;
            Assert.Equal(1f, facing.X, 3);
            Assert.Equal(0f, facing.Z, 3);

            Vec3 travelling = sim.SavedRotation * Vec3.ReferenceForward;
            Assert.Equal(-1f, travelling.X, 3);
        }

        [Fact]
        public void BackpedallingDoesNotOscillate()
        {
            CharVehicleSim sim = FacingPositiveX();
            Drive(sim, -1f);

            for (int i = 0; i < 90; i++)
            {
                float before = sim.Position.X;
                sim.Run(1f / 60f);
                Assert.True(sim.Position.X <= before,
                    $"frame {i} moved forwards: {before} -> {sim.Position.X}");
            }
        }

        [Fact]
        public void ReversingFromARunStartsFromRest()
        {
            CharVehicleSim sim = FacingPositiveX();
            Drive(sim, 1f);
            for (int i = 0; i < 60; i++)
                sim.Run(1f / 60f);
            Assert.True(sim.Speed > 5f);

            Drive(sim, -1f);
            Assert.Equal(0f, sim.Speed, 5);
            Assert.True(sim.Velocity.IsZero);
        }

        [Fact]
        public void ReleasingBackwardRestoresTheForwardDirection()
        {
            CharVehicleSim sim = FacingPositiveX();
            Drive(sim, -1f);
            for (int i = 0; i < 30; i++)
                sim.Run(1f / 60f);
            Assert.Equal(-1, sim.Direction);

            Drive(sim, 0f);
            Assert.Equal(1, sim.Direction);
            Assert.Equal(0f, sim.Speed, 5);
        }

        [Fact]
        public void TurningOnTheSpotIsNotUndoneByTheOrientationUpdate()
        {
            CharVehicleSim sim = FacingPositiveX();
            sim.BodyRotation = Quat.FromAxisAngle(Vec3.ReferenceUp, 0f);

            for (int i = 0; i < 10; i++)
                sim.Run(1f / 60f);

            Vec3 facing = sim.BodyRotation * Vec3.ReferenceForward;
            Assert.Equal(0f, facing.X, 3);
            Assert.Equal(1f, facing.Z, 3);
        }

        [Fact]
        public void SetRelRotReAimsTheVelocityThroughDirection()
        {
            CharVehicleSim sim = FacingPositiveX();
            Drive(sim, -1f);
            for (int i = 0; i < 60; i++)
                sim.Run(1f / 60f);

            float speed = sim.Velocity.Length;
            Assert.True(speed > 5f);

            sim.SetRelRot(Quat.FromAxisAngle(Vec3.ReferenceUp, 0f));

            Assert.Equal(speed, sim.Velocity.Length, 3);
            Assert.Equal(-speed, sim.Velocity.Z, 3);
            Assert.Equal(0f, sim.Velocity.X, 3);
        }

        [Fact]
        public void TurningWhileRunningActuallyTurnsTheBody()
        {
            CharVehicleSim sim = FacingPositiveX();
            Drive(sim, 1f);
            for (int i = 0; i < 60; i++)
                sim.Run(1f / 60f);
            Assert.True(sim.Speed > 5f);

            sim.SetRelRot(Quat.FromAxisAngle(Vec3.ReferenceUp, 0f));

            float z = sim.Position.Z;
            for (int i = 0; i < 30; i++)
                sim.Run(1f / 60f);

            Vec3 facing = sim.BodyRotation * Vec3.ReferenceForward;
            Assert.Equal(1f, facing.Z, 2);
            Assert.True(sim.Position.Z > z + 1f,
                $"after turning to +Z the body only reached z={sim.Position.Z} from {z}");
        }

        [Fact]
        public void TurningWhileStandingStillAlsoTurnsTheBody()
        {
            CharVehicleSim sim = FacingPositiveX();

            sim.SetRelRot(Quat.FromAxisAngle(Vec3.ReferenceUp, 0f));
            for (int i = 0; i < 10; i++)
                sim.Run(1f / 60f);

            Vec3 facing = sim.BodyRotation * Vec3.ReferenceForward;
            Assert.Equal(1f, facing.Z, 2);
        }

        [Fact]
        public void TurningWhileBackpedallingKeepsTravellingBackwards()
        {
            CharVehicleSim sim = FacingPositiveX();
            Drive(sim, -1f);
            for (int i = 0; i < 60; i++)
                sim.Run(1f / 60f);

            sim.SetRelRot(Quat.FromAxisAngle(Vec3.ReferenceUp, 0f));

            float z = sim.Position.Z;
            for (int i = 0; i < 30; i++)
                sim.Run(1f / 60f);

            Vec3 facing = sim.BodyRotation * Vec3.ReferenceForward;
            Assert.Equal(1f, facing.Z, 2);
            Assert.True(sim.Position.Z < z - 1f,
                $"backpedalling towards +Z facing should move to -Z; z went {z} -> {sim.Position.Z}");
        }
    }
}
