using N3Lite;
using N3Lite.Surfaces;
using Xunit;
using Path = N3Lite.Path;

namespace N3Lite.Tests
{
    public class NpcVehicleSimTests
    {
        sealed class Flat : ITileHeightSource
        {
            public int Width => 512;
            public int Height => 512;
            public float TileSize => 1f;
            public float SampleHeight(int x, int z) => 0f;
        }

        static NpcVehicleSim Npc(Vec3 at)
        {
            var sim = new NpcVehicleSim
            {
                Mass = 50f, MaxForce = 10f, MaxVel = 1f, NearProbeOffset = 0.5f,
                SlowingDistance = 1.5f,
                Surface = new TilemapSurface(new Flat()), Position = at,
            };
            sim.EnableFalling();
            sim.DisableSurfaceHug();
            sim.UseSurfaceNormal();
            sim.UpdateMotionConstraints(6f);
            return sim;
        }

        [Fact]
        public void AddWaypointDiscardsTheY()
        {
            var path = new Path();
            path.AddWaypoint(new Vec3(10f, 77f, 20f));

            Assert.Equal(0f, path.GetWaypoint(0).Y, 5);
        }

        [Fact]
        public void ARepeatedWaypointIsDropped()
        {
            var path = new Path();
            path.AddWaypoint(new Vec3(0f, 0f, 0f));
            path.AddWaypoint(new Vec3(0f, 5f, 0f));

            Assert.Equal(1, path.Size);
        }

        [Fact]
        public void SegmentLengthsAndTotalAreAccumulated()
        {
            var path = new Path();
            path.AddWaypoint(new Vec3(0f, 0f, 0f));
            path.AddWaypoint(new Vec3(3f, 0f, 0f));
            path.AddWaypoint(new Vec3(3f, 0f, 4f));

            Assert.Equal(3, path.Size);
            Assert.Equal(3f, path.GetSegLen(0), 4);
            Assert.Equal(4f, path.GetSegLen(1), 4);
            Assert.Equal(7f, path.TotalLength, 4);
            Assert.Equal(1f, path.GetDir(0).X, 4);
            Assert.Equal(1f, path.GetDir(1).Z, 4);
        }

        [Fact]
        public void DistanceMapsOntoTheRightSegment()
        {
            var path = new Path();
            path.AddWaypoint(new Vec3(0f, 0f, 0f));
            path.AddWaypoint(new Vec3(10f, 0f, 0f));
            path.AddWaypoint(new Vec3(10f, 0f, 10f));

            Assert.Equal(0f, path.MapPathDistanceToPoint(0f).X, 3);
            Assert.Equal(4f, path.MapPathDistanceToPoint(4f).X, 3);

            Vec3 p = path.MapPathDistanceToPoint(13f);
            Assert.Equal(10f, p.X, 3);
            Assert.Equal(3f, p.Z, 3);
        }

        [Fact]
        public void DistancePastTheEndClampsToTheFinalWaypoint()
        {
            var path = new Path();
            path.AddWaypoint(new Vec3(0f, 0f, 0f));
            path.AddWaypoint(new Vec3(10f, 0f, 0f));

            Vec3 p = path.MapPathDistanceToPoint(1000f);
            Assert.Equal(10f, p.X, 3);
            Assert.Equal(0f, p.Z, 3);
        }

        [Fact]
        public void AFreshGuideReportsTheOriginUntilItIsUpdated()
        {
            var path = new Path();
            path.AddWaypoint(new Vec3(5f, 0f, 0f));
            path.AddWaypoint(new Vec3(15f, 0f, 0f));

            var guide = new PathGuide(path, 2f, 1f);
            Assert.Equal(0f, guide.GuidePos.X, 4);

            guide.RestartGuide(path, 2f, 1f);
            Assert.Equal(7f, guide.GuidePos.X, 3);
        }

        [Fact]
        public void TheGuideSlidesAlongThePathAtMaxSpeed()
        {
            var path = new Path();
            path.AddWaypoint(new Vec3(0f, 0f, 0f));
            path.AddWaypoint(new Vec3(100f, 0f, 0f));

            var guide = new PathGuide();
            guide.RestartGuide(path, 6f, 0f);
            Assert.Equal(0f, guide.GuidePos.X, 3);

            for (int i = 0; i < 60; i++)
                guide.UpdateAddTime(1f / 60f);

            Assert.Equal(6f, guide.GuidePos.X, 2);
        }

        [Fact]
        public void TheGuideOnAnEmptyPathDoesNotThrow()
        {
            var guide = new PathGuide();
            guide.UpdateAddTime(0.5f);
            Assert.Equal(0f, guide.GuidePos.X, 4);
        }

        [Fact]
        public void AnNpcHasNoStrafeAndNoTurn()
        {
            NpcVehicleSim sim = Npc(new Vec3(100f, 0.01f, 100f));
            sim.SetStrafe(1f);
            sim.SetTurnRate(3f);

            Vec3 before = sim.Position;
            Quat facing = sim.BodyRotation;
            for (int i = 0; i < 120; i++)
                sim.Run(1f / 60f);

            Assert.Equal(before.X, sim.Position.X, 3);
            Assert.Equal(before.Z, sim.Position.Z, 3);
            Assert.Equal(facing.Y, sim.BodyRotation.Y, 4);
        }

        [Fact]
        public void AnNpcWithNoPathStandsStill()
        {
            NpcVehicleSim sim = Npc(new Vec3(100f, 0.01f, 100f));
            Vec3 before = sim.Position;

            for (int i = 0; i < 120; i++)
                sim.Run(1f / 60f);

            Assert.Equal(before.X, sim.Position.X, 3);
            Assert.Equal(before.Z, sim.Position.Z, 3);
        }

        [Fact]
        public void AnNpcWalksItsPath()
        {
            NpcVehicleSim sim = Npc(new Vec3(100f, 0.01f, 100f));
            sim.Path.AddWaypoint(new Vec3(100f, 0f, 100f));
            sim.Path.AddWaypoint(new Vec3(130f, 0f, 100f));
            sim.RestartPath();

            for (int i = 0; i < 300; i++)
            {
                sim.AdvanceGuide(1f / 60f);
                sim.Run(1f / 60f);
            }

            Assert.True(sim.Position.X > 115f, $"only reached x={sim.Position.X}");
            Assert.Equal(100f, sim.Position.Z, 1);
        }

        [Fact]
        public void AnNpcTurnsACornerOnItsPath()
        {
            NpcVehicleSim sim = Npc(new Vec3(100f, 0.01f, 100f));
            sim.Path.AddWaypoint(new Vec3(100f, 0f, 100f));
            sim.Path.AddWaypoint(new Vec3(120f, 0f, 100f));
            sim.Path.AddWaypoint(new Vec3(120f, 0f, 130f));
            sim.RestartPath();

            for (int i = 0; i < 600; i++)
            {
                sim.AdvanceGuide(1f / 60f);
                sim.Run(1f / 60f);
            }

            Assert.True(sim.Position.Z > 110f, $"did not turn the corner, z={sim.Position.Z}");
            Assert.InRange(sim.Position.X, 115f, 125f);
        }

        [Fact]
        public void AnNpcArrivesAtAFollowTargetAndIgnoresThePath()
        {
            NpcVehicleSim sim = Npc(new Vec3(100f, 0.01f, 100f));
            sim.Path.AddWaypoint(new Vec3(100f, 0f, 100f));
            sim.Path.AddWaypoint(new Vec3(60f, 0f, 100f));
            sim.RestartPath();

            sim.SetFollowPath(new[] { new Vec3(120f, 0f, 100f) });

            for (int i = 0; i < 300; i++)
            {
                sim.AdvanceGuide(1f / 60f);
                sim.Run(1f / 60f);
            }

            Assert.True(sim.Position.X > 110f, $"followed the path instead of the target, x={sim.Position.X}");
        }

        [Fact]
        public void ALockedDriveHaltsAnNpc()
        {
            NpcVehicleSim sim = Npc(new Vec3(100f, 0.01f, 100f));
            sim.Path.AddWaypoint(new Vec3(100f, 0f, 100f));
            sim.Path.AddWaypoint(new Vec3(130f, 0f, 100f));
            sim.RestartPath();

            for (int i = 0; i < 60; i++)
            {
                sim.AdvanceGuide(1f / 60f);
                sim.Run(1f / 60f);
            }
            Assert.True(sim.Speed > 0f);

            sim.DriveLocked = true;
            for (int i = 0; i < 120; i++)
            {
                sim.AdvanceGuide(1f / 60f);
                sim.Run(1f / 60f);
            }

            Assert.Equal(0f, sim.Speed, 3);
        }

        static void Step(NpcVehicleSim sim, int frames)
        {
            for (int i = 0; i < frames; i++)
            {
                sim.AdvanceGuide(1f / 60f);
                sim.Run(1f / 60f);
            }
        }

        [Fact]
        public void AFollowingNpcWalksEachPointAndStopsAboutOneMetreShortOfTheLast()
        {
            NpcVehicleSim sim = Npc(new Vec3(100f, 0.01f, 100f));
            int halts = 0;
            sim.Halted += () => halts++;

            sim.SetFollowPath(new[] { new Vec3(110f, 0f, 100f), new Vec3(110f, 0f, 110f) });
            Step(sim, 600);

            Assert.Empty(sim.FollowQueue);
            Assert.Equal(0f, sim.Speed, 3);
            Assert.Equal(1, halts);

            float dx = 110f - sim.Position.X, dz = 110f - sim.Position.Z;
            float gap = (float)System.Math.Sqrt(dx * dx + dz * dz);
            Assert.InRange(gap, 0.5f, 1.2f);
        }

        [Fact]
        public void SnapToStartPlacesTheBodyOnTheGroundUnderPointZeroAndQueuesTheRest()
        {
            NpcVehicleSim sim = Npc(new Vec3(100f, 0.01f, 100f));

            sim.SetFollowPath(new[] { new Vec3(50f, 1f, 60f), new Vec3(55f, 0f, 60f) }, snapToStart: true);

            Assert.Equal(50f, sim.Position.X, 3);
            Assert.Equal(60f, sim.Position.Z, 3);
            Assert.Equal(0f, sim.Position.Y, 2);                   // dropped onto the flat ground
            Assert.Single(sim.FollowQueue);
            Assert.Equal(55f, sim.FollowQueue[0].X, 3);
        }

        [Fact]
        public void SnapToStartKeepsALoneStartAsTheWaypoint()
        {
            NpcVehicleSim sim = Npc(new Vec3(100f, 0.01f, 100f));

            sim.SetFollowPath(new[] { new Vec3(50f, 0f, 60f) }, snapToStart: true);

            Assert.Single(sim.FollowQueue);
            Assert.Equal(50f, sim.FollowQueue[0].X, 3);
        }

        [Fact]
        public void WithoutSnapEveryPointIsAWaypoint()
        {
            NpcVehicleSim sim = Npc(new Vec3(100f, 0.01f, 100f));

            sim.SetFollowPath(new[] { new Vec3(50f, 0f, 60f), new Vec3(55f, 0f, 60f) });

            Assert.Equal(100f, sim.Position.X, 3);
            Assert.Equal(2, sim.FollowQueue.Count);
        }

        [Fact]
        public void FollowDisabledHoldsTheNpcInPlace()
        {
            NpcVehicleSim sim = Npc(new Vec3(100f, 0.01f, 100f));
            sim.FollowEnabled = false;

            sim.SetFollowPath(new[] { new Vec3(120f, 0f, 100f) });
            Step(sim, 120);

            Assert.Equal(100f, sim.Position.X, 1);
            Assert.Single(sim.FollowQueue);
        }

        [Fact]
        public void AMovingStateWithNothingToFollowAsksForAFullStop()
        {
            var sim = new NpcVehicleSim();
            int stops = 0;
            sim.FullStopRequested += () => stops++;

            sim.AdvanceGuide(1f / 60f);
            Assert.Equal(0, stops);

            sim.ForwardActive = true;
            sim.AdvanceGuide(1f / 60f);
            Assert.Equal(1, stops);

            sim.SetFollowPath(new[] { new Vec3(1f, 0f, 1f) });
            sim.AdvanceGuide(1f / 60f);
            Assert.Equal(1, stops);
        }

        [Fact]
        public void TheFollowQueueEndsAtAZeroPointAndHoldsThirty()
        {
            var sim = new NpcVehicleSim();

            sim.SetFollowPath(new[] { new Vec3(1f, 0f, 1f), Vec3.Zero, new Vec3(2f, 0f, 2f) });
            Assert.Single(sim.FollowQueue);

            var many = new Vec3[40];
            for (int i = 0; i < many.Length; i++)
                many[i] = new Vec3(i + 1f, 0f, 0f);
            sim.SetFollowPath(many);
            Assert.Equal(NpcVehicleSim.FollowQueueCapacity, sim.FollowQueue.Count);
        }

        [Fact]
        public void AnIdleNpcIsNotStepped()
        {
            NpcVehicleSim sim = Npc(new Vec3(100f, 5f, 100f));
            sim.LandNow(5f);                  // on the "ground" at 5 m, nothing to do

            Assert.False(sim.Run(1f / 60f));
            Assert.Equal(5f, sim.Position.Y, 5);

            sim.ForwardActive = true;         // a moving forward state keeps it stepping, so it falls
            for (int i = 0; i < 30; i++)
                sim.Run(1f / 60f);
            Assert.True(sim.Position.Y < 5f, $"was not stepped, y={sim.Position.Y}");
        }

        [Fact]
        public void TheGuideKeepsTheSpeedItWasRestartedWith()
        {
            NpcVehicleSim sim = Npc(new Vec3(100f, 0.01f, 100f));
            sim.Path.AddWaypoint(new Vec3(100f, 0f, 100f));
            sim.Path.AddWaypoint(new Vec3(200f, 0f, 100f));
            sim.RestartPath();
            float speed = sim.Guide.MaxSpeed;

            sim.UpdateMotionConstraints(2f);
            sim.AdvanceGuide(1f);

            Assert.Equal(speed, sim.Guide.MaxSpeed, 5);
        }

        [Fact]
        public void SeededMotionIsTakenWholeAndReadsBackTheSame()
        {
            NpcVehicleSim sim = Npc(new Vec3(100f, 0.01f, 100f));
            var velocity = new Vec3(3f, 0f, 0f);
            var path = new[] { new Vec3(95f, 0f, 100f), new Vec3(140f, 0f, 100f) };

            sim.SetMotionState(velocity, path, 2f);

            Assert.Equal(3f, sim.Speed, 5);                       // no ramp from rest
            Assert.Equal(2f, sim.Guide.Time, 5);
            Assert.Equal(95f + 2f * sim.MaxVel, sim.Guide.GuidePos.X, 3);

            var back = new System.Collections.Generic.List<Vec3>();
            sim.GetMotionState(out Vec3 v, back, out float t);
            Assert.Equal(velocity, v);
            Assert.Equal(2, back.Count);
            Assert.Equal(2f, t, 5);

            var copy = Npc(new Vec3(100f, 0.01f, 100f));
            copy.SetMotionState(v, back, t);
            Assert.Equal(sim.Guide.GuidePos, copy.Guide.GuidePos);
        }

        [Fact]
        public void SeedingWithNoWaypointsZeroesTheGuideTime()
        {
            NpcVehicleSim sim = Npc(new Vec3(100f, 0.01f, 100f));
            sim.SetMotionState(Vec3.Zero, null, 5f);

            Assert.True(sim.Path.Empty);
            Assert.Equal(0f, sim.Guide.Time, 5);
        }

        [Fact]
        public void AHaltNearThePathsEndClearsItAndRaisesHalted()
        {
            NpcVehicleSim sim = Npc(new Vec3(100f, 0.01f, 100f));
            int halts = 0;
            sim.Halted += () => halts++;

            sim.Path.AddWaypoint(new Vec3(100f, 0f, 100f));
            sim.Path.AddWaypoint(new Vec3(112f, 0f, 100f));
            sim.RestartPath();
            Step(sim, 600);

            Assert.True(sim.Path.Empty);
            Assert.Equal(1, halts);
        }

        [Fact]
        public void DirArriveHaltsOnceTheTargetIsBehind()
        {
            NpcVehicleSim sim = Npc(new Vec3(100f, 0.01f, 100f));
            sim.Path.AddWaypoint(new Vec3(100f, 0f, 100f));
            sim.Path.AddWaypoint(new Vec3(112f, 0f, 100f));
            sim.RestartPath();

            for (int i = 0; i < 600; i++)
            {
                sim.AdvanceGuide(1f / 60f);
                sim.Run(1f / 60f);
            }

            Assert.InRange(sim.Position.X, 110f, 114f);
            Assert.Equal(0f, sim.Speed, 2);
        }
    }
}
