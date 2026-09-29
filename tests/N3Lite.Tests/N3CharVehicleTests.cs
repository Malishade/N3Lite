using N3Lite;
using N3Lite.Surfaces;
using Xunit;

namespace N3Lite.Tests
{
    public class N3CharVehicleTests
    {
        sealed class Flat : ITileHeightSource
        {
            public int Width => 512;
            public int Height => 512;
            public float TileSize => 1f;
            public float SampleHeight(int x, int z) => 0f;
        }

        static readonly MovementProfile Running = new MovementProfile
        {
            ForwardSpeed = 6f,
            ReverseSpeed = 3.7f,
            StrafeSpeed = 3f,
            ReverseStrafeSpeed = 1.85f,
            CanDrive = true,
            AcceptsInput = true,
        };

        static N3CharVehicle Body(bool isNpc = false)
        {
            var body = new N3CharVehicle(isNpc)
            {
                Surface = new TilemapSurface(new Flat()),
                Position = new Vec3(100f, 0f, 100f),
            };
            body.SetProfile(Running);
            body.Warp(new Vec3(100f, 0f, 100f), new Vec3(0f, 0f, 1f));
            return body;
        }

        static void Run(N3CharVehicle body, float seconds)
        {
            for (float t = 0f; t < seconds; t += 1f / 60f)
                body.Tick(1f / 60f);
        }

        [Fact]
        public void ForwardMovesAlongTheFacing()
        {
            var body = Body();
            body.SetInputs(MovementFlags.Forward);
            Run(body, 1f);

            Assert.True(body.Position.Z > 101f);
            Assert.Equal(100f, body.Position.X, 2);
            Assert.True(body.IsMoving);
        }

        [Fact]
        public void ReleasingForwardStopsAtOnce()
        {
            var body = Body();
            body.SetInputs(MovementFlags.Forward);
            Run(body, 1f);

            body.SetInputs(MovementFlags.None);
            Vec3 stoppedAt = body.Position;
            Run(body, 0.5f);

            Assert.Equal(0f, body.CurrentSpeed, 3);
            Assert.Equal(stoppedAt.Z, body.Position.Z, 2);
        }

        [Fact]
        public void BackwardReversesTheDirectionAndKeepsTheFacing()
        {
            var body = Body();
            body.SetInputs(MovementFlags.Backward);
            Run(body, 1f);

            Assert.Equal(-1, body.Vehicle.Direction);
            Assert.Equal(3.7f, body.MaxVel, 5);
            Assert.True(body.Position.Z < 99.5f);
            Assert.True(body.Vehicle.GetBodyForward().Z > 0.9f);
        }

        [Theory]
        [InlineData(MovementFlags.Forward, 6f)]
        [InlineData(MovementFlags.Backward, 3.7f)]
        public void AcceleratesLinearlyFromRestToFullSpeedInHalfASecond(MovementFlags flags, float top)
        {
            // The drive force is maxForce = 2 * speed * mass along a unit forward, so the speed climbs
            // by the same step every frame. A cached forward that was not unit length scaled the force
            // by the speed and made the first metres a slow exponential crawl.
            var body = Body();
            body.SetInputs(flags);

            for (int frame = 1; frame <= 30; frame++)
            {
                body.Tick(1f / 60f);
                Assert.Equal(top * frame / 30f, body.CurrentSpeed, 3);
                Assert.Equal(1f, body.Vehicle.GetBodyForward().Length, 4);
            }
        }

        [Fact]
        public void TurnUsesTheStandingOrMovingRate()
        {
            var body = Body();
            body.SetInputs(MovementFlags.TurnRight);
            Assert.Equal(body.TurnRateStopped, body.Vehicle.TurnRate, 5);

            body.SetInputs(MovementFlags.TurnRight | MovementFlags.Forward);
            Assert.Equal(body.TurnRateMoving, body.Vehicle.TurnRate, 5);
        }

        [Fact]
        public void SittingRefusesInputAndJump()
        {
            var body = Body();
            body.SetProfile(new MovementProfile { AcceptsInput = false });

            body.SetInputs(MovementFlags.Forward | MovementFlags.Jump);
            Run(body, 0.5f);

            Assert.Equal(MovementFlags.None, body.Flags);
            Assert.False(body.TryStartJump());
            Assert.Equal(100f, body.Position.Z, 2);
        }

        [Fact]
        public void TheProfileSetsTheSpeeds()
        {
            var body = Body();
            Assert.Equal(6f, body.MaxVel, 5);

            body.SetInputs(MovementFlags.StrafeLeft);
            Assert.Equal(-3f, body.Vehicle.Strafe, 5);

            body.SetInputs(MovementFlags.StrafeLeft | MovementFlags.Backward);
            Assert.Equal(-1.85f, body.Vehicle.Strafe, 5);

            body.SetInputs(MovementFlags.Forward);
            Assert.Equal(6f, body.MaxVel, 5);
        }

        [Fact]
        public void ALockedDriveStillStrafes()
        {
            var body = Body();
            MovementProfile locked = Running;
            locked.CanDrive = false;
            body.SetProfile(locked);

            body.SetInputs(MovementFlags.Forward | MovementFlags.StrafeRight);
            Run(body, 1f);

            Assert.Equal(100f, body.Position.Z, 1);
            Assert.True(body.Position.X > 101f);
        }

        [Fact]
        public void FlyingTurnsGravityOffAndLandingBackOn()
        {
            var body = Body();
            MovementProfile flying = Running;
            flying.Flying = true;

            body.SetProfile(flying);
            Assert.False(body.Vehicle.FallingEnabled);

            body.SetProfile(Running);
            Assert.True(body.Vehicle.FallingEnabled);
        }

        [Fact]
        public void AnInputlessProfileClearsThePathOnlyOnEntry()
        {
            var body = Body(isNpc: true);
            int cleared = 0;
            body.PathCleared += () => cleared++;

            body.SetProfile(new MovementProfile { AcceptsInput = false });
            body.SetProfile(new MovementProfile { AcceptsInput = false });

            Assert.Equal(1, cleared);
        }

        [Fact]
        public void JumpRaisesStartAndLandAndClearsTheFlag()
        {
            var body = Body();
            body.JumpHeight = 2f;
            int started = 0, landed = 0;
            body.JumpStarted += () => started++;
            body.JumpLanded += () => landed++;

            body.SetInputs(MovementFlags.Jump);
            Run(body, 3f);

            Assert.Equal(1, started);
            Assert.Equal(1, landed);
            Assert.Equal(MovementFlags.None, body.Flags & MovementFlags.Jump);
        }

        [Fact]
        public void PlayerBodyRefusesAPath()
        {
            var body = Body();
            Assert.False(body.SetPath(new[] { new Vec3(110f, 0f, 100f) }));
        }

        [Fact]
        public void NpcBodyIgnoresInputAndFollowsAPath()
        {
            var body = Body(isNpc: true);
            Assert.False(body.SetInputs(MovementFlags.Forward));

            Assert.True(body.SetPath(new[] { new Vec3(100f, 0f, 100f), new Vec3(110f, 0f, 100f) }));
            Run(body, 3f);

            Assert.True(body.Position.X > 102f);
            Assert.Equal(100f, body.Position.Z, 1);
        }

        [Fact]
        public void ClearPathRaisesTheEventAndEmptiesTheNpcPath()
        {
            var body = Body(isNpc: true);
            body.SetPath(new[] { new Vec3(100f, 0f, 100f), new Vec3(110f, 0f, 100f) });
            int cleared = 0;
            body.PathCleared += () => cleared++;

            body.ClearPath();

            Assert.Equal(1, cleared);
            Assert.Equal(0, body.Npc.Path.Size);
        }

        [Fact]
        public void SwitchingKindKeepsPositionAndSurface()
        {
            var body = Body();
            ISurface surface = body.Surface;
            body.Position = new Vec3(120f, 0f, 130f);

            body.SelectVehicleKind(true);

            Assert.True(body.IsNpcVehicle);
            Assert.NotNull(body.Npc);
            Assert.Same(surface, body.Surface);
            Assert.Equal(120f, body.Position.X, 5);
            Assert.Equal(130f, body.Position.Z, 5);
        }
    }
}
