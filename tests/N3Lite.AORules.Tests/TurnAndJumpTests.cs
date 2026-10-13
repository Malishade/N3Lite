using N3Lite.Surfaces;
using Xunit;

namespace N3Lite.AORules.Tests
{
    public class TurnAndJumpTests
    {
        const int ForwardStart = 1, ForwardStop = 2, TurnRightStart = 9, TurnLeftStart = 0xc;
        const int TurnLeftMouse = 0xd, TurnLeftStop = 0xe, JumpStart = 0xf, SwitchToFly = 0x1d;

        sealed class Flat : ITileHeightSource
        {
            public int Width => 512;
            public int Height => 512;
            public float TileSize => 1f;
            public float SampleHeight(int x, int z) => 0f;
        }

        static CharMovement Body(int turnSpeed = 0)
        {
            var movement = new CharMovement();
            movement.SetStats(new MovementStats { Features = 0x4, TurnSpeed = turnSpeed, Strength = 100, Agility = 100 });
            movement.Core.Surface = new TilemapSurface(new Flat());
            movement.Core.Warp(new Vec3(100f, 0f, 100f), new Vec3(0f, 0f, 1f));
            Run(movement, 1f);
            return movement;
        }

        static void Run(CharMovement movement, float seconds)
        {
            for (int i = 0; i < (int)(seconds * 30f); i++)
                movement.Tick(1f / 30f);
        }

        [Theory]
        [InlineData(0, 3.5f, 1.5f)]
        [InlineData(5500, 0.5f, 0.25f)]
        public void AKeyboardTurnIsRatedFromStat267(int stat, float standing, float driving)
        {
            var movement = Body(stat);

            movement.Act(TurnLeftStart);
            Assert.Equal(-standing, movement.Core.Vehicle.TurnRate, 5);

            movement.Act(ForwardStart);
            Assert.Equal(-driving, movement.Core.Vehicle.TurnRate, 5);

            movement.Act(ForwardStop);
            Assert.Equal(-standing, movement.Core.Vehicle.TurnRate, 5);

            movement.Act(TurnLeftStop);
            Assert.Equal(0f, movement.Core.Vehicle.TurnRate);

            movement.Act(ForwardStart);
            movement.Act(TurnRightStart);
            Assert.Equal(driving, movement.Core.Vehicle.TurnRate, 5);
        }

        [Fact]
        public void AMouseTurnHasNoRateAndDrivingDoesNotGiveItOne()
        {
            var movement = Body();

            movement.Act(TurnLeftMouse);
            movement.Act(ForwardStart);

            Assert.Equal(0f, movement.Core.Vehicle.TurnRate);
        }

        [Fact]
        public void AGroundJumpLaunchesAndItsLandingEndsIt()
        {
            var movement = Body();
            float ground = movement.Core.Position.Y;

            movement.Act(JumpStart);
            Run(movement, 0.3f);
            Assert.True(movement.Core.Position.Y > ground + 0.5f);

            Run(movement, 2f);
            Assert.Equal(CharMovementStatus.Idle, movement.Status.Current.JumpState);
            Assert.Equal(ground, movement.Core.Position.Y, 3);
        }

        [Fact]
        public void JumpingWhileFlyingIsDropped()
        {
            var movement = Body();
            movement.Act(SwitchToFly);
            Run(movement, 0.5f);
            float y = movement.Core.Position.Y;
            CharMovementStatus.Fields before = movement.Status.Current;

            Assert.False(movement.Act(JumpStart));

            Assert.Equal(before, movement.Status.Current);
            Assert.False(movement.Core.Vehicle.FallingEnabled);
            Run(movement, 1f);
            Assert.Equal(y, movement.Core.Position.Y, 4);
        }

        [Fact]
        public void MechDataRefusesTheJump()
        {
            var movement = Body();
            movement.SetStats(new MovementStats { Features = 0x4, MechData = 1 });

            Assert.False(movement.Act(JumpStart));
            Assert.Equal(CharMovementStatus.Idle, movement.Status.Current.JumpState);
        }

        [Fact]
        public void FearAndAWaypointPathDropEveryControl()
        {
            var movement = Body();

            movement.AiControlled = true;
            Assert.False(movement.Act(ForwardStart));
            movement.AiControlled = false;

            movement.OnWaypointPath = true;
            Assert.False(movement.Act(ForwardStart));
            movement.OnWaypointPath = false;

            Assert.True(movement.Act(ForwardStart));
        }
    }
}
