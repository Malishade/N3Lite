using N3Lite.Surfaces;
using Xunit;

namespace N3Lite.AORules.Tests
{
    public class FlyTests
    {
        sealed class Flat : ITileHeightSource
        {
            public int Width => 512;
            public int Height => 512;
            public float TileSize => 1f;
            public float SampleHeight(int x, int z) => 0f;
        }

        static CharMovement Grounded(int orientationMode = 1)
        {
            var movement = new CharMovement();
            movement.SetStats(new MovementStats { OrientationMode = orientationMode });
            movement.Core.Surface = new TilemapSurface(new Flat());
            movement.Core.Warp(new Vec3(100f, 0f, 100f), new Vec3(0f, 0f, 1f));
            for (int i = 0; i < 30; i++)
                movement.Tick(1f / 30f);
            return movement;
        }

        static void Run(CharMovement movement, float seconds)
        {
            for (int i = 0; i < (int)(seconds * 30f); i++)
                movement.Tick(1f / 30f);
        }

        [Fact]
        public void SwitchingToFlyStopsFallingLiftsAndPitches()
        {
            var movement = Grounded();
            float ground = movement.Core.Position.Y;

            movement.EnterState(CharMovementRules.StateFly);

            Assert.False(movement.Core.Vehicle.FallingEnabled);
            Assert.Equal(ground + CharMovementRules.FlyStartLift, movement.Core.Position.Y, 4);
            Assert.Equal(CharMovementRules.FlyOrientationMode, movement.Core.Vehicle.OrientationMode);
        }

        [Fact]
        public void ElevatingUpRisesAndStoppingSinksSlowly()
        {
            var movement = Grounded();
            movement.EnterState(CharMovementRules.StateFly);
            float start = movement.Core.Position.Y;

            movement.ElevateUpStart();
            Assert.Equal(CharMovementRules.ElevateUpSpeed, movement.Core.Vehicle.Vertical);
            Run(movement, 1f);
            float top = movement.Core.Position.Y;
            Assert.Equal(start + 3f, top, 1);

            movement.ElevateUpStop();
            Assert.Equal(CharMovementRules.ElevateStopSpeed, movement.Core.Vehicle.Vertical);
            Run(movement, 1f);
            Assert.Equal(top - 0.8f, movement.Core.Position.Y, 1);
        }

        [Fact]
        public void LeavingFlyFallsAgainFromTheStatsOrientation()
        {
            var movement = Grounded(orientationMode: 0);
            movement.EnterState(CharMovementRules.StateFly);
            movement.ElevateUpStart();
            Run(movement, 1f);
            float high = movement.Core.Position.Y;

            movement.LeaveFly();

            Assert.Equal(CharMovementRules.StateRun, movement.State);
            Assert.True(movement.Core.Vehicle.FallingEnabled);
            Assert.Equal(0f, movement.Core.Vehicle.Vertical);
            Assert.Equal(0, movement.Core.Vehicle.OrientationMode);
            Assert.Equal(high + CharMovementRules.FlyLeaveLift, movement.Core.Position.Y, 4);

            Run(movement, 3f);
            Assert.True(movement.Core.Position.Y < 1f);
        }

        [Fact]
        public void OnlyWalkRunAndSwimLeaveFlightBehind()
        {
            var movement = Grounded();
            movement.EnterState(CharMovementRules.StateFly);

            movement.EnterState(CharMovementRules.StateSit);
            Assert.False(movement.Core.Vehicle.FallingEnabled);
            Assert.Equal(CharMovementRules.FlyOrientationMode, movement.Core.Vehicle.OrientationMode);

            movement.EnterState(CharMovementRules.StateFly);
            movement.EnterState(CharMovementRules.StateSwim);
            Assert.True(movement.Core.Vehicle.FallingEnabled);
        }

        [Fact]
        public void AFlyingStatusIsTakenWithoutTheLift()
        {
            var movement = Grounded();
            float ground = movement.Core.Position.Y;

            movement.ApplyStatusState(CharMovementRules.StateFly);

            Assert.False(movement.Core.Vehicle.FallingEnabled);
            Assert.Equal(CharMovementRules.FlyOrientationMode, movement.Core.Vehicle.OrientationMode);
            Assert.Equal(ground, movement.Core.Position.Y);
        }
    

        [Theory]
        [InlineData(0, 0)]
        [InlineData(1, 1)]
        [InlineData(2, 1)]
        public void Stat197SetsTheOrientationMode(int stat, int mode)
        {
            var movement = new CharMovement();
            movement.SetStats(new MovementStats { OrientationMode = stat });

            Assert.Equal(mode, movement.Core.Vehicle.OrientationMode);
        }

        [Fact]
        public void ACharacterStartsLevel()
            => Assert.Equal(0, new CharMovement().Core.Vehicle.OrientationMode);

        [Fact]
        public void NewStatsWhileFlyingKeepTheFlyingOrientation()
        {
            var movement = Grounded();
            movement.EnterState(CharMovementRules.StateFly);
            float y = movement.Core.Position.Y;

            movement.SetStats(new MovementStats { OrientationMode = 1 });

            Assert.Equal(CharMovementRules.FlyOrientationMode, movement.Core.Vehicle.OrientationMode);
            Assert.False(movement.Core.Vehicle.FallingEnabled);
            Assert.Equal(y, movement.Core.Position.Y);
        }

        [Fact]
        public void LeavingFlyTakesStat197AsItIs()
        {
            var movement = Grounded(orientationMode: 2);
            movement.EnterState(CharMovementRules.StateFly);
            movement.LeaveFly();

            Assert.Equal(2, movement.Core.Vehicle.OrientationMode);
        }
    }
}
