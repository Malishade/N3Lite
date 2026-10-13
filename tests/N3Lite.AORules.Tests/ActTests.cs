using N3Lite.Surfaces;
using Xunit;

namespace N3Lite.AORules.Tests
{
    public class ActTests
    {
        const int ForwardStart = 1, ElevateUpStart = 0x11, ElevateUpStop = 0x12, SwitchToRun = 0x19;
        const int SwitchToFly = 0x1d, SwitchToSit = 0x1e, LeaveFly = 0x27;

        sealed class Flat : ITileHeightSource
        {
            public int Width => 512;
            public int Height => 512;
            public float TileSize => 1f;
            public float SampleHeight(int x, int z) => 0f;
        }

        static CharMovement Body()
        {
            var movement = new CharMovement();
            movement.SetStats(new MovementStats { Features = 0x4 });
            movement.Core.Surface = new TilemapSurface(new Flat());
            movement.Core.Warp(new Vec3(100f, 0f, 100f), new Vec3(0f, 0f, 1f));
            for (int i = 0; i < 30; i++)
                movement.Tick(1f / 30f);
            return movement;
        }

        [Fact]
        public void ElevatingIsIgnoredOutOfFly()
        {
            var movement = Body();

            Assert.True(movement.Act(ElevateUpStart));
            Assert.Equal(0f, movement.Core.Vehicle.Vertical);
        }

        [Fact]
        public void FlyingThroughTheStatusMachine()
        {
            var movement = Body();
            int set = 0, cleared = 0;
            movement.FlagsSet += bits => set |= bits;
            movement.FlagsCleared += bits => cleared |= bits;

            Assert.True(movement.Act(SwitchToFly));
            Assert.Equal(CharMovementRules.StateFly, movement.State);
            Assert.False(movement.Core.Vehicle.FallingEnabled);
            Assert.Equal(CharMovementRules.FlagFlying, set);

            Assert.True(movement.Act(ElevateUpStart));
            Assert.Equal(CharMovementRules.ElevateUpSpeed, movement.Core.Vehicle.Vertical);
            Assert.True(movement.Act(ElevateUpStop));
            Assert.Equal(CharMovementRules.ElevateStopSpeed, movement.Core.Vehicle.Vertical);

            Assert.False(movement.Act(SwitchToSit));            // Fly refuses sitting
            Assert.Equal(CharMovementRules.StateFly, movement.State);

            Assert.True(movement.Act(LeaveFly));
            Assert.Equal(movement.Status.LastSpeedMode, movement.State);
            Assert.True(movement.Core.Vehicle.FallingEnabled);
            Assert.Equal(0f, movement.Core.Vehicle.Vertical);
            Assert.Equal(CharMovementRules.FlagFlying, cleared);
        }

        [Fact]
        public void SwitchingToRunLandsAFlyerAndSetsTheGait()
        {
            var movement = Body();
            movement.Act(SwitchToFly);

            movement.Act(SwitchToRun);

            Assert.True(movement.Core.Vehicle.FallingEnabled);
            Assert.Equal(CharMovementRules.StateRun, movement.Status.LastSpeedMode);
        }

        [Fact]
        public void ActionsDriveTheBodysFlags()
        {
            var movement = Body();

            movement.Act(ForwardStart);

            Assert.Equal(MovementFlags.Forward, movement.Core.Flags);
        }
    }
}
