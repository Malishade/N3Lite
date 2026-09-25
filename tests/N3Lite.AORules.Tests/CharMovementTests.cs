using N3Lite.AORules;
using Xunit;

namespace N3Lite.AORules.Tests
{
    public class CharMovementTests
    {
        [Fact]
        public void StatsSetTheJumpAndBodyHeights()
        {
            var movement = new CharMovement();
            movement.SetStats(new MovementStats { Strength = 300, Agility = 300, Scale = 150 });

            Assert.Equal(4f, movement.Core.JumpHeight, 5);
            Assert.Equal(3f, movement.Core.BodyHeight, 5);
        }

        [Fact]
        public void AnUnsetScaleReadsAsOne()
        {
            var movement = new CharMovement();
            movement.SetStats(new MovementStats());

            Assert.Equal(1f, movement.Core.JumpHeight, 5);
            Assert.Equal(2f, movement.Core.BodyHeight, 5);
        }

        [Fact]
        public void RunSpeedDrivesTheProfile()
        {
            var movement = new CharMovement();
            movement.SetStats(new MovementStats { RunSpeed = 275 });

            Assert.Equal(6f, movement.Core.Profile.ForwardSpeed, 3);
        }

        [Fact]
        public void TheStatsSurviveAStateChange()
        {
            var movement = new CharMovement();
            movement.SetStats(new MovementStats { RunSpeed = 275 });
            movement.EnterState(CharMovementRules.StateWalk);
            movement.EnterState(CharMovementRules.StateRun);

            Assert.Equal(6f, movement.Core.Profile.ForwardSpeed, 3);
        }

        [Fact]
        public void SittingRefusesTheJump()
        {
            var movement = new CharMovement();
            movement.EnterState(CharMovementRules.StateSit);

            Assert.False(movement.Core.Profile.AcceptsInput);
            Assert.False(movement.Core.TryStartJump());
        }

        [Fact]
        public void LeavingAStateReturnsToTheLastSpeedMode()
        {
            var movement = new CharMovement();
            movement.EnterState(CharMovementRules.StateWalk);
            movement.EnterState(CharMovementRules.StateSit);
            movement.LeaveState();

            Assert.Equal(CharMovementRules.StateWalk, movement.State);
        }

        [Fact]
        public void LeavingWithNoSpeedModeReturnsToRun()
        {
            var movement = new CharMovement();
            movement.LastSpeedMode = CharMovementRules.StateFly;
            movement.EnterState(CharMovementRules.StateSit);
            movement.LeaveState();

            Assert.Equal(CharMovementRules.StateRun, movement.State);
        }
    }
}
