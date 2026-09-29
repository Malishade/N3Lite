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
        public void Stat224Bit4LetsAnNpcFollow()
        {
            var movement = new CharMovement(isNpc: true);
            Assert.False(movement.Core.Npc.FollowEnabled);

            movement.SetStats(new MovementStats { Features = 0x4 });
            Assert.True(movement.Core.Npc.FollowEnabled);

            movement.SelectVehicleKind(false);
            movement.SelectVehicleKind(true);        // a rebuilt vehicle gets it again
            Assert.True(movement.Core.Npc.FollowEnabled);
        }

        [Fact]
        public void AFollowPathEndsAtAZeroPointAndHoldsThirty()
        {
            var movement = new CharMovement(isNpc: true);

            movement.SetFollowPath(new[] { new Vec3(1f, 0f, 1f), Vec3.Zero, new Vec3(2f, 0f, 2f) }, snapToStart: false);
            Assert.Single(movement.Core.Npc.FollowQueue);

            var many = new Vec3[40];
            for (int i = 0; i < many.Length; i++)
                many[i] = new Vec3(i + 1f, 0f, 0f);
            movement.SetFollowPath(many, snapToStart: false);
            Assert.Equal(CharMovement.FollowPathCapacity, movement.Core.Npc.FollowQueue.Count);
        }

        [Fact]
        public void AFollowPathSnapsToItsStartAndQueuesTheRest()
        {
            var movement = new CharMovement(isNpc: true);

            Assert.True(movement.SetFollowPath(new[] { new Vec3(50f, 0f, 60f), new Vec3(55f, 0f, 60f) }, snapToStart: true));

            Assert.Equal(50f, movement.Core.Position.X, 3);
            Assert.Single(movement.Core.Npc.FollowQueue);
            Assert.Equal(55f, movement.Core.Npc.FollowQueue[0].X, 3);
        }

        [Fact]
        public void AZeroStartIsNotSnappedTo()
        {
            var movement = new CharMovement(isNpc: true);
            movement.Core.Warp(new Vec3(10f, 0f, 10f), new Vec3(0f, 0f, 1f));

            movement.SetFollowPath(new[] { Vec3.Zero, new Vec3(55f, 0f, 60f) }, snapToStart: true);

            Assert.Equal(10f, movement.Core.Position.X, 3);
            Assert.Single(movement.Core.Npc.FollowQueue);
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
