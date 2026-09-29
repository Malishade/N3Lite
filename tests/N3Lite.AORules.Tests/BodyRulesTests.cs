using N3Lite.AORules;
using N3Lite.Bodies;
using Xunit;

namespace N3Lite.AORules.Tests
{
    public class BodyRulesTests
    {
        static ContactBody Body(float x, float z, float radius, int flags)
        {
            var body = new ContactBody { Position = new Vec3(x, 0f, z) };
            BodyRules.ApplyFlags(body, flags);
            BodyRules.SetMeshSphere(body, new Vec3(0f, 0f, 0f), radius, 1f);
            return body;
        }

        [Fact]
        public void SpheresThatJustTouchCount()
        {
            Assert.True(BodyContacts.SpheresTouch(new Vec3(0f, 0f, 0f), 1f, new Vec3(3f, 0f, 0f), 2f));
            Assert.False(BodyContacts.SpheresTouch(new Vec3(0f, 0f, 0f), 1f, new Vec3(3.01f, 0f, 0f), 2f));
        }

        [Fact]
        public void AWantingBodyIsToldByATellingOne()
        {
            var door = Body(0f, 0f, 1.5f, BodyRules.FlagWantsContacts);
            var character = Body(1f, 0f, 0.25f, BodyRules.FlagTellsContacts);

            BodyContacts.Run(new[] { door, character });

            Assert.Single(door.Contacts);
            Assert.Same(character, door.Contacts[0]);
            Assert.Empty(character.Contacts);
        }

        [Fact]
        public void TheOrderOfThePairDoesNotChangeWhoIsTold()
        {
            var door = Body(0f, 0f, 1.5f, BodyRules.FlagWantsContacts);
            var character = Body(1f, 0f, 0.25f, BodyRules.FlagTellsContacts);

            BodyContacts.Collide(character, door);

            Assert.Single(door.Contacts);
            Assert.Empty(character.Contacts);
        }

        [Fact]
        public void NobodyWantingMeansNoCheck()
        {
            var a = Body(0f, 0f, 5f, BodyRules.FlagTellsContacts);
            var b = Body(1f, 0f, 5f, BodyRules.FlagTellsContacts);

            Assert.False(BodyContacts.Check(a, b));
        }

        [Fact]
        public void BodiesMoreThanTenApartOnXOrZAreNotChecked()
        {
            var a = Body(0f, 0f, 50f, BodyRules.FlagWantsContacts);
            var b = Body(10.5f, 0f, 50f, BodyRules.FlagTellsContacts);

            Assert.False(BodyContacts.Check(a, b));
        }

        [Fact]
        public void AnExcludedBodyIsNotChecked()
        {
            var door = Body(0f, 0f, 1.5f, BodyRules.FlagWantsContacts);
            var character = Body(1f, 0f, 0.25f, BodyRules.FlagTellsContacts);
            character.Mode = ContactMode.Excluded;

            BodyContacts.Collide(door, character);

            Assert.Empty(door.Contacts);
        }

        [Fact]
        public void TheSphereKeepsOnlyItsHeightAndScales()
        {
            var body = new ContactBody();
            BodyRules.SetMeshSphere(body, new Vec3(0.3f, -0.66f, 0.15f), 1.53f, 2f);

            Assert.Equal(0f, body.SphereOffset.X);
            Assert.Equal(-1.32f, body.SphereOffset.Y, 5);
            Assert.Equal(0f, body.SphereOffset.Z);
            Assert.Equal(3.06f, body.SphereRadius, 5);
        }

        [Fact]
        public void ANegativeRadiusBecomesHalf()
        {
            var body = new ContactBody();
            BodyRules.SetMeshSphere(body, new Vec3(0f, 0f, 0f), -1f, 1f);

            Assert.Equal(0.5f, body.SphereRadius);
        }

        [Fact]
        public void AnUnloadedCatMeshUsesRadiusOneAtHalfHeight()
        {
            var body = new ContactBody();
            BodyRules.SetCatMeshSphere(body, false, new Vec3(0f, 9f, 0f), 9f, 1f);

            Assert.Equal(0.5f, body.SphereOffset.Y);
            Assert.Equal(1f, body.SphereRadius);
        }

        [Fact]
        public void TheSphereFollowsTheBodysRotation()
        {
            var body = new ContactBody
            {
                Position = new Vec3(10f, 0f, 0f),
                Rotation = Quat.FromAxisAngle(new Vec3(1f, 0f, 0f), (float)System.Math.PI / 2f),
            };
            BodyRules.SetMeshSphere(body, new Vec3(0f, 1f, 0f), 1f, 1f);

            Vec3 center = body.SphereCenter;
            Assert.Equal(10f, center.X, 4);
            Assert.Equal(0f, center.Y, 4);
            Assert.Equal(1f, System.Math.Abs(center.Z), 4);
        }
    }
}
