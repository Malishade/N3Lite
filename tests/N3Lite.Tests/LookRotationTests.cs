using System;
using Xunit;

namespace N3Lite.Tests
{
    public class LookRotationTests
    {
        static void AssertNear(Vec3 expected, Vec3 actual)
        {
            Assert.Equal(expected.X, actual.X, 4);
            Assert.Equal(expected.Y, actual.Y, 4);
            Assert.Equal(expected.Z, actual.Z, 4);
        }

        [Fact]
        public void ASlopedBodyFacesTheProjectedForwardWithTheNormalAsUp()
        {
            Vec3 up = new Vec3(0.3f, 1f, -0.2f);
            up = up * (1f / up.Length);
            Vec3 forward = new Vec3(1f, 0f, 2f);

            Quat q = Quat.LookRotation(ref forward, ref up);

            Assert.Equal(1f, forward.Length, 5);
            Assert.Equal(0f, Vec3.Dot(forward, up), 5);
            AssertNear(forward, q * Vec3.ReferenceForward);
            AssertNear(up, q * Vec3.ReferenceUp);
        }

        [Fact]
        public void FacingBackwardsTakesTheMatrixFallback()
        {
            // The direct quaternion is (0,0,0,0) here, so this is the half-turn branch.
            Vec3 forward = new Vec3(0f, 0f, -1f);
            Vec3 up = Vec3.ReferenceUp;

            Quat q = Quat.LookRotation(ref forward, ref up);

            Assert.Equal(new Quat(0f, 1f, 0f, 0f), q);
        }

        [Fact]
        public void AnUpsideDownUpIsSquaredInTheCallersVector()
        {
            Vec3 forward = new Vec3(0f, 0f, 1f);
            Vec3 up = new Vec3(0f, -1f, 0f);

            Quat.LookRotation(ref forward, ref up);

            Assert.Equal(Vec3.ReferenceUp, up);
        }

        [Fact]
        public void AStraightDownForwardBecomesCrossOfReferenceForwardAndUp()
        {
            Vec3 forward = new Vec3(0f, -5f, 0f);
            Vec3 up = Vec3.ReferenceUp;

            Quat q = Quat.LookRotation(ref forward, ref up);

            AssertNear(new Vec3(-1f, 0f, 0f), forward);
            AssertNear(forward, q * Vec3.ReferenceForward);
        }
    }
}
