namespace N3Lite
{
    /// <summary>
    /// The camera's eye, <c>n3Camera_t::UpdateTargetEye</c> (<c>N3 10020bdb</c>): where the camera
    /// orbits on the character follows the character mesh's <c>Attractor31_camera</c> as it is
    /// animated, so a swim, a sit or a crawl moves the pivot with the upper body.
    ///
    /// <para>
    /// The camera entity runs it on every camera Run while there is a controlled character
    /// (<c>10022345</c> at <c>10022aa5</c>; every view mode reaches that call), when its vehicle is set
    /// (<c>1002107a</c>), and when the "UseNoBobCamera" or "3rdPersonCamera" pref changes
    /// (<c>1002104d</c>, <c>100219d5</c>). The result goes to the vehicle's
    /// <see cref="CameraVehicleSim.SetEyeTargetLocalPos"/>.
    /// </para>
    ///
    /// <para>
    /// The blends are per call, as stock's are: stock runs this once per camera Run, i.e. once a
    /// frame, with no time step.
    /// </para>
    /// </summary>
    public sealed class CameraTargetEye
    {
        /// <summary>
        /// <c>+0x1e0</c>: the "UseNoBobCamera" pref (<c>OptionPanel/Root.xml</c>, Login category).
        /// Read once by the constructor as <c>value != 0</c> (<c>10021a76</c>) and re-set by its
        /// change callback as <c>value == 1</c> (<c>1002104d</c>).
        /// </summary>
        public bool NoBob;

        /// <summary>
        /// <c>+0x1e1</c>: take the attractor as it is on the next update instead of easing to it.
        /// Set by the constructor, by a failed attractor read, and by a teleport in first person with
        /// <see cref="NoBob"/> on (<c>10022b2d</c>); cleared by every update that does not take the
        /// no-bob branch.
        /// </summary>
        public bool Snap = true;

        /// <summary><c>+0x240</c>: the attractor's x and z are not cached yet (set only by the constructor).</summary>
        bool _cacheEmpty = true;

        /// <summary><c>+0x234</c>: the first attractor read, times the body scale.</summary>
        Vec3 _cached;

        /// <summary>
        /// <c>DAT_1005c87c</c>, a static in stock: the no-bob branch is easing. It starts once the eye is
        /// more than 0.5 m off (squared 0.25, <c>1003e2b0</c>) and stops inside 0.1 m (squared 0.01,
        /// <c>1003e2b4</c>).
        /// </summary>
        bool _noBobEasing;

        /// <summary>
        /// <c>10020af1</c>: the attractor's translation as the eye. The first read caches all three
        /// axes times <paramref name="bodyScale"/>; every later one keeps the cached x and z and takes
        /// only the height again, times the body scale of the moment.
        /// </summary>
        public Vec3 FromAttractor(Vec3 attractor, float bodyScale)
        {
            if (_cacheEmpty)
            {
                _cached = attractor * bodyScale;
                _cacheEmpty = false;
                return _cached;
            }

            return new Vec3(_cached.X, bodyScale * attractor.Y, _cached.Z);
        }

        /// <summary>
        /// <c>10020bdb</c>: the eye to set, from the vehicle's <paramref name="current"/> eye.
        /// <paramref name="hasMesh"/> false (no controlled character or no CAT mesh) gives a zero eye;
        /// <paramref name="attractorFound"/> false (neither <c>Attractor31_camera</c> nor
        /// <c>Attractor01_head</c>) gives a zero target and sets <see cref="Snap"/>. The height is
        /// floored at 0.3 (<c>1003e29c</c>) on every path.
        /// </summary>
        public Vec3 Update(Vec3 current, CameraViewMode viewMode, bool hasMesh, bool attractorFound,
            Vec3 attractor, float bodyScale)
        {
            Vec3 eye = Vec3.Zero;
            if (hasMesh)
            {
                if (attractorFound)
                    eye = FromAttractor(attractor, bodyScale);
                else
                    Snap = true;

                if (NoBob && !Snap)
                {
                    // 10020ca7: squared distance, no root (100013a9).
                    float d2 = (current - eye).LengthSquared;
                    if (!(0.01f <= d2))
                    {
                        _noBobEasing = false;
                        eye = current;
                    }
                    else if (!_noBobEasing && !(0.25f < d2))
                    {
                        eye = current;
                    }
                    else
                    {
                        _noBobEasing = true;
                        eye = eye * 0.01f + current * 0.99f;
                    }
                }
                else
                {
                    if (viewMode == CameraViewMode.FirstPerson)
                    {
                        // 10020d4a: the target weighted 0.999 (1003e2a4), the current 0.0009999871 (1003e2a8).
                        eye = eye * 0.999f + current * 0.0009999871f;
                    }
                    else
                    {
                        // 10020d68: a height of 100000 (1003e2a0) keeps the current eye.
                        if (eye.Y == 100000f)
                            eye = current;

                        // 10020d83: eased only when the current height is not zero.
                        if (current.Y != 0f && !Snap)
                            eye = eye * 0.2f + current * 0.8f;
                    }

                    Snap = false;
                }
            }

            if (eye.Y < 0.3f)
                eye.Y = 0.3f;
            return eye;
        }
    }
}
