using System.Collections.Generic;

namespace N3Lite.Bodies
{
    /// <summary>How a <see cref="ContactBody"/> takes part in body-against-body contact.</summary>
    public enum ContactMode
    {
        /// <summary>Checked only against a body that is not passive.</summary>
        Passive = 0,

        /// <summary>Never checked against any body.</summary>
        Excluded = 1,

        /// <summary>Checked against every body that is not <see cref="Excluded"/>.</summary>
        Active = 2,
    }

    /// <summary>
    /// A body for body-against-body contact: one sphere carried by the body's position and rotation,
    /// and the flags that decide who is told about a contact. <see cref="BodyContacts"/> runs the
    /// checks.
    /// </summary>
    public sealed class ContactBody
    {
        public Vec3 Position;
        public Quat Rotation = Quat.Identity;

        /// <summary>The sphere's centre relative to <see cref="Position"/>, before <see cref="Rotation"/>.</summary>
        public Vec3 SphereOffset;

        public float SphereRadius;

        /// <summary>False while the body has no sphere; a pair with such a body is never in contact.</summary>
        public bool HasSphere;

        /// <summary>Wants to be told about contacts.</summary>
        public bool WantsContacts;

        /// <summary>Tells the bodies it touches about itself.</summary>
        public bool TellsContacts;

        /// <summary>
        /// Pushes bodies it touches apart. Recorded only: <see cref="BodyContacts"/> does not push
        /// bodies apart.
        /// </summary>
        public bool Repulsive;

        public ContactMode Mode = ContactMode.Active;

        /// <summary>Whatever the caller hangs the body off.</summary>
        public object Owner;

        readonly List<ContactBody> _contacts = new List<ContactBody>();

        /// <summary>The bodies that told this one about a contact since <see cref="ClearContacts"/>.</summary>
        public IReadOnlyList<ContactBody> Contacts => _contacts;

        /// <summary>Where the sphere is: <c>position + rotation * offset</c>.</summary>
        public Vec3 SphereCenter => Position + Rotation * SphereOffset;

        public void ClearContacts() => _contacts.Clear();

        /// <summary><paramref name="teller"/> goes on this body's contact list.</summary>
        internal void AddContact(ContactBody teller) => _contacts.Add(teller);
    }
}
