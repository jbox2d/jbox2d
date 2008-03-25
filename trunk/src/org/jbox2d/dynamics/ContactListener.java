package org.jbox2d.dynamics;

import org.jbox2d.dynamics.contacts.ContactPoint;

public interface ContactListener {
	/// Implement this class to get collision results. You can use these results for
	/// things like sounds and game logic. You can also get contact results by
	/// traversing the contact lists after the time step. However, you might miss
	/// some contacts because continuous physics leads to sub-stepping.
	/// Additionally you may receive multiple callbacks for the same contact in a
	/// single time step.
	/// You should strive to make your callbacks efficient because there may be
	/// many callbacks per time step.
	/// @warning The contact separation is the last computed value.
	/// @warning You cannot create/destroy Box2D entities inside these callbacks.

		/// Called when a contact point is added. This includes the geometry
		/// and the forces.
		public void add(ContactPoint point);

		/// Called when a contact point persists. This includes the geometry
		/// and the forces.
		public void persist(ContactPoint point);

		/// Called when a contact point is removed. This includes the last
		/// computed geometry and forces.
		public void remove(ContactPoint point);

}
