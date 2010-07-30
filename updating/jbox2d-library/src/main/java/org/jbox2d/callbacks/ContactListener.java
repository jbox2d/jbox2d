package org.jbox2d.callbacks;

import org.jbox2d.collision.Manifold;
import org.jbox2d.dynamics.contacts.Contact;

// updated to rev 100
/**
 * Implement this class to get contact information. You can use these results for
 * things like sounds and game logic. You can also get contact results by
 * traversing the contact lists after the time step. However, you might miss
 * some contacts because continuous physics leads to sub-stepping.
 * Additionally you may receive multiple callbacks for the same contact in a
 * single time step.
 * You should strive to make your callbacks efficient because there may be
 * many callbacks per time step.
 * @warning You cannot create/destroy Box2D entities inside these callbacks.
 * @author Daniel Murphy
 *
 */
public interface ContactListener {

	/**
	 * Called when two fixtures begin to touch.
	 * @param contact
	 */
	public void beginContact(Contact contact);
	
	/**
	 * Called when two fixtures cease to touch.
	 * @param contact
	 */
	public void endContact(Contact contact);
	
	/**
	 * This is called after a contact is updated. This allows you to inspect a
	 * contact before it goes to the solver. If you are careful, you can modify the
	 * contact manifold (e.g. disable contact).
	 * A copy of the old manifold is provided so that you can detect changes.
	 * Note: this is called only for awake bodies.
	 * Note: this is called even when the number of contact points is zero.
	 * Note: this is not called for sensors.
	 * Note: if you set the number of contact points to zero, you will not
	 * get an EndContact callback. However, you may get a BeginContact callback
	 * the next step.
	 * Note: the oldManifold parameter is pooled, so it will be the same object for every callback
	 * for each thread.
	 * @param contact
	 * @param oldManifold
	 */
	public void preSolve(Contact contact, Manifold oldManifold);
	
	/**
	 * This lets you inspect a contact after the solver is finished. This is useful
	 * for inspecting impulses.
	 * Note: the contact manifold does not include time of impact impulses, which can be
	 * arbitrarily large if the sub-step is small. Hence the impulse is provided explicitly
	 * in a separate data structure.
	 * Note: this is only called for contacts that are touching, solid, and awake.
	 * @param contact
	 * @param impulse this is usually a pooled variable, so it will be modified after
	 * this call
	 */
	public void postSolve(Contact contact, ContactImpulse impulse);
}
