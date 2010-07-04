/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/
 * Box2D homepage: http://www.box2d.org
 * 
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package org.jbox2d.dynamics;

import java.util.List;

import org.jbox2d.collision.Manifold;
import org.jbox2d.collision.ManifoldPoint;
import org.jbox2d.collision.PairCallback;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.contacts.Contact;
import org.jbox2d.dynamics.contacts.ContactPoint;
import org.jbox2d.dynamics.contacts.NullContact;
import org.jbox2d.pooling.TLContactPoint;
import org.jbox2d.pooling.TLVec2;


//Updated to rev 56->104->142 of b2ContactManager.cpp/.h

/** Delegate of World - for internal use. */
public class ContactManager implements PairCallback {
	World m_world;

	// This lets us provide broadphase proxy pair user data for
	// contacts that shouldn't exist.
	NullContact m_nullContact;

	boolean m_destroyImmediate;

	public ContactManager() {
		m_nullContact = new NullContact();
		m_destroyImmediate = false;
	}

	public Object pairAdded(final Object proxyUserData1, final Object proxyUserData2) {
		Shape shape1 = (Shape) proxyUserData1;
		Shape shape2 = (Shape) proxyUserData2;

		Body body1 = shape1.getBody();
		Body body2 = shape2.getBody();

		if (body1.isStatic() && body2.isStatic()) {
			return m_nullContact;
		}

		if (shape1.getBody() == shape2.getBody()) {
			return m_nullContact;
		}

		if (body2.isConnected(body1)) {
			return m_nullContact;
		}

		if (m_world.m_contactFilter != null && m_world.m_contactFilter.shouldCollide(shape1, shape2) == false){
			return m_nullContact;
		}

		// Call the factory.
		final Contact c = Contact.createContact(shape1, shape2);

		if (c == null) {
			return m_nullContact;
		}

		// Contact creation may swap shapes.
		shape1 = c.getShape1();
		shape2 = c.getShape2();
		body1 = shape1.getBody();
		body2 = shape2.getBody();

		// Insert into the world.
		c.m_prev = null;
		c.m_next = m_world.m_contactList;
		if (m_world.m_contactList != null) {
			m_world.m_contactList.m_prev = c;
		}
		m_world.m_contactList = c;

		// Connect to island graph.

		// Connect to body 1
		c.m_node1.contact = c;
		c.m_node1.other = body2;

		c.m_node1.prev = null;
		c.m_node1.next = body1.m_contactList;
		if (body1.m_contactList != null) {
			body1.m_contactList.prev = c.m_node1;
		}
		body1.m_contactList = c.m_node1;

		// Connect to body 2
		c.m_node2.contact = c;
		c.m_node2.other = body1;

		c.m_node2.prev = null;
		c.m_node2.next = body2.m_contactList;
		if (body2.m_contactList != null) {
			body2.m_contactList.prev = c.m_node2;
		}
		body2.m_contactList = c.m_node2;

		++m_world.m_contactCount;
		return c;
	}

	// This is a callback from the broadphase when two AABB proxies cease
	// to overlap. We retire the b2Contact.
	public void pairRemoved(final Object proxyUserData1, final Object proxyUserData2,
	                        final Object pairUserData) {
		//B2_NOT_USED(proxyUserData1);
		//B2_NOT_USED(proxyUserData2);

		if (pairUserData == null) {
			return;
		}

		final Contact c = (Contact)pairUserData;
		if (c == m_nullContact) {
			return;
		}

		// An attached body is being destroyed, we must destroy this contact
		// immediately to avoid orphaned shape pointers.
		destroy(c);
	}

	// djm pooled
	private static final TLVec2 tlV1 = new TLVec2();
	private static final TLContactPoint tlCp = new TLContactPoint();
	
	public void destroy(final Contact c) {
		
		final Vec2 v1 = tlV1.get();
		final ContactPoint cp = tlCp.get();
		
		final Shape shape1 = c.getShape1();
		final Shape shape2 = c.getShape2();

		// Inform the user that this contact is ending.
		final int manifoldCount = c.getManifoldCount();
		if (manifoldCount > 0 && (m_world.m_contactListener != null))
		{
			final Body b1 = shape1.getBody();
			final Body b2 = shape2.getBody();
			final List<Manifold> manifolds = c.getManifolds();
			cp.shape1 = c.getShape1();
			cp.shape2 = c.getShape2();
			cp.friction = c.m_friction;
			cp.restitution = c.m_restitution;
			for (int i = 0; i < manifoldCount; ++i)
			{
				final Manifold manifold = manifolds.get(i);
				cp.normal.set(manifold.normal);
				for (int j = 0; j < manifold.pointCount; ++j) {

					final ManifoldPoint mp = manifold.points[j];
					b1.getWorldLocationToOut(mp.localPoint1, cp.position);
					b1.getLinearVelocityFromLocalPointToOut(mp.localPoint1, v1);
					// velocity isn't initialized in the contact point
					b2.getLinearVelocityFromLocalPointToOut(mp.localPoint2, cp.velocity);
					cp.velocity.subLocal(v1);
					cp.separation = mp.separation;
					cp.id.set(mp.id);
					m_world.m_contactListener.remove(cp);
				}
			}
		}

		// Remove from the world.
		if (c.m_prev != null) {
			c.m_prev.m_next = c.m_next;
		}

		if (c.m_next != null) {
			c.m_next.m_prev = c.m_prev;
		}

		if (c == m_world.m_contactList) {
			m_world.m_contactList = c.m_next;
		}

		final Body body1 = shape1.getBody();
		final Body body2 = shape2.getBody();

		// Remove from body 1
		if (c.m_node1.prev != null) {
			c.m_node1.prev.next = c.m_node1.next;
		}

		if (c.m_node1.next != null) {
			c.m_node1.next.prev = c.m_node1.prev;
		}

		if (c.m_node1 == body1.m_contactList) {
			body1.m_contactList = c.m_node1.next;
		}

		// Remove from body 2
		if (c.m_node2.prev != null) {
			c.m_node2.prev.next = c.m_node2.next;
		}

		if (c.m_node2.next != null) {
			c.m_node2.next.prev = c.m_node2.prev;
		}

		if (c.m_node2 == body2.m_contactList) {
			body2.m_contactList = c.m_node2.next;
		}

		// Call the factory.
		Contact.destroy(c);
		--m_world.m_contactCount;
		
	}

	public void collide() {
		// Update awake contacts.
		for (Contact c = m_world.m_contactList; c != null; c = c.getNext()) {
			final Body body1 = c.getShape1().getBody();
			final Body body2 = c.getShape2().getBody();
			if (body1.isSleeping() && body2.isSleeping()) {
				continue;
			}

			c.update(m_world.m_contactListener);
		}
	}
}
