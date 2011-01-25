/*******************************************************************************
 * Copyright (c) 2011, Daniel Murphy
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL DANIEL MURPHY BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
package org.jbox2d.dynamics.contacts;

import java.util.Stack;

import org.jbox2d.callbacks.ContactListener;
import org.jbox2d.collision.Manifold;
import org.jbox2d.collision.WorldManifold;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.collision.shapes.ShapeType;
import org.jbox2d.common.Transform;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.pooling.MutableStack;
import org.jbox2d.pooling.TLManifold;
import org.jbox2d.pooling.WorldPool;
import org.jbox2d.pooling.stacks.TLStack;
import org.jbox2d.structs.collision.ContactID;
import org.jbox2d.structs.collision.ManifoldPoint;

// updated to rev 100
/**
 * The class manages contact between two shapes. A contact exists for each
 * overlapping AABB in the broad-phase (except if filtered). Therefore a contact
 * object may exist that has no contact points.
 * 
 * @author daniel
 */
public abstract class Contact {
	
	// statistics gathering
	public static int activeContacts = 0;
	public static int contactPoolCount = 0;
	
	// Flags stored in m_flags
	// Used when crawling contact graph when forming islands.
	public static final int ISLAND_FLAG = 0x0001;
	// Set when the shapes are touching.
	public static final int TOUCHING_FLAG = 0x0002; // NO_UCD
	// This contact can be disabled (by user)
	public static final int ENABLED_FLAG = 0x0004;
	// This contact needs filtering because a fixture filter was changed.
	public static final int FILTER_FLAG = 0x0008;
	// This bullet contact had a TOI event
	public static final int BULLET_HIT_FLAG = 0x0010;

	public int m_flags;

	// World pool and list pointers.
	public Contact m_prev;
	public Contact m_next;

	// Nodes for connecting bodies.
	public ContactEdge m_nodeA = null;
	public ContactEdge m_nodeB = null;

	public Fixture m_fixtureA;
	public Fixture m_fixtureB;

	public Manifold m_manifold;

	public float m_toiCount;
	
	protected final WorldPool pool;

	protected Contact(WorldPool argPool) {
		m_fixtureA = null;
		m_fixtureB = null;
		m_nodeA = new ContactEdge();
		m_nodeB = new ContactEdge();
		m_manifold = new Manifold();
		pool = argPool;
	}

	/** initialization for pooling */
	public void init(Fixture fA, Fixture fB) {
		m_flags = 0;

		m_fixtureA = fA;
		m_fixtureB = fB;

		m_manifold.pointCount = 0;

		m_prev = null;
		m_next = null;

		m_nodeA.contact = null;
		m_nodeA.prev = null;
		m_nodeA.next = null;
		m_nodeA.other = null;

		m_nodeB.contact = null;
		m_nodeB.prev = null;
		m_nodeB.next = null;
		m_nodeB.other = null;

		m_toiCount = 0;
	}

	/**
	 * Get the contact manifold. Do not set the point count to zero. Instead
	 * call Disable.
	 */
	public Manifold getManifold() {
		return m_manifold;
	}

	/**
	 * Get the world manifold.
	 */
	public void getWorldManifold(WorldManifold worldManifold) {
		final Body bodyA = m_fixtureA.getBody();
		final Body bodyB = m_fixtureB.getBody();
		final Shape shapeA = m_fixtureA.getShape();
		final Shape shapeB = m_fixtureB.getShape();

		worldManifold.initialize(m_manifold, bodyA.getTransform(),
				shapeA.m_radius, bodyB.getTransform(), shapeB.m_radius);
	}

	/**
	 * Is this contact touching
	 * 
	 * @return
	 */
	public boolean isTouching() {
		return (m_flags & TOUCHING_FLAG) == TOUCHING_FLAG;
	}

	/**
	 * Enable/disable this contact. This can be used inside the pre-solve
	 * contact listener. The contact is only disabled for the current time step
	 * (or sub-step in continuous collisions).
	 * 
	 * @param flag
	 */
	public void setEnabled(boolean flag) {
		if (flag) {
			m_flags |= ENABLED_FLAG;
		} else {
			m_flags &= ~ENABLED_FLAG;
		}
	}

	/**
	 * Has this contact been disabled?
	 * 
	 * @return
	 */
	public boolean isEnabled() {
		return (m_flags & ENABLED_FLAG) == ENABLED_FLAG;
	}

	/**
	 * Get the next contact in the world's contact list.
	 * 
	 * @return
	 */
	public Contact getNext() {
		return m_next;
	}

	/**
	 * Get the first fixture in this contact.
	 * 
	 * @return
	 */
	public Fixture getFixtureA() {
		return m_fixtureA;
	}

	/**
	 * Get the second fixture in this contact.
	 * 
	 * @return
	 */
	public Fixture getFixtureB() {
		return m_fixtureB;
	}

	public abstract void evaluate(Manifold manifold, Transform xfA,
			Transform xfB);

	/**
	 * Flag this contact for filtering. Filtering will occur the next time step.
	 */
	public void flagForFiltering() {
		m_flags |= FILTER_FLAG;
	}

	// djm pooling
	private static final TLManifold tloldManifold = new TLManifold();

	public void update(ContactListener listener) {

		Manifold oldManifold = tloldManifold.get();
		oldManifold.set(m_manifold);

		// Re-enable this contact.
		m_flags |= ENABLED_FLAG;

		boolean touching = false;
		boolean wasTouching = (m_flags & TOUCHING_FLAG) == TOUCHING_FLAG;

		boolean sensorA = m_fixtureA.isSensor();
		boolean sensorB = m_fixtureB.isSensor();
		boolean sensor = sensorA || sensorB;

		Body bodyA = m_fixtureA.getBody();
		Body bodyB = m_fixtureB.getBody();
		Transform xfA = bodyA.getTransform();
		Transform xfB = bodyB.getTransform();
		//log.debug("TransformA: "+xfA);
		//log.debug("TransformB: "+xfB);
		
		if (sensor) {
			Shape shapeA = m_fixtureA.getShape();
			Shape shapeB = m_fixtureB.getShape();
			touching = pool.getCollision().testOverlap(shapeA, shapeB,
					xfA, xfB);

			// Sensors don't generate manifolds.
			m_manifold.pointCount = 0;
		} else {
			evaluate(m_manifold, xfA, xfB);
			touching = m_manifold.pointCount > 0;

			// Match old contact ids to new contact ids and copy the
			// stored impulses to warm start the solver.
			for (int i = 0; i < m_manifold.pointCount; ++i) {
				ManifoldPoint mp2 = m_manifold.points[i];
				mp2.normalImpulse = 0.0f;
				mp2.tangentImpulse = 0.0f;
				ContactID id2 = mp2.id;

				for (int j = 0; j < oldManifold.pointCount; ++j) {
					ManifoldPoint mp1 = oldManifold.points[j];

					if (mp1.id.isEqual(id2)) {
						mp2.normalImpulse = mp1.normalImpulse;
						mp2.tangentImpulse = mp1.tangentImpulse;
						break;
					}
				}
			}

			if (touching != wasTouching) {
				bodyA.setAwake(true);
				bodyB.setAwake(true);
			}
		}

		if (touching) {
			m_flags |= TOUCHING_FLAG;
		} else {
			m_flags &= ~TOUCHING_FLAG;
		}

		if (listener == null) {
			return;
		}

		if (wasTouching == false && touching == true) {
			listener.beginContact(this);
		}

		if (wasTouching == true && touching == false) {
			listener.endContact(this);
		}

		if (sensor == false && touching) {
			listener.preSolve(this, oldManifold);
		}
	}

	// // djm pooled
	// private static final TLTOIInput tlinput = new TLTOIInput();
	//	
	// protected float computeTOI(Sweep sweepA, Sweep sweepB){
	// TOIInput input = tlinput.get();
	// input.proxyA.set(m_fixtureA.getShape());
	// input.proxyB.set(m_fixtureB.getShape());
	// input.sweepA = sweepA;
	// input.sweepB = sweepB;
	// input.tolerance = Settings.linearSlop;
	//
	// return SingletonPool.getTOI().timeOfImpact(input);
	// }
}
