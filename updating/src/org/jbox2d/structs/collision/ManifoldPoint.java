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

package org.jbox2d.structs.collision;

import org.jbox2d.structs.collision.ContactID;
import org.jbox2d.common.Vec2;

//Updated to rev 56->108->139->218 of b2Collision.h

/**
 * A manifold point is a contact point belonging to a contact
 * manifold. It holds details related to the geometry and dynamics
 * of the contact points.
 * The local point usage depends on the manifold type:
 * <ul><li>e_circles: the local center of circleB</li>
 * <li>e_faceA: the local center of cirlceB or the clip point of polygonB</li>
 * <li>e_faceB: the clip point of polygonA</li></ul>
 * This structure is stored across time steps, so we keep it small.<br/>
 * Note: the impulses are used for internal caching and may not
 * provide reliable contact forces, especially for high speed collisions.
 */
public class ManifoldPoint {
	/** usage depends on manifold type */
	public final Vec2 m_localPoint;
	/** the non-penetration impulse */
	public float m_normalImpulse;
	/** the friction impulse */
	public float m_tangentImpulse;
	/** uniquely identifies a contact point between two shapes */
	public final ContactID m_id;

	/**
	 * Blank manifold point with everything zeroed out.
	 */
	public ManifoldPoint() {
		m_localPoint = new Vec2();
		m_normalImpulse = m_tangentImpulse = 0f;
		m_id = new ContactID();
	}

	/**
	 * Creates a manifold point as a copy of the given point
	 * @param cp point to copy from
	 */
	public ManifoldPoint(final ManifoldPoint cp) {
		m_localPoint = cp.m_localPoint.clone();
		m_normalImpulse = cp.m_normalImpulse;
		m_tangentImpulse = cp.m_tangentImpulse;
		m_id = new ContactID(cp.m_id);
	}

	/**
	 * Sets this manifold point form the given one
	 * @param cp the point to copy from
	 */
	public void set(final ManifoldPoint cp){
		m_localPoint.set(cp.m_localPoint);
		m_normalImpulse = cp.m_normalImpulse;
		m_tangentImpulse = cp.m_tangentImpulse;
		m_id.set(cp.m_id);
	}
}
