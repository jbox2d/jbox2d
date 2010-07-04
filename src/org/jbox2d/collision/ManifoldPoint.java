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

package org.jbox2d.collision;

import org.jbox2d.common.Vec2;

//Updated to rev 56->108->139 of b2Collision.h

/**
 * A manifold point is a contact point belonging to a contact
 * manifold. It holds details related to the geometry and dynamics
 * of the contact points.
 * The point is stored in local coordinates because CCD
 * requires sub-stepping in which the separation is stale.
 */
public class ManifoldPoint {
	/** Local position of the contact point in body1 */
    public final Vec2 localPoint1;
    /** Local position of the contact point in body2 */
    public final Vec2 localPoint2;
    /** The separation of the shapes along the normal vector */
    public float separation;
    /** The non-penetration force */
    public float normalImpulse;
    /** The friction force */
    public float tangentImpulse;
    /** Uniquely identifies a contact point between two shapes */
    public final ContactID id;

    /**
     * Blank manifold point with everything zeroed out.
     */
    public ManifoldPoint() {
        localPoint1 = new Vec2();
        localPoint2 = new Vec2();
        separation = normalImpulse = tangentImpulse = 0f;
        id = new ContactID();
    }

    /**
     * Creates a manifold point as a copy of the given point
     * @param cp point to copy from
     */
    public ManifoldPoint(ManifoldPoint cp) {
        localPoint1 = cp.localPoint1.clone();
        localPoint2 = cp.localPoint2.clone();
        separation = cp.separation;
        normalImpulse = cp.normalImpulse;
        tangentImpulse = cp.tangentImpulse;
        id = new ContactID(cp.id);
    }
    
    /**
     * Sets this manifold point form the given one
     * @param cp the point to copy from
     */
    public void set(ManifoldPoint cp){
    	localPoint1.set(cp.localPoint1);
    	localPoint2.set(cp.localPoint2);
        separation = cp.separation;
        normalImpulse = cp.normalImpulse;
        tangentImpulse = cp.tangentImpulse;
        id.set(cp.id);
    }
}
