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

import org.jbox2d.common.*;
import org.jbox2d.dynamics.Body;


//Updated through rev. 56 of b2Shape.cpp/.h

/// A shape is used for collision detection. Shapes are created in World.
/// You can use shape for collision detection before they are attached to the world.
/// @warning you cannot reuse shapes.
public abstract class Shape {
    public int uid; // unique id for shape for sorting

    static private int uidcount = 0;
    public ShapeType m_type;
    public Shape m_next;
    public Body m_body;
    
    // Sweep radius relative to the parent body's center of mass.
	public float m_sweepRadius;

	public float m_density;
    public float m_friction;
    public float m_restitution;
    

    public int m_proxyId;
    public int m_categoryBits;
    public int m_maskBits;
    public int m_groupIndex;
    
    public boolean m_isSensor;
    public Object m_userData;

    public Shape(ShapeDef def) {

        uid = uidcount++; //Java version only (C++ version sorts by memory location)
        
        m_userData = def.userData;
    	m_friction = def.friction;
    	m_restitution = def.restitution;
    	m_density = def.density;
    	m_body = null;
    	m_sweepRadius = 0.0f;
    	m_next = null;
    	m_proxyId = PairManager.NULL_PROXY;
    	m_categoryBits = def.categoryBits;
    	m_maskBits = def.maskBits;
    	m_groupIndex = def.groupIndex;
    	m_isSensor = def.isSensor;
        
    }
    
    /// Get the type of this shape. You can use this to down cast to the concrete shape.
	/// @return the shape type.
    public ShapeType getType() {
        return m_type;
    }
    
    /// Is this shape a sensor (non-solid)?
	/// @return the true if the shape is a sensor.
    public boolean isSensor() {
    	return m_isSensor;
    }

    /// Get the user data that was assigned in the shape definition. Use this to
	/// store your application specific data.
    public Object getUserData() {
        return m_userData;
    }

    /// Get the parent body of this shape. This is NULL if the shape is not attached.
	/// @return the parent body.
    public Body getBody() {
        return m_body;
    }

    /// Get the next shape in the parent body's shape list.
	/// @return the next shape.
    public Shape getNext() {
        return m_next;
    }
    
    public float getSweepRadius() {
    	return m_sweepRadius;
    }
    
    /// Test a point for containment in this shape. This only works for convex shapes.
	/// @param xf the shape world transform.
	/// @param p a point in world coordinates.
    public abstract boolean testPoint(XForm xf, Vec2 p);
    
    /*
    /// Perform a ray cast against this shape.
	/// @param xf the shape world transform.
	/// @param lambda returns the hit fraction. You can use this to compute the contact point
	/// p = (1 - lambda) * segment.p1 + lambda * segment.p2.
	/// @param normal returns the normal at the contact point. If there is no intersection, the normal
	/// is not set.
	/// @param segment defines the begin and end point of the ray cast.
	/// @param maxLambda a number typically in the range [0,1].
	/// @return true if there was an intersection.
	public  boolean TestSegment(	const b2XForm& xf,
								float32* lambda,
								b2Vec2* normal,
								const b2Segment& segment,
								float32 maxLambda) const = 0;*/
    
    /// Given a transform, compute the associated axis aligned bounding box for this shape.
	/// @param aabb returns the axis aligned box.
	/// @param xf the world transform of the shape.
	public abstract void computeAABB(AABB aabb, XForm xf);
	
	/// Given two transforms, compute the associated swept axis aligned bounding box for this shape.
	/// @param aabb returns the axis aligned box.
	/// @param xf1 the starting shape world transform.
	/// @param xf2 the ending shape world transform.
	public abstract void computeSweptAABB(AABB aabb,
										  XForm xf1,
										  XForm xf2);
	
	/// Compute the mass properties of this shape using its dimensions and density.
	/// The inertia tensor is computed about the local origin, not the centroid.
	/// @param massData returns the mass data for this shape.
	public abstract void computeMass(MassData massData);
	
	
	/* INTERNALS BELOW */
	public abstract void updateSweepRadius(Vec2 center);
    
    public boolean synchronize(BroadPhase broadPhase, XForm transform1, XForm transform2) {
    	if (m_proxyId == PairManager.NULL_PROXY) {	
    		return false;
    	}

    	// Compute an AABB that covers the swept shape (may miss some rotation effect).
    	AABB aabb = new AABB();
    	computeSweptAABB(aabb, transform1, transform2);
    	//if (this.getType() == ShapeType.CIRCLE_SHAPE){
    	//	System.out.println("Sweeping: "+transform1+" " +transform2);
    	//	System.out.println("Resulting AABB: "+aabb);
    	//}
    	if (broadPhase.inRange(aabb)) {
    		broadPhase.moveProxy(m_proxyId, aabb);
    		return true;
    	} else {
    		return false;
    	}
    }

    public void resetProxy(BroadPhase broadPhase, XForm transform){
    	if (m_proxyId != PairManager.NULL_PROXY){
    		broadPhase.destroyProxy(m_proxyId);
    	}

    	AABB aabb = new AABB();
    	computeAABB(aabb, transform);

    	boolean inRange = broadPhase.inRange(aabb);

    	// You are affecting a shape outside the world box.
    	assert(inRange);

    	if (inRange) {
    		m_proxyId = broadPhase.createProxy(aabb, this);
    	} else {
    		m_proxyId = PairManager.NULL_PROXY;
    	}
    }

    public static Shape create(ShapeDef def) {

        if (def.type == ShapeType.CIRCLE_SHAPE) {
            return new CircleShape(def);
        }
        else if (def.type == ShapeType.BOX_SHAPE
                || def.type == ShapeType.POLYGON_SHAPE) {
            return new PolygonShape(def);
        }
        assert false;
        return null;
    }
    
    public static void destroy(Shape s) {
        s.destructor();
    }

    public void destructor() {
        assert(m_proxyId == PairManager.NULL_PROXY);
    }
    
    public void createProxy(BroadPhase broadPhase, XForm transform) {
    	assert(m_proxyId == PairManager.NULL_PROXY);

    	AABB aabb = new AABB();
    	computeAABB(aabb, transform);

    	boolean inRange = broadPhase.inRange(aabb);

    	// You are creating a shape outside the world box.
    	assert(inRange);

    	if (inRange){
    		m_proxyId = broadPhase.createProxy(aabb, this);
    	} else {
    		m_proxyId = PairManager.NULL_PROXY;
    	}
    }
    
    public void destroyProxy(BroadPhase broadPhase) {
        if (m_proxyId != PairManager.NULL_PROXY) {
            broadPhase.destroyProxy(m_proxyId);
            m_proxyId = PairManager.NULL_PROXY;
        }
    }
    
}
