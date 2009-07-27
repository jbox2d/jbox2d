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

package org.jbox2d.collision.shapes;

import java.util.HashSet;
import java.util.Set;

import org.jbox2d.collision.AABB;
import org.jbox2d.collision.BroadPhase;
import org.jbox2d.collision.FilterData;
import org.jbox2d.collision.MassData;
import org.jbox2d.collision.PairManager;
import org.jbox2d.collision.Segment;
import org.jbox2d.collision.SegmentCollide;
import org.jbox2d.common.RaycastResult;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.contacts.Contact;
import org.jbox2d.dynamics.contacts.ContactEdge;
import org.jbox2d.pooling.TLAABB;


//Updated through rev. 56->139 of b2Shape.cpp/.h

/**
 * A shape is used for collision detection. Shapes are created in World.
 * You can use shape for collision detection before they are attached to the world.
 * <BR><BR><em>Warning</em>: you cannot reuse shapes on different bodies, they must
 * be re-created or copied.
 */
public abstract class Shape {
	/** Unique id for shape for sorting (C++ version uses memory address) */
	public int uid;
	/**
	 * Used to generate uids - not initialized on applet reload,
	 * but that's okay since these just have to be unique.
	 */
	static private int uidcount = 0;

	public ShapeType m_type;
	public Shape m_next;
	public Body m_body;

	/** Sweep radius relative to the parent body's center of mass. */
	public float m_sweepRadius;

	public float m_density;
	public float m_friction;
	public float m_restitution;


	public int m_proxyId;

	public FilterData m_filter;

	public boolean m_isSensor;
	public Object m_userData;

	public Shape(final ShapeDef def) {

		uid = uidcount++; //Java version only (C++ version sorts by memory location)

		m_userData = def.userData;
		m_friction = def.friction;
		m_restitution = def.restitution;
		m_density = def.density;
		m_body = null;
		m_sweepRadius = 0.0f;
		m_next = null;
		m_proxyId = PairManager.NULL_PROXY;
		m_filter = new FilterData();
		m_filter.categoryBits = def.filter.categoryBits;
		m_filter.maskBits = def.filter.maskBits;
		m_filter.groupIndex = def.filter.groupIndex;
		m_isSensor = def.isSensor;

	}

	/** Get the coefficient of friction. */
	public float getFriction() {
		return m_friction;
	}

	/** Set the coefficient of friction. */
	public void setFriction(final float friction) {
		m_friction = friction;
	}

	/** Get the coefficient of restitution. */
	public float getRestitution() {
		return m_restitution;
	}

	/** Set the coefficient of restitution. */
	public void setRestitution(final float restitution) {
		m_restitution = restitution;
	}

	/** Set the collision filtering data. */
	public void setFilterData(final FilterData filter){
		m_filter.set(filter);
	}

	/** Get the collision filtering data. */
	public FilterData getFilterData() {
		return m_filter;
	}

	/**
	 * Get the type of this shape. You can use this to down cast to the concrete shape.
	 * @return the shape type.
	 */
	public ShapeType getType() {
		return m_type;
	}

	/**
	 * Is this shape a sensor (non-solid)?
	 * @return the true if the shape is a sensor.
	 */
	public boolean isSensor() {
		return m_isSensor;
	}

	/**
	 * Get the user data that was assigned in the shape definition. Use this to
	 * store your application specific data.
	 */
	public Object getUserData() {
		return m_userData;
	}

	/**
	 * Set the user data associated with the object.
	 * @param o User data to set
	 */
	public void setUserData(final Object o) {
		m_userData = o;
	}

	/**
	 * Get the parent body of this shape. This is NULL if the shape is not attached.
	 * @return the parent body.
	 */
	public Body getBody() {
		return m_body;
	}

	/**
	 * Get the next shape in the parent body's shape list.
	 * @return the next shape.
	 */
	public Shape getNext() {
		return m_next;
	}

	/**
	 * Get the sweep radius of the shape.
	 * @return the sweep radius
	 */
	public float getSweepRadius() {
		return m_sweepRadius;
	}

	/**
	 * Test a point for containment in this shape. This only works for convex shapes.
	 * @param xf the shape world transform.
	 * @param p a point in world coordinates.
	 * @return true if the point is within the shape
	 */
	public abstract boolean testPoint(XForm xf, Vec2 p);


	/**
	 *  Perform a ray cast against this shape.
	 *  @param xf the shape world transform.
	 *  @param out is where the results are placed: <ul><li>lambda returns the hit fraction, based on
	 *  the distance between the two points. You can use this to compute the contact point
	 *  p = (1 - lambda) * segment.p1 + lambda * segment.p2.</li>
	 *  <li>normal returns the normal at the contact point. If there is no intersection, the normal
	 *  is not set.</li></ul>
	 *  @param segment defines the begin and end point of the ray cast.
	 *  @param maxLambda a number typically in the range [0,1].
	 *  @return true if there was an intersection.
	 */
	public abstract SegmentCollide testSegment(XForm xf,
	                                    RaycastResult out,
	                                    Segment segment,
	                                    float maxLambda);

	/**
	 * Given a transform, compute the associated axis aligned bounding box for this shape.
	 * @param aabb returns the axis aligned box.
	 * @param xf the world transform of the shape.
	 */
	public abstract void computeAABB(AABB aabb, XForm xf);

	/**
	 * Given two transforms, compute the associated swept axis aligned bounding box for this shape.
	 * @param aabb returns the axis aligned box. (return parameter)
	 * @param xf1 the starting shape world transform.
	 * @param xf2 the ending shape world transform.
	 */
	public abstract void computeSweptAABB(AABB aabb,
	                                      XForm xf1,
	                                      XForm xf2);

	/**
	 * Compute the mass properties of this shape using its dimensions and density.
	 * The inertia tensor is computed about the local origin, not the centroid.
	 * @param massData returns the mass data for this shape. (return parameter)
	 */
	public abstract void computeMass(MassData massData);


	/* INTERNALS BELOW */
	/** Internal */
	public abstract void updateSweepRadius(Vec2 center);

	// djm pooling
	private static final TLAABB tlAabb = new TLAABB();
	/** Internal */
	public boolean synchronize(final BroadPhase broadPhase, final XForm transform1, final XForm transform2) {
		if (m_proxyId == PairManager.NULL_PROXY) {
			return false;
		}

		// Compute an AABB that covers the swept shape (may miss some rotation effect).
		AABB aabb = tlAabb.get();
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

	/** Internal */
	public void refilterProxy(final BroadPhase broadPhase, final XForm transform){
		if (m_proxyId == PairManager.NULL_PROXY){
			return;
		}

		broadPhase.destroyProxy(m_proxyId);
		// djm don't pool this, it could be used to
		// create a proxy
		final AABB aabb = new AABB();
		computeAABB(aabb, transform);

		final boolean inRange = broadPhase.inRange(aabb);

		if (inRange) {
			m_proxyId = broadPhase.createProxy(aabb, this);
		} else {
			m_proxyId = PairManager.NULL_PROXY;
		}
	}

	/** Internal */
	public static Shape create(final ShapeDef def) {

		if (def.type == ShapeType.CIRCLE_SHAPE) {
			return new CircleShape(def);
		}
		else if (def.type == ShapeType.POLYGON_SHAPE) {
			return new PolygonShape(def);
		}
		else if (def.type == ShapeType.POINT_SHAPE) {
			return new PointShape(def);
		}
		assert false;
		return null;
	}

	/** Internal */
	public static void destroy(final Shape s) {
		if (s.getType() == ShapeType.EDGE_SHAPE) {
			final EdgeShape edge = (EdgeShape)s;
			if (edge.m_nextEdge != null) {
				edge.m_nextEdge.m_prevEdge = null;
			}
			if (edge.m_prevEdge != null) {
				edge.m_prevEdge.m_nextEdge = null;
			}
		}

		s.destructor();
	}

	/** Internal */
	public void destructor() {
		assert(m_proxyId == PairManager.NULL_PROXY);
	}

	/** Internal */
	public void createProxy(final BroadPhase broadPhase, final XForm transform) {
		assert(m_proxyId == PairManager.NULL_PROXY);

		// djm don't pool this,
		// could be used
		final AABB aabb = new AABB();
		computeAABB(aabb, transform);

		final boolean inRange = broadPhase.inRange(aabb);

		// You are creating a shape outside the world box.
		assert(inRange);

		if (inRange){
			m_proxyId = broadPhase.createProxy(aabb, this);
		} else {
			m_proxyId = PairManager.NULL_PROXY;
		}
	}

	/** Internal */
	public void destroyProxy(final BroadPhase broadPhase) {
		if (m_proxyId != PairManager.NULL_PROXY) {
			broadPhase.destroyProxy(m_proxyId);
			m_proxyId = PairManager.NULL_PROXY;
		}
	}

	/**
	 * Compute the volume and centroid of this fixture intersected with a half plane
	 * @param normal the surface normal
	 * @param offset the surface offset along normal
	 * @param c returns the centroid
	 * @return the total volume less than offset along normal
	 */
	public float computeSubmergedArea(Vec2 normal, float offset, Vec2 c) {
		return this.computeSubmergedArea(normal, offset, m_body.getXForm(), c);
	}

	/**
	 * @param normal
	 * @param offset
	 * @param form
	 * @param c
	 * @return
	 */
	public float computeSubmergedArea(Vec2 normal, float offset, XForm form, Vec2 c) {
		return 0;
	}

	/**
	 * @return shape density
	 */
	public float getDensity() {
		return m_density;
	}
	
	/**
	 * @return a Set<Shape> of all shapes in contact with this one
	 */
	public Set<Shape> getShapesInContact() {
		ContactEdge curr = this.m_body.getContactList();
		Set<Shape> touching = new HashSet<Shape>();
		while (curr != null) {
			if (curr.contact.m_shape1 == this) {
				touching.add(curr.contact.m_shape2);
			} else if (curr.contact.m_shape2 == this) {
				touching.add(curr.contact.m_shape1);
			}
			curr = curr.next;
		}
		return touching;
	}
	
	/**
	 * @return a Set<Contact> of all (active) contacts involving this shape
	 */
	public Set<Contact> getContacts() {
		ContactEdge curr = this.m_body.getContactList();
		Set<Contact> contacts = new HashSet<Contact>();
		while (curr != null) {
			if (curr.contact.getManifoldCount() > 0) {
				if (curr.contact.m_shape1 == this) {
					contacts.add(curr.contact);
				} else if (curr.contact.m_shape2 == this) {
					contacts.add(curr.contact);
				}
			}
			curr = curr.next;
		}
		return contacts;
	}
}
