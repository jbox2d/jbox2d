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

import org.jbox2d.collision.AABB;
import org.jbox2d.collision.MassData;
import org.jbox2d.collision.Segment;
import org.jbox2d.collision.structs.SegmentCollide;
import org.jbox2d.collision.structs.ShapeType;
import org.jbox2d.collision.structs.TestSegmentResult;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;


//Updated through rev. 56->139-218 of b2Shape.cpp/.h

/**
 * A shape is used for collision detection. You can create a shape however you like.
 * Shapes used for simulation in World are created automatically when a Fixture
 * is created.
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
	public float m_radius;

	public Shape() {

		uid = uidcount++; //Java version only (C++ version sorts by memory location)
		m_type = ShapeType.UNKNOWN_SHAPE;
		/*
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
		m_isSensor = def.isSensor;*/
	}
	
	/**
	 * Get the type of this shape. You can use this to down cast to the concrete shape.
	 * @return the shape type.
	 */
	public ShapeType getType() {
		return m_type;
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
	                                    TestSegmentResult out,
	                                    Segment segment,
	                                    float maxLambda);

	/**
	 * Given a transform, compute the associated axis aligned bounding box for this shape.
	 * @param aabb returns the axis aligned box.
	 * @param xf the world transform of the shape.
	 */
	public abstract void computeAABB(AABB aabb, XForm xf);

	/**
	 * Compute the mass properties of this shape using its dimensions and density.
	 * The inertia tensor is computed about the local origin, not the centroid.
	 * @param massData returns the mass data for this shape.
	 * @param density the density in kilograms per meter squared.
	 */
	public abstract void computeMass(MassData massData, float density);

	
	/**
	 * Compute the volume and centroid of this shape intersected with a half plane
	 * @param normal the surface normal
	 * @param offset the surface offset along normal
	 * @param xf the shape transform
	 * @param c returns the centroid
	 * @return the total volume less than offset along normal
	 */
	public abstract float computeSubmergedArea(Vec2 normal,
									  float offset,
									  XForm xf, 
									  Vec2 c);

	
	 /**
	  * Compute the sweep radius. This is used for conservative advancement (continuous
	  * collision detection).
	  * @param pivot is the pivot point for rotation.
	  * @return the distance of the farthest point from the pivot.
	  */
	public abstract float computeSweepRadius(Vec2 pivot);
	
	public abstract void destructor();

	public abstract Vec2 getVertex(int indexA);
	
	public abstract int getSupport(Vec2 direction);
}
