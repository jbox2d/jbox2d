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
import org.jbox2d.collision.BroadPhase;
import org.jbox2d.collision.FilterData;
import org.jbox2d.collision.MassData;
import org.jbox2d.collision.PairManager;
import org.jbox2d.collision.Segment;
import org.jbox2d.collision.structs.SegmentCollide;
import org.jbox2d.collision.structs.ShapeType;
import org.jbox2d.collision.structs.TestSegmentResult;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;
import org.jbox2d.dynamics.Body;


//Updated through rev. 56->139-[221 of groundzero] of Shape.cpp/.h

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


	protected ShapeType m_type;
	protected Shape m_next;
	protected Body m_body;

	/**
	 * Sweep radius relative to the parent body's center of mass.
	 */
	protected float m_sweepRadius;

	protected float  m_density;
	protected float m_friction;
	protected float m_restitution;

	protected short m_proxyId;
	protected FilterData m_filter;

	protected boolean m_isSensor;

	protected Object m_userData;

	public Shape(final ShapeDef def) {

		uid = uidcount++; //Java version only (C++ version sorts by memory location)
		m_type = ShapeType.UNKNOWN_SHAPE;

		m_userData = def.userData;
		m_friction = def.friction;
		m_restitution = def.restitution;
		m_density = def.density;
		m_body = null;
		m_sweepRadius = 0.0f;

		m_next = null;

		m_proxyId = PairManager.NULL_PROXY;

		m_filter = def.filter;

		m_isSensor = def.isSensor;
	}

	/**
	 * called when this shape is destroyed through
	 * {@link #destroy(Shape)}.  Override to do any cleanup.
	 */
	public void destructor(){
		assert(m_proxyId == PairManager.NULL_PROXY);
	}


	/**
	 * Get the type of this shape. You can use this to down cast to the concrete shape.
	 * @return the shape type.
	 */
	public final ShapeType getType() {
		return m_type;
	}

	/**
	 * Get the maximum radius about the parent body's center of mass.
	 * @return
	 */
	public final float getSweepRadius(){
		return m_sweepRadius;
	}

	/**
	 * Get the coefficient of friction.
	 * @return
	 */
	public final float getFriction(){
		return m_friction;
	}

	/**
	 * Set the coefficient of friction.
	 * @param friction
	 */
	public final void setFriction(final float friction){
		m_friction = friction;
	}

	/**
	 * Get the coefficient of restitution.
	 * @return
	 */
	public final float getRestitution(){
		return m_restitution;
	}

	/**
	 * Set the coefficient of restitution.
	 * @param restitution
	 */
	public final void setRestitution(final float restitution){
		m_restitution = restitution;
	}

	/**
	 * Get the density of the shape.
	 * @return
	 */
	public final float getDensity(){
		return m_density;
	}

	/**
	 * Set the density of the shape.
	 * @param density
	 */
	public final void setDensity(final float density){
		m_density = density;
	}

	/**
	 * Is this shape a sensor (non-solid)?
	 * @return the true if the shape is a sensor.
	 */
	public final boolean isSensor(){
		return m_isSensor;
	}

	/**
	 *  Set the contact filtering data. You must call b2World::Refilter to correct
	 * existing contacts/non-contacts.
	 * @param filter
	 */
	public final void setFilterData(final FilterData filter){
		m_filter = filter;
	}

	/**
	 * Get the contact filtering data.
	 * @return
	 */
	public final FilterData getFilterData(){
		return m_filter;
	}

	/**
	 * Get the parent body of this shape. This is NULL if the shape is not attached.
	 * @return the parent body.
	 * @return
	 */
	public final Body getBody(){
		return m_body;
	}

	/**
	 * Get the next shape in the parent body's shape list.
	 * @return the next shape.
	 * @return
	 */
	public Shape getNext(){
		return m_next;
	}

	/**
	 * Get the user data that was assigned in the shape definition. Use this to
	 * store your application specific data.
	 */
	public Object getUserData(){
		return m_userData;
	}

	/**
	 * Set the user data. Use this to store your application specific data.
	 * @param data
	 */
	public void setUserData(final Object data){
		m_userData = data;
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
	 * Given two transforms, compute the associated swept axis aligned bounding box for this shape.
	 * @param aabb returns the axis aligned box.
	 * @param xf1 the starting shape world transform.
	 * @param xf2 the ending shape world transform.
	 */
	public abstract void computeSweptAABB(AABB aabb, XForm xf1, XForm xf2);


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
	
	protected abstract void updateSweepRadius(Vec2 center);

	// DJM pooling
	private static final AABB createAabb = new AABB();
	protected final void createProxy(final BroadPhase broadPhase, final XForm xf){
		assert(m_proxyId == PairManager.NULL_PROXY);

		computeAABB(createAabb, xf);

		final boolean inRange = broadPhase.inRange( createAabb);

		// You are creating a shape outside the world box.
		assert(inRange);

		if(inRange){
			m_proxyId = broadPhase.createProxy( createAabb, this);
		}else{
			m_proxyId = PairManager.NULL_PROXY;
		}
	}

	protected final void destroyProxy(final BroadPhase broadPhase){
		if(m_proxyId != PairManager.NULL_PROXY){
			broadPhase.destroyProxy( m_proxyId);
			m_proxyId = PairManager.NULL_PROXY;
		}
	}

	private static final AABB syncAABB = new AABB();
	protected final boolean synchronize(final BroadPhase broadPhase, final XForm xf1, final XForm xf2){
		if(m_proxyId == PairManager.NULL_PROXY){
			return false;
		}

		computeSweptAABB(syncAABB, xf1, xf2);

		if(broadPhase.inRange( syncAABB)){
			broadPhase.moveProxy( m_proxyId, syncAABB);
			return true;
		}else{
			return false;
		}
	}

	private static final AABB refilterAABB = new AABB();
	protected final void refilterProxy(final BroadPhase broadPhase, final XForm xf){
		if(m_proxyId == PairManager.NULL_PROXY){
			return;
		}

		broadPhase.destroyProxy( m_proxyId);

		computeAABB(refilterAABB, xf);

		final boolean inRange = broadPhase.inRange( createAabb);

		if(inRange){
			m_proxyId = broadPhase.createProxy( createAabb, this);
		}else{
			m_proxyId = PairManager.NULL_PROXY;
		}
	}
	

	// TODO djm: stack pooling
	public static final Shape create(final ShapeDef def){
		switch(def.type){
			case CIRCLE_SHAPE:
				return new CircleShape(def);
				break;
			case POLYGON_SHAPE:
				return new PolygonSahpe(def);
				break;
			default:
				assert(false);
				return null;
		}
	}

	// TODO djm: stack pooling
	public static final void destroy(final Shape shape){
		switch(shape.m_type){
			case CIRCLE_SHAPE:
			case POLYGON_SHAPE:
				shape.destructor();
				break;
			case EDGE_SHAPE:
				final EdgeShape edge = (EdgeShape) shape;
				if(edge.m_nextEdge != null){
					edge.m_nextEdge.m_prevEdge = null;
				}
				if(edge.m_prevEdge != null){
					edge.m_prevEdge.m_nextEdge = null;
				}
				edge.destructor();
				break;
			default:
				assert(false);
		}

	}
}
