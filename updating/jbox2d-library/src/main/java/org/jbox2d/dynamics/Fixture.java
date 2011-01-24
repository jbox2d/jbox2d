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
package org.jbox2d.dynamics;

import org.jbox2d.collision.AABB;
import org.jbox2d.collision.broadphase.BroadPhase;
import org.jbox2d.collision.broadphase.DynamicTreeNode;
import org.jbox2d.collision.shapes.MassData;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.collision.shapes.ShapeType;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.contacts.Contact;
import org.jbox2d.dynamics.contacts.ContactEdge;
import org.jbox2d.structs.collision.RayCastInput;
import org.jbox2d.structs.collision.RayCastOutput;

// updated to rev 100
// thread-safe pooling
/**
 * A fixture is used to attach a shape to a body for collision detection. A fixture
 * inherits its transform from its parent. Fixtures hold additional non-geometric data
 * such as friction, collision filters, etc.
 * Fixtures are created via b2Body::CreateFixture.
 * @warning you cannot reuse fixtures.
 *
 * @author daniel
 */
public class Fixture {

	public final AABB m_aabb = new AABB();
	
	public float m_density;
	
	public Fixture m_next;
	public Body m_body;
	
	public Shape m_shape;
	
	public float m_friction;
	public float m_restitution;
	
	public DynamicTreeNode m_proxy;
	public final Filter m_filter;
	
	public boolean m_isSensor;
	
	public Object m_userData;
	
	public Fixture(){
		m_userData = null;
		m_body = null;
		m_next = null;
		m_proxy = null;
		m_shape = null;
		m_filter = new Filter();
	}
	
	/**
	 * Get the type of the child shape. You can use this to down cast to the concrete shape.
	 * @return the shape type.
	 */
	public ShapeType getType(){
		return m_shape.getType();
	}
	
	/**
	 * Get the child shape. You can modify the child shape, however you should not change the
	 * number of vertices because this will crash some collision caching mechanisms.
	 * @return
	 */
	public Shape getShape(){
		return m_shape;
	}
	/**
	 * Is this fixture a sensor (non-solid)?
	 * @return the true if the shape is a sensor.
	 * @return
	 */
	public boolean isSensor(){
		return m_isSensor;
	}
	
	/**
	 * Set if this fixture is a sensor.
	 * @param sensor
	 */
	public void setSensor(boolean sensor){
		m_isSensor = sensor;
	}
	
	/**
	 * Set the contact filtering data. This is an expensive operation and should
	 * not be called frequently. This will not update contacts until the next time
	 * step when either parent body is awake.
	 * @param filter
	 */
	public void setFilterData(final Filter filter){
		m_filter.set(filter);
		
		if(m_body == null){
			return;
		}
		
		// Flag associated contacts for filtering.
		ContactEdge edge = m_body.getContactList();
		while (edge != null){
			Contact contact = edge.contact;
			Fixture fixtureA = contact.getFixtureA();
			Fixture fixtureB = contact.getFixtureB();
			if (fixtureA == this || fixtureB == this){
				contact.flagForFiltering();
			}
			edge = edge.next;
		}
	}
	
	/**
	 * Get the contact filtering data.
	 * @return
	 */
	public Filter getFilterData(){
		return m_filter;
	}
	
	/**
	 * Get the parent body of this fixture. This is NULL if the fixture is not attached.
	 * @return the parent body.
	 * @return
	 */
	public Body getBody(){
		return m_body;
	}
	
	/**
	 * Get the next fixture in the parent body's fixture list.
	 * @return the next shape.
	 * @return
	 */
	public Fixture getNext(){
		return m_next;
	}
	
	public void setDensity(float density){
		assert(density >= 0f);
		m_density = density;
	}
	
	public float getDensity(){
		return m_density;
	}
	
	/**
	 * Get the user data that was assigned in the fixture definition. Use this to
	 * store your application specific data.
	 * @return
	 */
	public Object getUserData(){
		return m_userData;
	}

	/**
	 * Set the user data. Use this to store your application specific data.
	 * @param data
	 */
	public void setUserData(Object data){
		m_userData = data;
	}
	
	/**
	 * Test a point for containment in this fixture. This only works for convex shapes.
	 * @param xf the shape world transform.
	 * @param p a point in world coordinates.
	 * @param p
	 * @return
	 */
	public boolean testPoint(final Vec2 p){
		return m_shape.testPoint( m_body.m_xf, p);
	}
	
	/**
	 * Cast a ray against this shape.
	 * @param output the ray-cast results.
	 * @param input the ray-cast input parameters.
	 * @param output
	 * @param input
	 */
	public boolean raycast(RayCastOutput output, RayCastInput input){
		return m_shape.raycast( output, input, m_body.m_xf);
	}

	/**
	 * Get the mass data for this fixture. The mass data is based on the density and
	 * the shape. The rotational inertia is about the shape's origin.
	 * @return
	 */
	public void getMassData(MassData massData){
		m_shape.computeMass(massData, m_density);
	}

	/**
	 * Get the coefficient of friction.
	 * @return
	 */
	public float getFriction(){
		return m_friction;
	}

	/**
	 * Set the coefficient of friction.
	 * @param friction
	 */
	public void setFriction(float friction){
		m_friction = friction;
	}

	/**
	 * Get the coefficient of restitution.
	 * @return
	 */
	public float getRestitution(){
		return m_restitution;
	}

	/**
	 * Set the coefficient of restitution.
	 * @param restitution
	 */
	public void setRestitution(float restitution){
		m_restitution = restitution;
	}
	
	/**
	 * Get the fixture's AABB. This AABB may be enlarge and/or stale.
	 * If you need a more accurate AABB, compute it using the shape and
	 * the body transform.
	 * @return
	 */
	public AABB getAABB(){
		return m_aabb;
	}
	
	
	// We need separation create/destroy functions from the constructor/destructor because
	// the destructor cannot access the allocator (no destructor arguments allowed by C++).
	
	public void create(Body body, FixtureDef def){
		m_userData = def.userData;
		m_friction = def.friction;
		m_restitution = def.restitution;
		
		m_body = body;
		m_next = null;
		
		m_filter.set(def.filter);
		
		m_isSensor = def.isSensor;
		
		m_shape = def.shape.clone();
		
		m_density = def.density;
	}
	
	public void destroy(){
		
		// The proxy must be destroyed before calling this.
		assert(m_proxy == null);
		
		// Free the child shape.
		// yeah woo jvm
		// TODO djm should I pool this then?
		m_shape = null;
	}
	
	// These support body activation/deactivation.
	public void createProxy(BroadPhase broadPhase, final Transform xf){
		assert(m_proxy == null);
		
		// Create proxy in the broad-phase.
		m_shape.computeAABB( m_aabb, xf);
		m_proxy = broadPhase.createProxy( m_aabb, this);
	}
	
	/**
	 * Internal method
	 * @param broadPhase
	 */
	public void destroyProxy(BroadPhase broadPhase){
		if(m_proxy == null){
			return;
		}
		
		broadPhase.destroyProxy( m_proxy);
		m_proxy = null;
	}
	
	private final AABB pool1 = new AABB();
	private final AABB pool2 = new AABB();
	
	/**
	 * Internal method
	 * @param broadPhase
	 * @param xf1
	 * @param xf2
	 */
	protected void synchronize(BroadPhase broadPhase, final Transform transform1, final Transform transform2){
		if(m_proxy == null){
			return;
		}
		
		// Compute an AABB that covers the swept shape (may miss some rotation effect).
		
//		AABB aabb1 = tlaabb1.get();
//		AABB aabb2 = tlaabb2.get();
//		
//		m_shape.computeAABB( aabb1, transform1);
//		m_shape.computeAABB( aabb2, transform2);
//		
//		m_aabb.combine(aabb1, aabb2);
//		
//		Vec2 disp = tldisp.get();
//		disp.set( transform2.position).subLocal(transform1.position);
//		
//		broadPhase.moveProxy( m_proxy, m_aabb, disp);
		
		m_shape.computeAABB( pool1, transform1);
		m_shape.computeAABB( pool2, transform2);
		m_aabb.lowerBound.x = pool1.lowerBound.x < pool2.lowerBound.x ? pool1.lowerBound.x : pool2.lowerBound.x;
		m_aabb.lowerBound.y = pool1.lowerBound.y < pool2.lowerBound.y ? pool1.lowerBound.y : pool2.lowerBound.y;
		m_aabb.upperBound.x = pool1.upperBound.x > pool2.upperBound.x ? pool1.upperBound.x : pool2.upperBound.x;
		m_aabb.upperBound.y = pool1.upperBound.y > pool2.upperBound.y ? pool1.upperBound.y : pool2.upperBound.y;
		
		final Vec2 disp = pool1.lowerBound; // just use this vec for pooling
		disp.x = transform2.position.x - transform1.position.x;
		disp.y = transform2.position.y - transform1.position.y;
		
		broadPhase.moveProxy( m_proxy, m_aabb, disp);
	}
}
