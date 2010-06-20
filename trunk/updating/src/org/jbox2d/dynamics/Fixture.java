package org.jbox2d.dynamics;

import org.jbox2d.collision.AABB;
import org.jbox2d.collision.broadphase.BroadPhase;
import org.jbox2d.collision.broadphase.DynamicTreeNode;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.contacts.Contact;
import org.jbox2d.pooling.TLAABB;
import org.jbox2d.pooling.TLVec2;
import org.jbox2d.structs.collision.RayCastInput;
import org.jbox2d.structs.collision.RayCastOutput;
import org.jbox2d.structs.collision.shapes.MassData;
import org.jbox2d.structs.collision.shapes.ShapeType;
import org.jbox2d.structs.dynamics.contacts.ContactEdge;

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
	
	public MassData m_massData;
	
	public Fixture m_next;
	public Body m_body;
	
	public Shape m_shape;
	
	public float m_friction;
	public float m_restitution;
	
	public DynamicTreeNode m_proxy;
	public Filter m_filter;
	
	public boolean m_isSensor;
	
	public Object m_userData;
	
	public Fixture(){
		m_userData = null;
		m_body = null;
		m_next = null;
		m_proxy = null;
		m_shape = null;
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
		if(m_isSensor == sensor){
			return;
		}
		
		m_isSensor = sensor;
		
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
				contact.setAsSensor(fixtureA.m_isSensor || fixtureB.m_isSensor);
			}
			edge = edge.next;
		}
	}
	
	/**
	 * Set the contact filtering data. This is an expensive operation and should
	 * not be called frequently. This will not update contacts until the next time
	 * step when either parent body is awake.
	 * @param filter
	 */
	public void setFilterData(final Filter filter){
		m_filter = filter;
		
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
	public void raycast(RayCastOutput output, RayCastInput input){
		m_shape.raycast( output, input, m_body.m_xf);
	}

	/**
	 * Get the mass data for this fixture. The mass data is based on the density and
	 * the shape. The rotational inertia is about the shape's origin.
	 * @return
	 */
	public MassData getMassData(){
		return m_massData;
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
	 * Internal method
	 * @param broadPhase
	 * @param body
	 * @param xf
	 * @param def
	 */
	public void create(BroadPhase broadPhase, Body body, final Transform xf, final FixtureDef def){
		m_userData = def.userData;
		m_friction = def.friction;
		m_restitution = def.restitution;
		
		m_body = body;
		m_next = null;
		
		m_filter = def.filter;
		
		m_isSensor = def.isSensor;
		
		m_shape = def.shape.clone();
		
		m_shape.computeMass( m_massData, def.density);
		
		// Create proxy in the broad-phase.
		m_shape.computeAABB( m_aabb, xf);
		
		m_proxy = broadPhase.createProxy( m_aabb, this);
	}
	
	/**
	 * Internal method
	 * @param broadPhase
	 */
	public void destroy(BroadPhase broadPhase){
		if(m_proxy != null){
			broadPhase.destroyProxy( m_proxy);
			m_proxy = null;
		}
		
		m_shape = null;
	}
	
	private final static TLAABB tlaabb1 = new TLAABB();
	private final static TLAABB tlaabb2 = new TLAABB();
	private final static TLVec2 tldisp = new TLVec2();
	/**
	 * Internal method
	 * @param broadPhase
	 * @param xf1
	 * @param xf2
	 */
	public void synchronize(BroadPhase broadPhase, final Transform transform1, final Transform transform2){
		if(m_proxy == null){
			return;
		}
		
		// Compute an AABB that covers the swept shape (may miss some rotation effect).
		AABB aabb1 = tlaabb1.get();
		AABB aabb2 = tlaabb2.get();
		
		m_shape.computeAABB( aabb1, transform1);
		m_shape.computeAABB( aabb2, transform2);
		
		Vec2 disp = tldisp.get();
		disp.set( transform2.position).subLocal(transform1.position);
		
		broadPhase.moveProxy( m_proxy, m_aabb, disp);
	}
}
