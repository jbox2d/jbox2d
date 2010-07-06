package org.jbox2d.dynamics.contacts;

import org.jbox2d.collision.AABB;
import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Sweep;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.pooling.SingletonPool;
import org.jbox2d.pooling.TLManifold;
import org.jbox2d.pooling.TLTOIInput;
import org.jbox2d.structs.collision.ContactID;
import org.jbox2d.structs.collision.Manifold;
import org.jbox2d.structs.collision.ManifoldPoint;
import org.jbox2d.structs.collision.TOIInput;
import org.jbox2d.structs.collision.WorldManifold;
import org.jbox2d.structs.collision.shapes.ShapeType;
import org.jbox2d.structs.dynamics.contacts.ContactCreateFcn;
import org.jbox2d.structs.dynamics.contacts.ContactEdge;
import org.jbox2d.structs.dynamics.contacts.ContactRegister;

/**
 * The class manages contact between two shapes. A contact exists for each overlapping
 * AABB in the broad-phase (except if filtered). Therefore a contact object may exist
 * that has no contact points.
 *
 * @author daniel
 */
public abstract class Contact {
	// Flags stored in m_flags
	// Used when crawling contact graph when forming islands.
	public static final int ISLAND_FLAG		= 0x0001;
    // Set when the shapes are touching.
	public static final int TOUCHING_FLAG	= 0x0002;
	// This contact can be disabled (by user)
	public static final int ENABLED_FLAG		= 0x0004;
	// This contact needs filtering because a fixture filter was changed.
	public static final int FILTER_FLAG			= 0x0008;
	// This bullet contact had a TOI event
	public static final int BULLET_HIT_FLAG		= 0x0010;
	
	public static final ContactRegister[][] s_registers = new ContactRegister[ShapeType.TYPE_COUNT][ShapeType.TYPE_COUNT];
	public static boolean s_initialized = false;
	
	public static void addType(ContactCreateFcn createFcn, ShapeType type1, ShapeType type2){
		assert( org.jbox2d.structs.collision.shapes.intValue < type1.intValue && type1.intValue < ShapeType.TYPE_COUNT);
		assert( org.jbox2d.structs.collision.shapes.intValue < type2.intValue && type2.intValue < ShapeType.TYPE_COUNT);
		
		ContactRegister register = new ContactRegister();
		register.createFcn = createFcn;
		register.s1 = type1;
		register.s2 = type2;
		register.primary = true;
		s_registers[type1.intValue][type2.intValue] = register;
		
		if(type1 != type2){
			ContactRegister register2 = new ContactRegister();
			register2.createFcn = createFcn;
			register2.s1 = type2;
			register2.s2 = type1;
			register.primary = false;
			s_registers[type2.intValue][type1.intValue] = register;
		}
	}
	
	public static void initializeRegisters(){
		addType(new CircleContact(), org.jbox2d.structs.collision.shapes.CIRCLE_SHAPE, org.jbox2d.structs.collision.shapes.CIRCLE_SHAPE);
		addType(new PolygonAndCircleContact(), org.jbox2d.structs.collision.shapes.POLYGON_SHAPE, org.jbox2d.structs.collision.shapes.CIRCLE_SHAPE);
		addType(new PolygonContact(), org.jbox2d.structs.collision.shapes.POLYGON_SHAPE, org.jbox2d.structs.collision.shapes.POLYGON_SHAPE);
	}
	
	public static Contact create(Fixture fixtureA, Fixture fixtureB){
		if(s_initialized == false){
			initializeRegisters();
			s_initialized = true;
		}
		
		ShapeType type1 = fixtureA.getType();
		ShapeType type2 = fixtureB.getType();
		
		assert( org.jbox2d.structs.collision.shapes.intValue < type1.intValue && type1.intValue < ShapeType.TYPE_COUNT);
		assert( org.jbox2d.structs.collision.shapes.intValue < type2.intValue && type2.intValue < ShapeType.TYPE_COUNT);
		
		ContactCreateFcn createFcn = s_registers[type1.intValue][type2.intValue].createFcn;
		if(createFcn != null){
			if( s_registers[type1.intValue][type2.intValue].primary){
				return createFcn.contactCreateFcn(fixtureA, fixtureB);
			}else{
				return createFcn.contactCreateFcn(fixtureB, fixtureA);
			}
		}else{
			return null;
		}
	}
	
	public static void destroy(Contact contact, ShapeType typeA, ShapeType typeB){
		
	}
	
	public static void destroy(Contact contact){
		assert(s_initialized == true);
		
		if(contact.m_manifold.pointCount > 0){
			contact.getFixtureA().getBody().wakeUp();
			contact.getFixtureB().getBody().wakeUp();
		}
		
		ShapeType type1 = contact.getFixtureA().getType();
		ShapeType type2 = contact.getFixtureB().getType();
		
		assert( org.jbox2d.structs.collision.shapes.intValue < type1.intValue && type1.intValue < ShapeType.TYPE_COUNT);
		assert( org.jbox2d.structs.collision.shapes.intValue < type2.intValue && type2.intValue < ShapeType.TYPE_COUNT);
		
		ContactCreateFcn destoryFcn = s_registers[type1.intValue][type2.intValue].createFcn;
		destoryFcn.contactDestroyFcn(contact);
	}
	
	
	protected int m_flags;
	
	// World pool and list pointers.
	protected Contact m_prev;
	protected Contact m_next;

	// Nodes for connecting bodies.
	protected ContactEdge m_nodeA;
	protected ContactEdge m_nodeB;

	protected Fixture m_fixtureA;
	protected Fixture m_fixtureB;

	protected Manifold m_manifold;

	protected float m_toi;
	
	
	protected Contact(){
		m_fixtureA = null;
		m_fixtureB = null;
	}
	
	protected Contact(Fixture fA, Fixture fB){
		m_flags = 0;
		
		if( fA.isSensor() || fB.isSensor() ){
			m_flags |= e_sensorFlag;
		}
		
		Body bodyA = fA.getBody();
		Body bodyB = fB.getBody();

		if (bodyA.isStatic() || bodyA.isBullet() || bodyB.isStatic() || bodyB.isBullet()){
			m_flags |= e_continuousFlag;
		}
		else{
			m_flags &= ~e_continuousFlag;
		}

		m_fixtureA = fA;
		m_fixtureB = fB;

		m_manifold = new Manifold();
		m_manifold.pointCount = 0;

		m_prev = null;
		m_next = null;

		m_nodeA = new ContactEdge();
		m_nodeA.contact = null;
		m_nodeA.prev = null;
		m_nodeA.next = null;
		m_nodeA.other = null;

		m_nodeB = new ContactEdge();
		m_nodeB.contact = null;
		m_nodeB.prev = null;
		m_nodeB.next = null;
		m_nodeB.other = null;
	}
	
	
	/**
	 * Get the contact manifold. Do not set the point count to zero. Instead
	 * call Disable.
	 */
	public Manifold getManifold(){
		return m_manifold;
	}

	/**
	 * Get the world manifold.
	 */
	public void getWorldManifold(WorldManifold worldManifold){
		final Body bodyA = m_fixtureA.getBody();
		final Body bodyB = m_fixtureB.getBody();
		final Shape shapeA = m_fixtureA.getShape();
		final Shape shapeB = m_fixtureB.getShape();
		
		worldManifold.initialize(m_manifold, bodyA.getTransform(), shapeA.m_radius, bodyB.getTransform(), shapeB.m_radius);
		
	}

	/**
	 * Is this contact solid? Returns false if the shapes are separate,
	 * sensors, or the contact has been disabled.
	 * @return true if this contact should generate a response.
	 */
	public boolean isSolid(){
		int nonSolid = e_sensorFlag | e_disabledFlag;
		return (m_flags & nonSolid) == 0;
	}

	/**
	 * Is this contact touching.
	 * @return
	 */
	public boolean isTouching(){
		return (m_flags & e_touchingFlag) != 0;
	}

	/**
	 * Does this contact generate TOI events for continuous simulation?
	 * @return
	 */
	public boolean isContinuous(){
		return (m_flags & e_continuousFlag) != 0;
	}

    /**
     * Change this to be a sensor or non-sensor contact.
     * @param sensor
     */
	public void setAsSensor(boolean sensor){
		if(sensor){
			m_flags |= e_sensorFlag;
		}else{
			m_flags &= ~e_sensorFlag;
		}
	}

	/**
	 * Disable this contact. This can be used inside the pre-solve
	 * contact listener. The contact is only disabled for the current
	 * time step (or sub-step in continuous collisions).
	 */
	public void disable(){
		m_flags |= e_disabledFlag;
	}

	/**
	 * Get the next contact in the world's contact list.
	 * @return
	 */
	public Contact getNext(){
		return m_next;
	}

	/**
	 * Get the first fixture in this contact.
	 * @return
	 */
	public Fixture getFixtureA(){
		return m_fixtureA;
	}

	/**
	 * Get the second fixture in this contact.
	 * @return
	 */
	public Fixture getFixtureB(){
		return m_fixtureB;
	}

	/**
	 * Flag this contact for filtering. Filtering will occur the next time step.
	 */
	public void flagForFiltering(){
		m_flags |= e_filterFlag;
	}
	
	// djm pooling
	private static final TLManifold tloldManifold = new TLManifold();
	
	protected void update(ContactListener listener){
		
		Manifold oldManifold = tloldManifold.get();
		oldManifold.set(m_manifold);
		
		// Re-enable this contact.
		m_flags &= ~e_disabledFlag;

		if ( AABB.testOverlap(m_fixtureA.m_aabb, m_fixtureB.m_aabb)){
			evaluate();
		}
		else{
			m_manifold.pointCount = 0;
		}

		Body bodyA = m_fixtureA.getBody();
		Body bodyB = m_fixtureB.getBody();

		int oldCount = oldManifold.pointCount;
		int newCount = m_manifold.pointCount;

		if (newCount == 0 && oldCount > 0){
			bodyA.wakeUp();
			bodyB.wakeUp();
		}

		// Slow contacts don't generate TOI events.
		if (bodyA.isStatic() || bodyA.isBullet() || bodyB.isStatic() || bodyB.isBullet()){
			m_flags |= e_continuousFlag;
		}
		else{
			m_flags &= ~e_continuousFlag;
		}

		// Match old contact ids to new contact ids and copy the
		// stored impulses to warm start the solver.
		for (int i = 0; i < m_manifold.pointCount; ++i){
			ManifoldPoint mp2 = m_manifold.points[i];
			mp2.normalImpulse = 0.0f;
			mp2.tangentImpulse = 0.0f;
			ContactID id2 = mp2.id;

			for (int j = 0; j < oldManifold.pointCount; ++j){
				ManifoldPoint mp1 = oldManifold.points[j];

				if (mp1.id.key == id2.key){
					mp2.normalImpulse = mp1.normalImpulse;
					mp2.tangentImpulse = mp1.tangentImpulse;
					break;
				}
			}
		}

		if (newCount > 0){
			m_flags |= e_touchingFlag;
		}
		else{
			m_flags &= ~e_touchingFlag;
		}

		if (oldCount == 0 && newCount > 0){
			listener.beginContact(this);
		}

		if (oldCount > 0 && newCount == 0){
			listener.endContact(this);
		}

		if ((m_flags & e_sensorFlag) == 0){
			// djm NOTE: this manifold is pooled, don't modify it
			listener.preSolve(this, oldManifold);
		}
	}
	
	protected abstract void evaluate();

	// djm pooled
	private static final TLTOIInput tlinput = new TLTOIInput();
	
	protected float computeTOI(Sweep sweepA, Sweep sweepB){
		TOIInput input = tlinput.get();
		input.proxyA.set(m_fixtureA.getShape());
		input.proxyB.set(m_fixtureB.getShape());
		input.sweepA = sweepA;
		input.sweepB = sweepB;
		input.tolerance = Settings.linearSlop;

		return SingletonPool.getTOI().timeOfImpact(input);
	}
}
