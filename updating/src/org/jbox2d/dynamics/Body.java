package org.jbox2d.dynamics;

import org.jbox2d.collision.broadphase.BroadPhase;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.Sweep;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.contacts.Contact;
import org.jbox2d.pooling.TLFixtureDef;
import org.jbox2d.pooling.TLTransform;
import org.jbox2d.pooling.TLVec2;
import org.jbox2d.structs.collision.shapes.MassData;
import org.jbox2d.structs.dynamics.contacts.ContactEdge;

/**
 * A rigid body. These are created via World.createBody.
 */

public class Body {
	public static final int e_islandFlag = 0x0001;
	public static final int e_sleepFlag = 0x0002;
	public static final int e_allowSleepFlag = 0x0004;
	public static final int e_bulletFlag = 0x0008;
	public static final int e_fixedRotationFlag = 0x0010;
	
	public static final int e_staticType = 0;
	public static final int e_dynamicType = 1;
	public static final int e_maxType = 2;
	
	
	public int m_flags;
	public int m_type;
	
	public int m_islandIndex;
	
	/**
	 * The body orgin transform.
	 */
	public final Transform m_xf = new Transform();
	
	/**
	 * The swept motion for CCD
	 */
	public final Sweep m_sweep = new Sweep();
	
	public final Vec2 m_linearVelocity = new Vec2();
	
	public float m_angularVelocity = 0;
	
	public final Vec2 m_force = new Vec2();
	public float m_torque = 0;
	
	public World m_world;
	
	public Body m_prev;
	public Body m_next;
	
	public Fixture m_fixtureList;
	public int m_fixtureCount;
	
	public Joint m_jointList;
	public ContactEdge m_contactList;
	
	public float m_mass, m_invMass;
	public float m_I, m_invI;
	
	public float m_linearDamping;
	public float m_angularDamping;
	
	public float m_sleeptime;
	
	public Object m_userData;
	
	public Body(final BodyDef bd, World world){
		m_flags = 0;
		
		if (bd.isBullet){
			m_flags |= e_bulletFlag;
		}
		if (bd.fixedRotation){
			m_flags |= e_fixedRotationFlag;
		}
		if (bd.allowSleep){
			m_flags |= e_allowSleepFlag;
		}
		if (bd.isSleeping){
			m_flags |= e_sleepFlag;
		}
		
		m_world = world;
		
		m_xf.position = bd.position;
		m_xf.R.set(bd.angle);

		m_sweep.localCenter.setZero();
		m_sweep.alpha0 = 1.0f;
		m_sweep.a0 = m_sweep.a = bd.angle;
		//m_sweep.c0 = m_sweep.c = Transform.mul(m_xf, m_sweep.localCenter);
		Transform.mulToOut(m_xf, m_sweep.localCenter, m_sweep.c0);
		m_sweep.c.set(m_sweep.c0);
		
		m_jointList = null;
		m_contactList = null;
		m_prev = null;
		m_next = null;

		m_linearVelocity = bd.linearVelocity;
		m_angularVelocity = bd.angularVelocity;

		m_linearDamping = bd.linearDamping;
		m_angularDamping = bd.angularDamping;

		m_force.set(0.0f, 0.0f);
		m_torque = 0.0f;

		m_sleeptime = 0.0f;

		m_mass = 0.0f;
		m_invMass = 0.0f;
		m_I = 0.0f;
		m_invI = 0.0f;

		m_type = e_staticType;

		m_userData = bd.userData;

		m_fixtureList = null;
		m_fixtureCount = 0;
	}
	
	/**
	 * Creates a fixture and attach it to this body. Use this function if you need
	 * to set some fixture parameters, like friction. Otherwise you can create the
	 * fixture directly from a shape.
	 * This function automatically updates the mass of the body.
	 * @param def the fixture definition.
	 * @warning This function is locked during callbacks.
	 */
	public final Fixture createFixture( FixtureDef def){
		assert(m_world.isLocked() == false);
		if (m_world.isLocked() == true){
			return null;
		}

		BroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;

		Fixture fixture = new Fixture();
		fixture.create(broadPhase, this, m_xf, def);

		fixture.m_next = m_fixtureList;
		m_fixtureList = fixture;
		++m_fixtureCount;

		fixture.m_body = this;

		boolean needMassUpdate = fixture.m_massData.mass > 0.0f || fixture.m_massData.I > 0.0f;

		// Adjust mass properties if needed.
		if (needMassUpdate){
			resetMass();
		}

		// Let the world know we have a new fixture. This will cause new contacts
		// to be created at the beginning of the next time step.
		m_world.m_flags |= World.e_newFixture;

		return fixture;
	}
	
	// djm pooling
	private static final TLFixtureDef tldef = new TLFixtureDef();
	/**
	 * Creates a fixture from a shape and attach it to this body.
	 * This is a convenience function. Use FixtureDef if you need to set parameters
	 * like friction, restitution, user data, or filtering.
	 * This function automatically updates the mass of the body.
	 * @param shape the shape to be cloned.
	 * @param density the shape density (set to zero for static bodies).
	 * @warning This function is locked during callbacks.
	 */
	public final Fixture createFixture( Shape shape, float density){
		FixtureDef def = tldef.get();
		def.shape = shape;
		def.density = density;

		return createFixture(def);
	}

	/**
	 * Destroy a fixture. This removes the fixture from the broad-phase and
	 * therefore destroys any contacts associated with this fixture. All fixtures
	 * attached to a body are implicitly destroyed when the body is destroyed.
	 * @param fixture the fixture to be removed.
	 * @warning This function is locked during callbacks.
	 */
	public final void destroyFixture(Fixture fixture){
		assert(m_world.isLocked() == false);
		if (m_world.isLocked() == true){
			return;
		}

		assert(fixture.m_body == this);

		// Remove the fixture from this body's singly linked list.
		assert(m_fixtureCount > 0);
		Fixture node = m_fixtureList;
		boolean found = false;
		while (node != null)
		{
			if (node == fixture){
				node = fixture.m_next;
				found = true;
				break;
			}

			node = node.m_next;
		}

		// You tried to remove a shape that is not attached to this body.
		assert(found);

		// Destroy any contacts associated with the fixture.
		ContactEdge edge = m_contactList;
		while (edge != null){
			Contact c = edge.contact;
			edge = edge.next;

			Fixture fixtureA = c.getFixtureA();
			Fixture fixtureB = c.getFixtureB();

			if (fixture == fixtureA || fixture == fixtureB){
				// This destroys the contact and removes it from
				// this body's contact list.
				m_world.m_contactManager.Destroy(c);
			}
		}

		boolean needMassUpdate = fixture.m_massData.mass > 0.0f || fixture.m_massData.I > 0.0f;

		BroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;

		fixture.destroy(broadPhase);
		fixture.m_body = null;
		fixture.m_next = null;
		//fixture.~Fixture();
		//allocator.Free(fixture, sizeof(Fixture));

		--m_fixtureCount;

		// Adjust mass properties if needed.
		if (needMassUpdate){
			resetMass();
		}
	}

	/**
	 * Set the position of the body's origin and rotation.
	 * This breaks any contacts and wakes the other bodies.
	 * @param position the world position of the body's local origin.
	 * @param angle the world rotation in radians.
	 */
	public final void setTransform( Vec2 position, float angle){
		assert(m_world.isLocked() == false);
		if (m_world.isLocked() == true){
			return;
		}

		m_xf.R.set(angle);
		m_xf.position = position;

		//m_sweep.c0 = m_sweep.c = Mul(m_xf, m_sweep.localCenter);
		Transform.mulToOut(m_xf, m_sweep.localCenter, m_sweep.c0);
		m_sweep.c.set(m_sweep.c0);
		
		m_sweep.a0 = m_sweep.a = angle;

		BroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;
		for (Fixture f = m_fixtureList; f != null; f = f.m_next)
		{
			f.synchronize(broadPhase, m_xf, m_xf);
		}

		m_world.m_contactManager.findNewContacts();
	}

	/**
	 * Get the body transform for the body's origin.
	 * @return the world transform of the body's origin.
	 */
	public final Transform getTransform(){
		return m_xf;
	}

	/**
	 * Get the world body origin position.
	 * @return the world position of the body's origin.
	 */
	public final Vec2 getPosition(){
		return m_xf.position;
	}

	/**
	 * Get the angle in radians.
	 * @return the current world rotation angle in radians.
	 */
	public final float getAngle(){
		return m_sweep.a;
	}

	/**
	 * Get the world position of the center of mass.
	 */
	public final Vec2 getWorldCenter(){
		return m_sweep.c;
	}

	/**
	 * Get the local position of the center of mass.
	 */
	public final Vec2 getLocalCenter(){
		return m_sweep.localCenter;
	}

	/**
	 * Set the linear velocity of the center of mass.
	 * @param v the new linear velocity of the center of mass.
	 */
	public final void setLinearVelocity( Vec2 v){
		m_linearVelocity.set(v);
	}

	/**
	 * Get the linear velocity of the center of mass.
	 * @return the linear velocity of the center of mass.
	 */
	public final Vec2 getLinearVelocity(){
		return m_linearVelocity;
	}

	/**
	 * Set the angular velocity.
	 * @param omega the new angular velocity in radians/second.
	 */
	public final void setAngularVelocity(float omega){
		m_angularVelocity = omega;
	}

	/**
	 * Get the angular velocity.
	 * @return the angular velocity in radians/second.
	 */
	public final float getAngularVelocity(){
		return m_angularVelocity;
	}

	// djm pooling
	private static final TLVec2 tltemp = new TLVec2();
	/**
	 * Apply a force at a world point. If the force is not
	 * applied at the center of mass, it will generate a torque and
	 * affect the angular velocity. This wakes up the body.
	 * @param force the world force vector, usually in Newtons (N).
	 * @param point the world position of the point of application.
	 */
	public final void applyForce( Vec2 force,  Vec2 point){
		if(isSleeping()){
			wakeUp();
		}
		
		m_force.addLocal(force);
		Vec2 temp = tltemp.get();
		temp.set(point).subLocal(m_sweep.c);
		m_torque += Vec2.cross(temp, force);
	}

	/**
	 * Apply a torque. This affects the angular velocity
	 * without affecting the linear velocity of the center of mass.
	 * This wakes up the body.
	 * @param torque about the z-axis (out of the screen), usually in N-m.
	 */
	public final void applyTorque(float torque){
		if(isSleeping()){
			wakeUp();
		}
		
		m_torque += torque;
	}

	// djm pooling from above
	/**
	 * Apply an impulse at a point. This immediately modifies the velocity.
	 * It also modifies the angular velocity if the point of application
	 * Is not at the center of mass. This wakes up the body.
	 * @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
	 * @param point the world position of the point of application.
	 */
	public final void applyImpulse( Vec2 impulse,  Vec2 point){
		if(isSleeping()){
			wakeUp();
		}
		Vec2 temp = tltemp.get();
		
		temp.set(impulse).mulLocal(m_invMass);
		m_linearVelocity.addLocal(temp);
		
		temp.set(point).subLocal(m_sweep.c);
		m_angularVelocity += m_invI * Vec2.cross(temp, impulse);
	}

	/**
	 * Get the total mass of the body.
	 * @return the mass, usually in kilograms (kg).
	 */
	public final float getMass(){
		return m_mass;
	}

	/**
	 * Get the central rotational inertia of the body.
	 * @return the rotational inertia, usually in kg-m^2.
	 */
	public final float getInertia(){
		return m_I;
	}

	/**
	 * Get the mass data of the body. The rotational inertia is relative
	 * to the center of mass.
	 * @return a struct containing the mass, inertia and center of the body.
	 */
	public final void getMassData(MassData data){
		data.mass = m_mass;
		data.I = m_I;
		data.center.set(m_sweep.localCenter);
	}

	// djm pooling from below
	/**
	 * Set the mass properties to override the mass properties of the fixtures.
	 * Note that this changes the center of mass position. You can make the body
	 * static by using zero mass.
	 * Note that creating or destroying fixtures can also alter the mass.
	 * @warning The supplied rotational inertia is assumed to be relative to the center of mass.
	 * @param massData the mass properties.
	 */
	public final void setMassData( MassData massData){
		// TODO_ERIN adjust linear velocity and torque to account for movement of center.
		assert(m_world.isLocked() == false);
		if (m_world.isLocked() == true){
			return;
		}

		m_invMass = 0.0f;
		m_I = 0.0f;
		m_invI = 0.0f;

		m_mass = massData.mass;

		if (m_mass > 0.0f){
			m_invMass = 1.0f / m_mass;
		}

		if (massData.I > 0.0f && (m_flags & e_fixedRotationFlag) == 0){
			m_I = massData.I - m_mass * Vec2.dot(massData.center, massData.center);
			m_invI = 1.0f / m_I;
		}

		// Move center of mass.
		Vec2 oldCenter = m_sweep.c;
		m_sweep.localCenter = massData.center;
		//m_sweep.c0 = m_sweep.c = Mul(m_xf, m_sweep.localCenter);
		Transform.mulToOut(m_xf, m_sweep.localCenter, m_sweep.c0);
		m_sweep.c.set(m_sweep.c0);
		
		
		// Update center of mass velocity.
		//m_linearVelocity += Cross(m_angularVelocity, m_sweep.c - oldCenter);
		Vec2 temp = tltemp.get();
		temp.set(m_sweep.c).subLocal(oldCenter);
		Vec2.crossToOut(m_angularVelocity, temp, temp);
		m_linearVelocity.addLocal(temp);

		int oldType = m_type;
		if (m_invMass == 0.0f && m_invI == 0.0f){
			m_type = e_staticType;
			m_angularVelocity = 0.0f;
			m_linearVelocity.setZero();
		}
		else{
			m_type = e_dynamicType;
		}

		// If the body type changed, we need to flag contacts for filtering.
		if (oldType != m_type){
			for (ContactEdge ce = m_contactList; ce != null; ce = ce.next){
				ce.contact.flagForFiltering();
			}
		}
	}

	// djm pooling from below;
	/**
	 * This resets the mass properties to the sum of the mass properties of the fixtures.
	 * This normally does not need to be called unless you called setMassData to override
	 * the mass and you later want to reset the mass.
	 */
	public final void resetMass(){
		// Compute mass data from shapes. Each shape has its own density.
		m_mass = 0.0f;
		m_invMass = 0.0f;
		m_I = 0.0f;
		m_invI = 0.0f;

		Vec2 center = Vec2.ZERO;
		Vec2 temp = tltemp.get();
		for (Fixture f = m_fixtureList; f != null; f = f.m_next){
			final MassData massData = f.getMassData();
			m_mass += massData.mass;
			//center += massData.mass * massData.center;
			temp.set(massData.center).mulLocal(massData.mass);
			center.addLocal(temp);
			m_I += massData.I;
		}

		// Compute center of mass.
		if (m_mass > 0.0f){
			m_invMass = 1.0f / m_mass;
			center.mulLocal(m_invMass);
		}

		if (m_I > 0.0f && (m_flags & e_fixedRotationFlag) == 0){
			// Center the inertia about the center of mass.
			m_I -= m_mass * Vec2.dot(center, center);
			assert(m_I > 0.0f);
			m_invI = 1.0f / m_I;
		}
		else{
			m_I = 0.0f;
			m_invI = 0.0f;
		}

		// Move center of mass.
		Vec2 oldCenter = m_sweep.c;
		m_sweep.localCenter = center;
		//m_sweep.c0 = m_sweep.c = Mul(m_xf, m_sweep.localCenter);
		Transform.mulToOut(m_xf, m_sweep.localCenter, m_sweep.c0);
		m_sweep.c.set(m_sweep.c0);
		
		
		// Update center of mass velocity.
		//m_linearVelocity += Cross(m_angularVelocity, m_sweep.c - oldCenter);
		temp.set(m_sweep.c).subLocal(oldCenter);
		Vec2.crossToOut(m_angularVelocity, temp, temp);
		m_linearVelocity.addLocal(temp);
		
		// Determine the new body type.
		int oldType = m_type;
		if (m_invMass == 0.0f && m_invI == 0.0f){
			m_type = e_staticType;
		}
		else{
			m_type = e_dynamicType;
		}

		// If the body type changed, we need to flag contacts for filtering.
		if (oldType != m_type){
			for (ContactEdge ce = m_contactList; ce != null; ce = ce.next){
				ce.contact.flagForFiltering();
			}
		}
	}

	/**
	 * Get the world coordinates of a point given the local coordinates.
	 * @param localPoint a point on the body measured relative the the body's origin.
	 * @return the same point expressed in world coordinates.
	 */
	public final Vec2 getWorldPoint( Vec2 localPoint){
		Vec2 v = new Vec2();
		getWorldPointToOut(localPoint, v);
		return v;
	}
	
	public final void getWorldPointToOut(Vec2 localPoint, Vec2 out){
		Transform.mulToOut(m_xf, localPoint, out);
	}

	/**
	 * Get the world coordinates of a vector given the local coordinates.
	 * @param localVector a vector fixed in the body.
	 * @return the same vector expressed in world coordinates.
	 */
	public final Vec2 getWorldVector( Vec2 localVector){
		Vec2 out = new Vec2();
		getWorldVectorToOut(localVector, out);
		return out;
	}
	
	public final void getWorldVectorToOut( Vec2 localVector, Vec2 out){
		Mat22.mulToOut(m_xf.R, localVector, out);
	}

	/**
	 * Gets a local point relative to the body's origin given a world point.
	 * @param a point in world coordinates.
	 * @return the corresponding local point relative to the body's origin.
	 */
	public final Vec2 getLocalPoint( Vec2 worldPoint){
		Vec2 out = new Vec2();
		getLocalPointToOut(worldPoint, out);
		return out;
	}
	
	public final void getLocalPointToOut( Vec2 worldPoint, Vec2 out){
		Transform.mulTransToOut(m_xf, worldPoint, out);
	}

	/**
	 * Gets a local vector given a world vector.
	 * @param a vector in world coordinates.
	 * @return the corresponding local vector.
	 */
	public final Vec2 getLocalVector( Vec2 worldVector){
		Vec2 out = new Vec2();
		getLocalVectorToOut(worldVector, out);
		return out;
	}
	
	public final void getLocalVectorToOut( Vec2 worldVector, Vec2 out){
		Mat22.mulTransToOut(m_xf.R, worldVector, out);
	}

	/**
	 * Get the world linear velocity of a world point attached to this body.
	 * @param a point in world coordinates.
	 * @return the world velocity of a point.
	 */
	public final Vec2 getLinearVelocityFromWorldPoint( Vec2 worldPoint){
		Vec2 out = new Vec2();
		getLinearVelocityFromWorldPointToOut(worldPoint, out);
		return out;
	}
	
	public final void getLinearVelocityFromWorldPointToOut( Vec2 worldPoint, Vec2 out){
		out.set(worldPoint).subLocal(m_sweep.c);
		Vec2.crossToOut(m_angularVelocity, out, out);
		out.addLocal(m_linearVelocity);
	}

	/**
	 * Get the world velocity of a local point.
	 * @param a point in local coordinates.
	 * @return the world velocity of a point.
	 */
	public final Vec2 getLinearVelocityFromLocalPoint( Vec2 localPoint){
		Vec2 out = new Vec2();
		getLinearVelocityFromLocalPointToOut(localPoint, out);
		return out;
	}
	
	public final void getLinearVelocityFromLocalPointToOut( Vec2 localPoint, Vec2 out){
		getWorldPointToOut(localPoint,out);
		getLinearVelocityFromWorldPointToOut(localPoint, out);
	}

	/** Get the linear damping of the body. */
	public final float getLinearDamping(){
		return m_linearDamping;
	}

	/** Set the linear damping of the body. */
	public final void setLinearDamping(float linearDamping){
		m_linearDamping = linearDamping;
	}

	/** Get the angular damping of the body. */
	public final float getAngularDamping(){
		return m_angularDamping;
	}

	/** Set the angular damping of the body. */
	public final void setAngularDamping(float angularDamping){
		m_angularDamping = angularDamping;
	}

	/** Is this body treated like a bullet for continuous collision detection?*/
	public final boolean isBullet(){
		return (m_flags & e_bulletFlag) == e_bulletFlag;
	}

	/** Should this body be treated like a bullet for continuous collision detection?*/
	public final void setBullet(boolean flag){
		if (flag){
			m_flags |= e_bulletFlag;
		}
		else{
			m_flags &= ~e_bulletFlag;
		}
	}

	/**
	 * Is this body static (immovable)?*/
	public final boolean isStatic(){
		return m_type == e_staticType;
	}

	/** Is this body dynamic (movable)?*/
	public final boolean isDynamic(){
		return m_type == e_dynamicType;
	}

	/** Is this body sleeping (not simulating).*/
	public final boolean isSleeping(){
		return (m_flags & e_sleepFlag) == e_sleepFlag;
	}

	/** Is this body allowed to sleep*/
	public final boolean isAllowSleeping(){
		return (m_flags & e_allowSleepFlag) == e_allowSleepFlag;
	}

	/** You can disable sleeping on this body.*/
	public final void allowSleeping(boolean flag){
		if (flag){
			m_flags |= e_allowSleepFlag;
		}
		else{
			m_flags &= ~e_allowSleepFlag;
			wakeUp();
		}
	}

	/** Wake up this body so it will begin simulating.*/
	public final void wakeUp(){
		m_flags &= ~e_sleepFlag;
		m_sleeptime = 0.0f;
	}

	/**
	 * Put this body to sleep so it will stop simulating.
	 * This also sets the velocity to zero.
	 */
	public final void putToSleep(){
		m_flags |= e_sleepFlag;
		m_sleeptime = 0.0f;
		m_linearVelocity.setZero();
		m_angularVelocity = 0.0f;
		m_force.setZero();
		m_torque = 0.0f;
	}

	/** Get the list of all fixtures attached to this body.*/
	public final Fixture getFixtureList(){
		return m_fixtureList;
	}

	/** Get the list of all joints attached to this body.*/
	public final JointEdge getJointList(){
		return m_jointList;
	}

	/**
	 * Get the list of all contacts attached to this body.
	 * @warning this list changes during the time step and you may
	 * miss some collisions if you don't use ContactListener.
	 */
	public final ContactEdge getContactList(){
		return m_contactList;
	}

	/** Get the next body in the world's body list.*/
	public final Body getNext(){
		return m_next;
	}

	/** Get the user data pointer that was provided in the body definition.*/
	public final Object getUserData(){
		return m_userData;
	}

	/**
	 * Set the user data. Use this to store your application specific data.*/
	public final void setUserData(Object data){
		m_userData = data;
	}

	/**
	 * Get the parent world of this body.*/
	public final World getWorld(){
		return m_world;
	}
	
	// djm pooling
	private static final TLTransform tlxf1 = new TLTransform();
	
	protected final void synchronizeFixtures(){
		Transform xf1 = tlxf1.get();
		xf1.R.set(m_sweep.a0);
		//xf1.position = m_sweep.c0 - Mul(xf1.R, m_sweep.localCenter);
		Mat22.mulToOut(xf1.R, m_sweep.localCenter, xf1.position);
		xf1.position.mulLocal(-1).addLocal(m_sweep.c0);
		
		BroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;
		for (Fixture f = m_fixtureList; f != null; f = f.m_next){
			f.synchronize(broadPhase, xf1, m_xf);
		}
	}
	
	public final void synchronizeTransform(){
		m_xf.R.set(m_sweep.a);
		
		//m_xf.position = m_sweep.c - Mul(m_xf.R, m_sweep.localCenter);

		Mat22.mulToOut(m_xf.R, m_sweep.localCenter, m_xf.position);
		m_xf.position.mul(-1).addLocal(m_sweep.c);
	}

	/**
	 * This is used to prevent connected bodies from colliding.
	 * It may lie, depending on the collideConnected flag in the
	 * connecting joint.
	 * @return
	 */
	protected final boolean isConnected(Body other){
		for (JointEdge jn = m_jointList; jn != null; jn = jn.next){
			if (jn.other == other){
				return jn.joint.m_collideConnected == false;
			}
		}

		return false;
	}

	protected final void advance(float t){
		m_sweep.advance(t);
		m_sweep.c.set(m_sweep.c0);
		m_sweep.a = m_sweep.a0;
		synchronizeTransform();
	}
}
