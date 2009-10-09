package org.jbox2d.dynamics;

import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.Sweep;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;
import org.jbox2d.structs.collision.MassData;
import org.jbox2d.structs.dynamics.contacts.ContactEdge;

/**
 * A rigid body. These are created via World::createBody.
 *
 * @author daniel
 */

public class Body {
	public static final int e_islandFlag = 0x0001;
	public static final int e_sleepFlag = 0x0002;
	public static final int e_allowSleepFlag = 0x0004;
	public static final int e_bulletFlag = 0x0008;
	public static final int e_fixedRotationFlag = 0x0010;
	
	public static final int TYPE_STATIC = 0;
	public static final int TYPE_DYNAMIC = 1;
	public static final int MAX_TYPES = 2;
	
	
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
	
	public float m_angularVelocity;
	
	public final Vec2 m_force = new Vec2();
	
	public World m_world;
	
	public Body m_prev;
	public Body m_next;
	
	public Fixture m_fixtureList;
	public int m_fixtureCount;
	
	public Joint m_jointList;
	public ContactEdge m_contactList;
	
	public float m_mass, m_invMass;
	public float m_I, m_invI;
	
	public float m_linearDampening;
	public float m_anglularDampening;
	
	public float m_sleeptime;
	
	public Object m_userData;
	
	
	/**
	 * Creates a fixture and attach it to this body. Use this function if you need
	 * to set some fixture parameters, like friction. Otherwise you can create the
	 * fixture directly from a shape.
	 * This function automatically updates the mass of the body.
	 * @param def the fixture definition.
	 * @warning This function is locked during callbacks.
	 */
	public final Fixture createFixture( FixtureDef def){
		
	}

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
		
	}

	/**
	 * Destroy a fixture. This removes the fixture from the broad-phase and
	 * therefore destroys any contacts associated with this fixture. All fixtures
	 * attached to a body are implicitly destroyed when the body is destroyed.
	 * @param fixture the fixture to be removed.
	 * @warning This function is locked during callbacks.
	 */
	public final void destroyFixture(Fixture fixture);

	/**
	 * Set the position of the body's origin and rotation.
	 * This breaks any contacts and wakes the other bodies.
	 * @param position the world position of the body's local origin.
	 * @param angle the world rotation in radians.
	 */
	public final void setTransform( Vec2 position, float angle);

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

	/**
	 * Apply a force at a world point. If the force is not
	 * applied at the center of mass, it will generate a torque and
	 * affect the angular velocity. This wakes up the body.
	 * @param force the world force vector, usually in Newtons (N).
	 * @param point the world position of the point of application.
	 */
	public final void applyForce( Vec2 force,  Vec2 point);

	/**
	 * Apply a torque. This affects the angular velocity
	 * without affecting the linear velocity of the center of mass.
	 * This wakes up the body.
	 * @param torque about the z-axis (out of the screen), usually in N-m.
	 */
	public final void applyTorque(float torque);

	/**
	 * Apply an impulse at a point. This immediately modifies the velocity.
	 * It also modifies the angular velocity if the point of application
	 * Is not at the center of mass. This wakes up the body.
	 * @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
	 * @param point the world position of the point of application.
	 */
	public final void applyImpulse( Vec2 impulse,  Vec2 point);

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

	/**
	 * Set the mass properties to override the mass properties of the fixtures.
	 * Note that this changes the center of mass position. You can make the body
	 * static by using zero mass.
	 * Note that creating or destroying fixtures can also alter the mass.
	 * @warning The supplied rotational inertia is assumed to be relative to the center of mass.
	 * @param massData the mass properties.
	 */
	public final void setMassData( MassData data);

	/**
	 * This resets the mass properties to the sum of the mass properties of the fixtures.
	 * This normally does not need to be called unless you called setMassData to override
	 * the mass and you later want to reset the mass.
	 */
	public final void resetMass();

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
	public final Vec2 getLocalVector( Vec2 worldVector) ;

	/**
	 * Get the world linear velocity of a world point attached to this body.
	 * @param a point in world coordinates.
	 * @return the world velocity of a point.
	 */
	public final Vec2 getLinearVelocityFromWorldPoint( Vec2 worldPoint) ;

	/**
	 * Get the world velocity of a local point.
	 * @param a point in local coordinates.
	 * @return the world velocity of a point.
	 */
	public final Vec2 getLinearVelocityFromLocalPoint( Vec2 localPoint) ;

	/** Get the linear damping of the body. */
	public final float getLinearDamping() ;

	/**
	 * Set the linear damping of the body. */
	public final void setLinearDamping(float linearDamping);

	/**
	 * Get the angular damping of the body. */
	public final float getAngularDamping() ;

	/**
	 * Set the angular damping of the body. */
	public final void setAngularDamping(float angularDamping);

	/**
	 * Is this body treated like a bullet for continuous collision detection?*/
	public final boolean isBullet() ;

	/**
	 * Should this body be treated like a bullet for continuous collision detection?*/
	public final void setBullet(boolean flag);

	/**
	 * Is this body static (immovable)?*/
	public final boolean isStatic() ;

	/**
	 * Is this body dynamic (movable)?*/
	public final boolean isDynamic() ;

	/**
	 * Is this body sleeping (not simulating).*/
	public final boolean isSleeping() ;

	/**
	 * Is this body allowed to sleep*/
	public final boolean isAllowSleeping() ;

	/**
	 * You can disable sleeping on this body.*/
	public final void AllowSleeping(boolean flag);

	/**
	 * Wake up this body so it will begin simulating.*/
	public final void WakeUp();

	/**
	 * Put this body to sleep so it will stop simulating.
	 * This also sets the velocity to zero.
	 */
	public final void PutToSleep();

	/**
	 * Get the list of all fixtures attached to this body.*/
	public final Fixture getFixtureList();

	/**
	 * Get the list of all fixtures attached to this body.*/
	public final Fixture getFixtureList() ;

	/**
	 * Get the list of all joints attached to this body.*/
	public final JointEdge getJointList();

	/**
	 * Get the list of all contacts attached to this body.
	 * @warning this list changes during the time step and you may
	 * miss some collisions if you don't use ContactListener.
	 */
	public final ContactEdge getContactList();

	/**
	 * Get the next body in the world's body list.*/
	public final Body getNext();

	/**
	 * Get the next body in the world's body list.*/
	public final Body getNext() ;

	/**
	 * Get the user data pointer that was provided in the body definition.*/
	public final void getUserData() ;

	/**
	 * Set the user data. Use this to store your application specific data.*/
	public final void setUserData(void data);

	/**
	 * Get the parent world of this body.*/
	public final World getWorld()
	
	
	protected final void synchronizeFixtures(){
		
	}
	
	protected final void synchronizeTransform(){
		
	}

	/**
	 * This is used to prevent connected bodies from colliding.
	// It may lie, depending on the collideConnected flag.
	 * @return
	 */
	protected final boolean isConnected(Body other);

	protected final void advance(float t);
}
