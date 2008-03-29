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

package org.jbox2d.dynamics;

import org.jbox2d.collision.MassData;
import org.jbox2d.collision.Shape;
import org.jbox2d.collision.ShapeDef;
import org.jbox2d.common.*;
import org.jbox2d.dynamics.contacts.ContactEdge;
import org.jbox2d.dynamics.joints.JointEdge;

// Updated to rev. 54->118 of b2Body.cpp/.h
// Rewritten completely for rev. 118 (too many changes, needed reorganization for maintainability)

public class Body {
	
	//m_flags
	public static final int e_frozenFlag = 0x0002;
	public static final int e_islandFlag = 0x0004;
	public static final int e_sleepFlag = 0x0008;
	public static final int e_allowSleepFlag = 0x0010;
	public static final int e_bulletFlag = 0x0020;
	public static final int e_fixedRotationFlag = 0x0040;
	public int m_flags;

	//m_type
	public static final int e_staticType = 0;
	public static final int e_dynamicType = 1;
	public static final int e_maxTypes = 2;
	public int m_type;
	
	/** The body origin transform */
	public XForm m_xf; 
	
	/** The swept motion for CCD */
	public Sweep m_sweep;  
	
	public Vec2 m_linearVelocity;
	public float m_angularVelocity;
	
	public Vec2 m_force;
	public float m_torque;

	public World m_world;
	public Body m_prev;
	public Body m_next;

	public Shape m_shapeList;
	public int m_shapeCount;

	public JointEdge m_jointList;
	public ContactEdge m_contactList;

	public float m_mass, m_invMass;
	public float m_I, m_invI;

	public float m_linearDamping;
	public float m_angularDamping;

	public float m_sleepTime;

	/** 
	 * A holder to attach external data to a body.
	 * Useful to keep track of what game entity
	 * each body represents. This is copied from
	 * the BodyDef used to create the body, so may
	 * be set there instead. 
	 */
	public Object m_userData;

	/**
	 * Should not be called by user, as it will not
	 * be properly added to the world.  Instead,
	 * create a BodyDef object and pass it
	 * to World.createDynamicBody or World.createStaticBody.
	 * 
	 * @param bd Body definition
	 * @param type Body.e_dynamicType or Body.e_staticType
	 * @param world World to create body in
	 */
	public Body(BodyDef bd, int type, World world) {
		assert(world.m_lock == false);
		assert(type < e_maxTypes);
		
		m_flags = 0;
		
		if (bd.isBullet) m_flags |= e_bulletFlag;
		if (bd.fixedRotation) m_flags |= e_fixedRotationFlag;
		if (bd.allowSleep) m_flags |= e_allowSleepFlag;
		if (bd.isSleeping) m_flags |= e_sleepFlag;
		
		m_type = type;
		
		m_world = world;
		
		m_xf = new XForm();
		
		m_xf.position.set(bd.position);
		m_xf.R.set(bd.angle);
		
		m_sweep = new Sweep();
		m_sweep.localCenter.set(bd.massData.center);
		m_sweep.t0 = 1.0f;
		m_sweep.a0 = m_sweep.a = bd.angle;
		m_sweep.c.set(XForm.mul(m_xf, m_sweep.localCenter));
		m_sweep.c0.set(m_sweep.c);
		
		m_jointList = null;
		m_contactList = null;
		m_prev = null;
		m_next = null;
		
		m_linearDamping = bd.linearDamping;
		m_angularDamping = bd.angularDamping;
		
		m_force = new Vec2(0.0f, 0.0f);
		m_torque = 0.0f;
		
		m_linearVelocity = new Vec2(0.0f, 0.0f);
		m_angularVelocity = 0.0f;
		
		m_sleepTime = 0.0f;
		
		m_mass = 0.0f;
		m_invMass = 0.0f;
		m_I = 0.0f;
		m_invI = 0.0f;
		
		if (m_type == e_dynamicType) {
			m_mass = bd.massData.mass;
		}
		
		if (m_mass > 0.0f) {
			m_invMass = 1.0f / m_mass;
		}
		
		if ((m_flags & Body.e_fixedRotationFlag) == 0 &&
				m_type == e_dynamicType) {
			m_I = bd.massData.I;
		}
		
		if (m_I > 0.0f) {
			m_invI = 1.0f / m_I;
		}
		
		m_userData = bd.userData;
		
		m_shapeList = null;
		m_shapeCount = 0;
		
	}

	/**
	 *  Creates a shape and attach it to this body.
	 * <BR><em>Warning</em>: This function is locked during callbacks.
	 * @param def the shape definition.
	 */
	public Shape createShape(ShapeDef def){
		assert(m_world.m_lock == false);

		if (m_world.m_lock == true){
			return null;
		}

		Shape s = Shape.create(def);

		s.m_next = m_shapeList;
		m_shapeList = s;
		++m_shapeCount;

		s.m_body = this;

		// Add the shape to the world's broad-phase.
		s.createProxy(m_world.m_broadPhase, m_xf);

		// Compute the sweep radius for CCD.
		s.updateSweepRadius(m_sweep.localCenter);

		return s;
	}
	
	/** 
	 * Destroy a shape. This removes the shape from the broad-phase and
	 * therefore destroys any contacts associated with this shape. All shapes
	 * attached to a body are implicitly destroyed when the body is destroyed.
	 * <BR><em>Warning</em>: This function is locked during callbacks.
	 * @param s the shape to be removed.
	 */
	public void destroyShape(Shape s){
		assert(m_world.m_lock == false);
		if (m_world.m_lock == true) {
			return;
		}

		assert(s.m_body == this);
		s.destroyProxy(m_world.m_broadPhase);

		assert(m_shapeCount > 0);
		
		// Remove s from linked list, fix up connections
		// TODO: verify that this works right
		Shape node = m_shapeList;
		Shape prevNode = null;
		boolean found = false;
		while (node != null) {
			if (node == s) {
				if (prevNode == null) {
					m_shapeList = s.m_next;
					found = true;
					break;
				} else {
					prevNode.m_next = s.m_next;
					found = true;
					break;
				}
			}
			prevNode = node;
			node = node.m_next;
		}
		/*
		Shape** node = &m_shapeList;
		boolean found = false;
		while (*node != NULL)
		{
			if (*node == s)
			{
				*node = s->m_next;
				found = true;
				break;
			}

			node = &(*node)->m_next;
		}
		*/

		// You tried to remove a shape that is not attached to this body.
		assert(found);

		s.m_body = null;
		s.m_next = null;

		--m_shapeCount;
		Shape.destroy(s);
	}
	
	/**
	 * Set the mass properties. Note that this changes the center of mass position.
	 * If you are not sure how to compute mass properties, use setMassFromShapes().
	 * The inertia tensor is assumed to be relative to the center of mass.
	 * @param massData the mass properties.
	 */
	public void setMass(MassData massData){
		assert(m_world.m_lock == false);
		if (m_world.m_lock == true) return;
		
		if (m_type == e_staticType) return;
		
		m_mass = 0.0f;
		m_invMass = 0.0f;
		m_I = 0.0f;
		m_invI = 0.0f;
		
		m_mass = massData.mass;
		
		if (m_mass > 0.0f) {
			m_invMass = 1.0f / m_mass;
		}
		
		if ((m_flags & Body.e_fixedRotationFlag) == 0) {
			m_I = massData.I;
		}
		
		if (m_I > 0.0f) {
			m_invI = 1.0f / m_I;
		}
		
		// Move center of mass.
		m_sweep.localCenter.set(massData.center);
		m_sweep.c.set(XForm.mul(m_xf, m_sweep.localCenter));
		m_sweep.c0.set(m_sweep.c);
		
		// Update the sweep radii of all child shapes
		for (Shape s = m_shapeList; s != null; s = s.m_next) {
			s.updateSweepRadius(m_sweep.localCenter);
		}
	}

	/** 
	 * Compute the mass properties from the attached shapes. You typically call this
	 * after adding all the shapes. If you add or remove shapes later, you may want
	 * to call this again. Note that this changes the center of mass position.
	 */
	public void setMassFromShapes(){
		assert(m_world.m_lock == false);
		if (m_world.m_lock == true) return;
		if (m_type == e_staticType) return;
		
		// Compute mass data from shapes.  Each shape has its own density.
		m_mass = 0.0f;
		m_invMass = 0.0f;
		m_I = 0.0f;
		m_invI = 0.0f;
		
		Vec2 center = new Vec2(0.0f, 0.0f);
		for (Shape s = m_shapeList; s != null; s = s.m_next) {
			MassData massData = new MassData();
			s.computeMass(massData);
			m_mass += massData.mass;
			center.x += massData.mass * massData.center.x;
			center.y += massData.mass * massData.center.y;
			m_I += massData.I;
		}
		
		// Compute center of mass, and shift the origin to the COM
		if (m_mass > 0.0f) {
			m_invMass = 1.0f / m_mass;
			center.x *= m_invMass;
			center.y *= m_invMass;
		} else {
			m_invMass = 0.0f;
			m_invI = 0.0f;
		}
		
		if (m_I > 0.0f && (m_flags & e_fixedRotationFlag) == 0) {
			// Center the inertia about the center of mass
			m_I -= m_mass * Vec2.dot(center, center);
			assert(m_I > 0.0f);
			m_invI = 1.0f / m_I;
		} else {
			m_I = 0.0f;
			m_invI = 0.0f;
		}
		
		// Move center of mass
		m_sweep.localCenter.set(center);
		m_sweep.c.set(XForm.mul(m_xf, m_sweep.localCenter));
		m_sweep.c0.set(m_sweep.c);
		
		// Update the sweep radii of all child shapes
		for (Shape s = m_shapeList; s != null; s = s.m_next) {
			s.updateSweepRadius(m_sweep.localCenter);
		}
	}
	
	/**
	 * Set the position of the body's origin and rotation (radians).
	 * This breaks any contacts and wakes the other bodies.
	 * @param position the new world position of the body's origin (not necessarily
	 * the center of mass).
	 * @param angle the new world rotation angle of the body in radians.
	 * @return false if the movement put a shape outside the world. In this case the
	 * body is automatically frozen.
	 */
	public boolean setXForm(Vec2 position, float angle){
		assert(m_world.m_lock == false);
		if (m_world.m_lock == true) return true;
		if (isFrozen())	return false;

		m_xf.R.set(angle);
		m_xf.position.set(position);

		m_sweep.c.set(XForm.mul(m_xf, m_sweep.localCenter));
		m_sweep.c0.set(m_sweep.c);
		m_sweep.a0 = m_sweep.a = angle;

		boolean freeze = false;

		for (Shape s = m_shapeList; s != null; s = s.m_next) {
			boolean inRange = s.synchronize(m_world.m_broadPhase, m_xf, m_xf);

			if (inRange == false) {
				freeze = true;
				break;
			}
		}

		if (freeze == true) {
			m_flags |= e_frozenFlag;
			m_linearVelocity.setZero();
			m_angularVelocity = 0.0f;
			for (Shape s = m_shapeList; s != null; s = s.m_next) {
				s.destroyProxy(m_world.m_broadPhase);
			}

			// Failure
			return false;
		}

		// Success
		m_world.m_broadPhase.commit();

		return true;
	}
	
	/**
	 * Get a copy of the body transform for the body's origin.
	 * @return the world transform of the body's origin.
	 */
	public XForm getXForm(){
		XForm xf = new XForm();
		xf.set(m_xf);
		return xf;
	}

	/**
	 * Get a copy of the world body origin position.  This
	 * is not necessarily the same as the center of mass.
	 * @return the world position of the body's origin.
	 */
	public Vec2 getPosition(){
		return m_xf.position.clone();
	}

	/**
	 * Get the angle in radians.
	 * @return the current world rotation angle in radians.
	 */
	public float getAngle(){
		return m_sweep.a;
	}

	/** Get a copy of the world position of the center of mass. */
	public Vec2 getWorldCenter(){
		return m_sweep.c.clone();
	}

	/** Get a copy of the local position of the center of mass. */
	public Vec2 getLocalCenter(){
		return m_sweep.localCenter.clone();
	}

	/**
	 * Set the linear velocity of the center of mass.
	 * @param v the new linear velocity of the center of mass.
	 */
	public void setLinearVelocity(Vec2 v){
		m_linearVelocity.set(v);
	}

	/**
	 * Get a copy of the linear velocity of the center of mass.
	 * @return the linear velocity of the center of mass.
	 */
	public Vec2 getLinearVelocity(){
		return m_linearVelocity.clone();
	}

	/**
	 * Set the angular velocity.
	 * @param omega the new angular velocity in radians/second.
	 */
	public void setAngularVelocity(float omega){
		m_angularVelocity = omega;
	}

	/** 
	 * Get the angular velocity.
	 * @return the angular velocity in radians/second.
	 */
	public float getAngularVelocity(){
		return m_angularVelocity;
	}

	/**
	 * Apply a force at a world point. If the force is not
	 * applied at the center of mass, it will generate a torque and
	 * affect the angular velocity. This wakes up the body.
	 * @param force the world force vector, usually in Newtons (N).
	 * @param point the world position of the point of application.
	 */
	public void applyForce(Vec2 force, Vec2 point){
		if (isSleeping()) wakeUp();
		m_force.addLocal(force);
		m_torque += Vec2.cross(point.sub(m_sweep.c), force);
	}

	/**
	 * Apply a torque. This affects the angular velocity
	 * without affecting the linear velocity of the center of mass.
	 * This wakes up the body.
	 * @param torque about the z-axis (out of the screen), usually in N-m.
	 */
	public void applyTorque(float torque){
		if (isSleeping()) wakeUp();
		m_torque += torque;
	}

	/**
	 * Apply an impulse at a point. This immediately modifies the velocity.
	 * It also modifies the angular velocity if the point of application
	 * is not at the center of mass. This wakes up the body.
	 * @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
	 * @param point the world position of the point of application.
	 */
	public void applyImpulse(Vec2 impulse, Vec2 point){
		if (isSleeping()) wakeUp();
		m_linearVelocity.x += m_invMass * impulse.x;
		m_linearVelocity.y += m_invMass * impulse.y;
		m_angularVelocity += m_invI * Vec2.cross(point.sub(m_sweep.c), impulse);
	}

	/**
	 * Get the total mass of the body.
	 * @return the mass, usually in kilograms (kg).
	 */
	public float getMass(){
		return m_mass;
	}

	/**
	 * Get the central rotational inertia of the body.
	 * @return the rotational inertia, usually in kg-m^2.
	 */
	public float getInertia(){
		return m_I;
	}

	/** 
	 * Get the world coordinates of a point given the local coordinates.
	 * @param localPoint a point on the body measured relative the the body's origin.
	 * @return the same point expressed in world coordinates.
	 */
	public Vec2 getWorldPoint(Vec2 localPoint){
		return XForm.mul(m_xf, localPoint);
	}

	/**
	 * Get the world coordinates of a vector given the local coordinates.
	 * @param localVector a vector fixed in the body.
	 * @return the same vector expressed in world coordinates.
	 */
	public Vec2 getWorldVector(Vec2 localVector){
		return Mat22.mul(m_xf.R, localVector);
	}

	/**
	 * Gets a local point relative to the body's origin given a world point.
	 * @param worldPoint a point in world coordinates.
	 * @return the corresponding local point relative to the body's origin.
	 */
	public Vec2 getLocalPoint(Vec2 worldPoint){
		return XForm.mulT(m_xf, worldPoint);
	}

	/** 
	 * Gets a local vector given a world vector.
	 * @param worldVector a vector in world coordinates.
	 * @return the corresponding local vector.
	 */
	public Vec2 getLocalVector(Vec2 worldVector){
		return Mat22.mulT(m_xf.R, worldVector);
	}

	/** Is this body treated like a bullet for continuous collision detection? */
	public boolean isBullet(){
		return (m_flags & e_bulletFlag) == e_bulletFlag;
	}

	/** 
	 * Should this body be treated like a bullet for continuous collision detection? 
	 * Use sparingly, as continuous collision detection can be expensive.
	 */
	public void setBullet(boolean flag){
		if (flag) {
			m_flags |= e_bulletFlag;
		} else {
			m_flags &= ~e_bulletFlag;
		}
	}

	/** Is this body static (immovable)? */
	public boolean isStatic(){
		return m_type == e_staticType;
	}

	/** Is this body dynamic (movable)? */
	public boolean isDynamic(){
		return m_type == e_dynamicType;
	}

	/** Is this body frozen? */
	public boolean isFrozen(){
		return (m_flags & e_frozenFlag) == e_frozenFlag;
	}

	/** Is this body sleeping (not simulating). */
	public boolean isSleeping(){
		return (m_flags & e_sleepFlag) == e_sleepFlag;
	}

	/** Set to false to prevent this body from sleeping due to inactivity. */
	public void allowSleeping(boolean flag){
		if (flag) {
			m_flags |= e_allowSleepFlag;
		} else {
			m_flags &= ~e_allowSleepFlag;
			wakeUp();
		}
	}

	/** Wake up this body so it will begin simulating. */
	public void wakeUp(){
		m_flags &= ~e_sleepFlag;
		m_sleepTime = 0.0f;
	}

	/**
	 * Get the linked list of all shapes attached to this body.
	 * @return first Shape in linked list
	 */
	public Shape getShapeList(){
		return m_shapeList;
	}

	/**
	 * Get the linked list of all joints attached to this body.
	 * @return first JointEdge in linked list
	 */
	public JointEdge getJointList(){
		return m_jointList;
	}

	/**
	 * Get the linked list of all contacts attached to this body.
	 * @return first ContactEdge in linked list
	 */
	public ContactEdge getContactList(){
		return m_contactList;
	}

	/** Get the next body in the world's body list. */
	public Body getNext(){
		return m_next;
	}

	/** Get the user data Object reference that was provided in the body definition. */
	public Object getUserData(){
		return m_userData;
	}
	
	/* INTERNALS BELOW */

	/** For internal use only. */
	public void computeMass(){
		//seems to be missing from C++ version...
	}

	/** For internal use only. */
	public boolean synchronizeShapes(){
		XForm xf1 = new XForm();
		xf1.R.set(m_sweep.a0);
		xf1.position.set(m_sweep.c0.sub(Mat22.mul(xf1.R, m_sweep.localCenter)));
		
		boolean inRange = true;
		for (Shape s = m_shapeList; s != null; s = s.m_next) {
			inRange = s.synchronize(m_world.m_broadPhase, xf1, m_xf);
			if (inRange == false) break;
		}
		
		if (inRange == false) {
			m_flags |= e_frozenFlag;
			m_linearVelocity.setZero();
			m_angularVelocity = 0.0f;
			for (Shape s = m_shapeList; s != null; s = s.m_next) {
				s.destroyProxy(m_world.m_broadPhase);
			}
			
			// Failure
			return false;
		}
		
		// Success
		return true;
	}

	/** For internal use only. */
	public void synchronizeTransform(){
		m_xf.R.set(m_sweep.a);
		m_xf.position.set(m_sweep.c.sub(Mat22.mul(m_xf.R,m_sweep.localCenter)));
		//System.out.println(m_xf);
	}

	/**
	 * This is used to prevent connected bodies from colliding.
	 * It may lie, depending on the collideConnected flag, so
	 * it won't be very useful external to the engine.
	 */
	public boolean isConnected(Body other){
		for (JointEdge jn = m_jointList; jn != null; jn = jn.next) {
			if (jn.other == other) {
				//System.out.println("connected");
				return (jn.joint.m_collideConnected == false);
			}
		}
		return false;
	}

	/** For internal use only. */
	public void advance(float t){
		// Advance to the new safe time
		m_sweep.advance(t);
		m_sweep.c.set(m_sweep.c0);
		m_sweep.a = m_sweep.a0;
		synchronizeTransform();
	}
	
}

