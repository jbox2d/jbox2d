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

import java.util.HashSet;
import java.util.Set;

import org.jbox2d.collision.MassData;
import org.jbox2d.collision.shapes.EdgeChainDef;
import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.collision.shapes.ShapeDef;
import org.jbox2d.collision.shapes.ShapeType;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Sweep;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;
import org.jbox2d.dynamics.contacts.ContactEdge;
import org.jbox2d.dynamics.controllers.ControllerEdge;
import org.jbox2d.dynamics.joints.JointEdge;
import org.jbox2d.pooling.TLVec2;
import org.jbox2d.pooling.TLXForm;

// Updated to rev. 54->118->142 of b2Body.cpp/.h
// Rewritten completely for rev. 118 (too many changes, needed reorganization for maintainability)

/**
 * A 2-dimensional rigid body.  Do not create Body objects directly;
 * instead, pass a BodyDef to either World::createStaticBody or
 * World::createDynamicBody and then call Body::createShape(ShapeDef)
 * to add geometry.  For a dynamic body, don't forget to call
 * Body::setMassFromShapes or (for experts) Body::setMass(MassData) -
 * if you forget to set the mass, the simulation will have problems.
 * <BR><BR>
 * When possible, quantities of interest should be accessed via
 * getters/setters rather than through the m_* variables.  These are
 * internal variables, and their use is generally unsupported.
 */
public class Body {
	private static volatile int nextID = 0;
	private static Object idLock = new Object();
	private int m_uniqueID;

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
	
	public ControllerEdge m_controllerList;

	/** The body origin transform */
	public final XForm m_xf;

	/** The swept motion for CCD */
	public Sweep m_sweep;

	public final Vec2 m_linearVelocity;
	public float m_angularVelocity;

	public final Vec2 m_force;
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
	 * Empty body, with no world
	 */
	public Body() {
		this( new BodyDef(), null);
	}

	/**
	 * Should not be called by user, as it will not
	 * be properly added to the world.  Instead,
	 * create a BodyDef object and pass it
	 * to World.createDynamicBody or World.createStaticBody.
	 * 
	 * @param bd Body definition
	 * @param world World to create body in
	 */
	public Body(final BodyDef bd, final World world) {
		assert(world.m_lock == false);
		
		synchronized(idLock) {
			m_uniqueID = nextID++;
		}

		m_flags = 0;

		if (bd.isBullet) {
			m_flags |= e_bulletFlag;
		}
		if (bd.fixedRotation) {
			m_flags |= e_fixedRotationFlag;
		}
		if (bd.allowSleep) {
			m_flags |= e_allowSleepFlag;
		}
		if (bd.isSleeping) {
			m_flags |= e_sleepFlag;
		}

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

		m_invMass = 0.0f;
		m_I = 0.0f;
		m_invI = 0.0f;

		m_mass = bd.massData.mass;

		if (m_mass > 0.0f) {
			m_invMass = 1.0f / m_mass;
		}

		if ((m_flags & Body.e_fixedRotationFlag) == 0) {
			m_I = bd.massData.I;
		}

		if (m_I > 0.0f) {
			m_invI = 1.0f / m_I;
		}

		if (m_invMass == 0.0f && m_invI == 0.0f) {
			m_type = e_staticType;
		} else {
			m_type = e_dynamicType;
		}

		m_userData = bd.userData;

		m_shapeList = null;
		m_shapeCount = 0;

//		System.out.println("Body hash code: " + this.hashCode());
	}

	// djm this isn't a hot method, allocation is just fine
	private float connectEdges(final EdgeShape s1, final EdgeShape s2, final float angle1) {
		final float angle2 = (float)Math.atan2(s2.getDirectionVector().y, s2.getDirectionVector().x);

		final Vec2 core = s2.getDirectionVector().mul( (float)Math.tan((angle2 - angle1) * 0.5f)) ;
		(core.subLocal(s2.getNormalVector())).mulLocal(Settings.toiSlop).addLocal(s2.getVertex1());

		final Vec2 cornerDir = s1.getDirectionVector().add(s2.getDirectionVector());
		cornerDir.normalize();

		final boolean convex = Vec2.dot(s1.getDirectionVector(), s2.getNormalVector()) > 0.0f;
		s1.setNextEdge(s2, core, cornerDir, convex);
		s2.setPrevEdge(s1, core, cornerDir, convex);
		return angle2;
	}

	/**
	 *  Creates a shape and attach it to this body.
	 * <BR><em>Warning</em>: This function is locked during callbacks.
	 * @param def the shape definition.
	 */
	// djm not a hot method, allocations are fine
	public Shape createShape(final ShapeDef def){
		assert(m_world.m_lock == false);

		if (m_world.m_lock == true){
			return null;
		}

		// TODO: Decide on a better place to initialize edgeShapes. (b2Shape::Create() can't
		//       return more than one shape to add to parent body... maybe it should add
		//       shapes directly to the body instead of returning them?)
		if (def.type == ShapeType.EDGE_SHAPE) {
			final EdgeChainDef edgeDef = (EdgeChainDef)def;
			Vec2 v1;// = new Vec2();
			Vec2 v2;// = new Vec2();
			int i = 0;

			if (edgeDef.isLoop()) {
				v1 = edgeDef.getVertices().get(edgeDef.getVertexCount()-1);
				i = 0;
			} else {
				v1 = edgeDef.getVertices().get(0);
				i = 1;
			}

			EdgeShape s0 = null;
			EdgeShape s1 = null;
			EdgeShape s2 = null;
			float angle = 0.0f;
			for (; i < edgeDef.getVertexCount(); i++) {
				v2 = edgeDef.getVertices().get(i);

				s2 = new EdgeShape(v1, v2, def);
				s2.m_next = m_shapeList;
				m_shapeList = s2;
				++m_shapeCount;
				s2.m_body = this;
				s2.createProxy(m_world.m_broadPhase, m_xf);
				s2.updateSweepRadius(m_sweep.localCenter);

				if (s1 == null) {
					s0 = s2;
					angle = (float)Math.atan2(s2.getDirectionVector().y, s2.getDirectionVector().x);
				} else {
					angle = connectEdges(s1, s2, angle);
				}
				s1 = s2;
				v1 = v2;
			}
			if (edgeDef.isLoop()) {
				connectEdges(s1, s0, angle);
			}
			return s0;
		}


		final Shape s = Shape.create(def);

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
	public void destroyShape(final Shape s){
		assert(m_world.m_lock == false);
		if (m_world.m_lock == true) {
			return;
		}

		assert(s.getBody() == this);
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
	public void setMass(final MassData massData){
		assert(m_world.m_lock == false);
		if (m_world.m_lock == true) {
			return;
		}

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
		XForm.mulToOut(m_xf, m_sweep.localCenter,m_sweep.c);
		m_sweep.c0.set(m_sweep.c);

		// Update the sweep radii of all child shapes
		for (Shape s = m_shapeList; s != null; s = s.m_next) {
			s.updateSweepRadius(m_sweep.localCenter);
		}

		final int oldType = m_type;
		if (m_invMass == 0.0f && m_invI == 0.0f) {
			m_type = e_staticType;
		} else {
			m_type = e_dynamicType;
		}

		// If the body type changed, we need to refilter the broad-phase proxies.
		if (oldType != m_type) {
			for (Shape s = m_shapeList; s != null; s = s.m_next)
			{
				s.refilterProxy(m_world.m_broadPhase, m_xf);
			}
		}
	}

	private static final TLVec2 tlCenter = new TLVec2();
	/**
	 * Compute the mass properties from the attached shapes. You typically call this
	 * after adding all the shapes. If you add or remove shapes later, you may want
	 * to call this again. Note that this changes the center of mass position.
	 */
	public void setMassFromShapes(){
		assert(m_world.m_lock == false);
		if (m_world.m_lock == true) {
			return;
		}

		// Compute mass data from shapes.  Each shape has its own density.
		m_mass = 0.0f;
		m_invMass = 0.0f;
		m_I = 0.0f;
		m_invI = 0.0f;

		// djm might as well allocate, not really a hot path
		final Vec2 center = tlCenter.get();
		center.setZero();
		for (Shape s = m_shapeList; s != null; s = s.m_next) {
			final MassData massData = new MassData();
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
		XForm.mulToOut(m_xf, m_sweep.localCenter, m_sweep.c);
		m_sweep.c0.set(m_sweep.c);

		// Update the sweep radii of all child shapes
		for (Shape s = m_shapeList; s != null; s = s.m_next) {
			s.updateSweepRadius(m_sweep.localCenter);
		}

		final int oldType = m_type;
		if (m_invMass == 0.0f && m_invI == 0.0f) {
			m_type = e_staticType;
		} else {
			m_type = e_dynamicType;
		}

		// If the body type changed, we need to refilter the broad-phase proxies.
		if (oldType != m_type) {
			for (Shape s = m_shapeList; s != null; s = s.m_next) {
				s.refilterProxy(m_world.m_broadPhase, m_xf);
			}
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
	public boolean setXForm(final Vec2 position, final float angle){
		assert(m_world.m_lock == false);
		if (m_world.m_lock == true) {
			return true;
		}
		if (isFrozen()) {
			return false;
		}

		m_xf.R.set(angle);
		m_xf.position.set(position);

		XForm.mulToOut(m_xf, m_sweep.localCenter, m_sweep.c);
		m_sweep.c0.set(m_sweep.c);
		m_sweep.a0 = m_sweep.a = angle;

		boolean freeze = false;

		for (Shape s = m_shapeList; s != null; s = s.m_next) {
			final boolean inRange = s.synchronize(m_world.m_broadPhase, m_xf, m_xf);

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
		final XForm xf = new XForm();
		xf.set(m_xf);
		return xf;
	}

	/**
	 * More for internal use.  It isn't copied,
	 * so don't modify it.  instead try to use {@link #setXForm(Vec2, float)}.
	 * Otherwise, this also gives you direct access to the body's XForm, if you
	 * really need to change something (careful!).
	 * @see #getXForm()
	 * @see #setXForm(Vec2, float)
	 * @return an uncopied version of this body's XForm
	 */
	public XForm getMemberXForm(){
		return m_xf;
	}

	/**
	 * You probably don't want to use this
	 * function.  What you really want is getWorldCenter(),
	 * which returns the center of mass (which actually has
	 * some physical significance).
	 * <p>
	 * Just in case you do want to use this,
	 * Get a copy of the world body origin position.  This
	 * is not necessarily the same as the center of mass.
	 * In fact, it's not anything in particular.  Just a
	 * point.
	 * <p>
	 * @return a copy of the world position of the body's origin.
	 */
	public Vec2 getPosition(){
		return m_xf.position.clone();
	}

	/**
	 * This is more for internal use.  It isn't copied, so don't
	 * modify it.  This is the position of the body's XForm
	 * ({@link #getXForm()}), and if you want to change that I would
	 * suggest using {@link #setXForm(Vec2, float)}.  Modifying this
	 * will not do what you want.
	 * @see #getPosition()
	 * @return the body's world position of the body's origin.
	 */
	public Vec2 getMemberPosition(){
		return m_xf.position;
	}

	/**
	 * Get the angle in radians.
	 * @return the current world rotation angle in radians.
	 */
	public float getAngle(){
		return m_sweep.a;
	}

	/**
	 * Get a copy of the world position of the center of mass.
	 * @return a copy of the world position
	 */
	public Vec2 getWorldCenter(){
		return m_sweep.c.clone();
	}

	/**
	 * More for internal use. It isn't copied, so don't
	 * modify it.  Modifying this will not do what you want,
	 * instead use {@link #setXForm(Vec2, float)}
	 * @see #getWorldCenter()
	 * @return the world position
	 */
	public Vec2 getMemberWorldCenter(){
		return m_sweep.c;
	}

	/**
	 * Get local position of the center of mass.
	 * @return a copy of the local position of the center of mass
	 */
	public Vec2 getLocalCenter(){
		return m_sweep.localCenter.clone();
	}

	/**
	 * More for internal use. It isn't a copy, so don't
	 * modify it.
	 * @return the local position of the center of mass
	 */
	public Vec2 getMemberLocalCenter(){
		return m_sweep.localCenter;
	}

	/**
	 * Set the linear velocity of the center of mass.
	 * @param v the new linear velocity of the center of mass.
	 */
	public void setLinearVelocity(final Vec2 v){
		m_linearVelocity.set(v);
	}

	/**
	 * Get the linear velocity of the center of mass. This isn't a copy,
	 * so modifying this will change the linear velocity.
	 * @return a the linear velocity of the center of mass.
	 */
	public Vec2 getLinearVelocity(){
		return m_linearVelocity;
	}

	/**
	 * Set the angular velocity.
	 * @param omega the new angular velocity in radians/second.
	 */
	public void setAngularVelocity(final float omega){
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
	// djm only one instantiated object, so we inline
	public void applyForce(final Vec2 force, final Vec2 point){
		if (isSleeping()) {
			wakeUp();
		}
		m_force.addLocal(force);
		//m_torque += Vec2.cross(point.sub(m_sweep.c), force);
		m_torque += (point.x - m_sweep.c.x) * force.y - (point.y - m_sweep.c.y) * force.x;
	}

	/**
	 * Apply a torque. This affects the angular velocity
	 * without affecting the linear velocity of the center of mass.
	 * This wakes up the body.
	 * @param torque about the z-axis (out of the screen), usually in N-m.
	 */
	public void applyTorque(final float torque){
		if (isSleeping()) {
			wakeUp();
		}
		m_torque += torque;
	}

	/**
	 * Apply an impulse at a point. This immediately modifies the velocity.
	 * It also modifies the angular velocity if the point of application
	 * is not at the center of mass. This wakes up the body.
	 * @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
	 * @param point the world position of the point of application.
	 */
	// djm only one allocation, so we inline
	public void applyImpulse(final Vec2 impulse, final Vec2 point){
		if (isSleeping()) {
			wakeUp();
		}
		m_linearVelocity.x += m_invMass * impulse.x;
		m_linearVelocity.y += m_invMass * impulse.y;
		//m_angularVelocity += m_invI * Vec2.cross(point.sub(m_sweep.c), impulse);
		m_angularVelocity += m_invI * ((point.x - m_sweep.c.x) * impulse.y - (point.y - m_sweep.c.y) * impulse.x);
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
	public Vec2 getWorldLocation(final Vec2 localPoint){
		return XForm.mul(m_xf, localPoint);
	}

	/**
	 * Get the world coordinates of a point given the local coordinates.
	 * @param localPoint a point on the body measured relative the the body's origin.
	 * @param out where to put the same point expressed in world coordinates.
	 */
	public void getWorldLocationToOut(final Vec2 localPoint, final Vec2 out){
		XForm.mulToOut( m_xf, localPoint, out);
	}

	/**
	 * Get the world coordinates of a point given the local coordinates.
	 * @param localPoint a point on the body measured relative the the body's origin.
	 * @return the same point expressed in world coordinates.
	 * @deprecated Use getWorldLocation instead (clearer naming convention)
	 */
	@Deprecated
	public Vec2 getWorldPoint(final Vec2 localPoint) {
		return getWorldLocation(localPoint);
	}

	/**
	 * Get the world coordinates of a vector given the local coordinates.
	 * @param localVector a vector fixed in the body.
	 * @return the same vector expressed in world coordinates.
	 * @deprecated Use getWorldDirection instead (clearer naming convention)
	 */
	@Deprecated
	public Vec2 getWorldVector(final Vec2 localVector){
		return getWorldDirection(localVector);
	}

	/**
	 * Get the world coordinates of a direction given the local direction.
	 * @param localDirection a vector fixed in the body.
	 * @return the same vector expressed in world coordinates.
	 */
	public Vec2 getWorldDirection(final Vec2 localDirection) {
		return Mat22.mul(m_xf.R, localDirection);
	}

	/**
	 * Get the world coordinates of a direction given the local direction.
	 * @param localDirection a vector fixed in the body.
	 * @param out where to put the same vector expressed in world coordinates.
	 */
	public void getWorldDirectionToOut(final Vec2 localDirection, final Vec2 out){
		Mat22.mulToOut( m_xf.R, localDirection, out);
	}

	/**
	 * Gets a local point relative to the body's origin given a world point.
	 * @param worldPoint a point in world coordinates.
	 * @return the corresponding local point relative to the body's origin.
	 */
	public Vec2 getLocalPoint(final Vec2 worldPoint){
		return XForm.mulTrans(m_xf, worldPoint);
	}

	/**
	 * Gets a local point relative to the body's origin given a world point.
	 * @param worldPoint a point in world coordinates.
	 * @param out where to put the the corresponding local point relative to the body's origin.
	 */
	public void getLocalPointToOut(final Vec2 worldPoint, final Vec2 out){
		XForm.mulTransToOut(m_xf, worldPoint, out);
	}

	/**
	 * Gets a local vector given a world vector.
	 * @param worldVector a vector in world coordinates.
	 * @return the corresponding local vector.
	 */
	public Vec2 getLocalVector(final Vec2 worldVector){
		return Mat22.mulTrans(m_xf.R, worldVector);
	}

	/**
	 * Gets a local vector given a world vector.
	 * @param worldVector a vector in world coordinates.
	 * @param out where to put the corresponding local vector.
	 */
	public void getLocalVectorToOut(final Vec2 worldVector, final Vec2 out){
		Mat22.mulTransToOut( m_xf.R, worldVector, out); // bug fix, thanks Keraj
	}

	/** Is this body treated like a bullet for continuous collision detection? */
	public boolean isBullet(){
		return (m_flags & e_bulletFlag) == e_bulletFlag;
	}

	/**
	 * Should this body be treated like a bullet for continuous collision detection?
	 * Use sparingly, as continuous collision detection can be expensive.
	 */
	public void setBullet(final boolean flag){
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
	public void allowSleeping(final boolean flag){
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

	/*
	 * Get the linked list of all contacts attached to this body.
	 * @return first ContactEdge in linked list
	 */
	/* Removed from C++ version
	public ContactEdge getContactList(){
		return m_contactList;
	}*/

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


	// djm pooled
	private static final TLXForm tlXf1 = new TLXForm();
	/** For internal use only. */
	public boolean synchronizeShapes(){
		// INLINED
		final XForm xf1 = tlXf1.get();
		xf1.R.set(m_sweep.a0);
		Mat22 R = xf1.R;
		Vec2 v = m_sweep.localCenter;
		xf1.position.set(m_sweep.c0.x - (R.col1.x * v.x + R.col2.x * v.y),
						 m_sweep.c0.y - (R.col1.y * v.x + R.col2.y * v.y));

		boolean inRange = true;
		for (Shape s = m_shapeList; s != null; s = s.m_next) {
			inRange = s.synchronize(m_world.m_broadPhase, xf1, m_xf);
			if (inRange == false) {
				break;
			}
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
		//m_xf.position.set(m_sweep.c.sub(Mat22.mul(m_xf.R,m_sweep.localCenter)));
		final Vec2 v1 = m_sweep.localCenter;
		m_xf.position.x = m_sweep.c.x - (m_xf.R.col1.x * v1.x + m_xf.R.col2.x * v1.y);
		m_xf.position.y = m_sweep.c.y - (m_xf.R.col1.y * v1.x + m_xf.R.col2.y * v1.y);
		//System.out.println(m_xf);
	}

	/**
	 * This is used to prevent connected bodies from colliding.
	 * It may lie, depending on the collideConnected flag, so
	 * it won't be very useful external to the engine.
	 */
	public boolean isConnected(final Body other){
		for (JointEdge jn = m_jointList; jn != null; jn = jn.next) {
			if (jn.other == other) {
				//System.out.println("connected");
				return (jn.joint.m_collideConnected == false);
			}
		}
		return false;
	}

	/** For internal use only. */
	public void advance(final float t){
		// Advance to the new safe time
		m_sweep.advance(t);
		m_sweep.c.set(m_sweep.c0);
		m_sweep.a = m_sweep.a0;
		synchronizeTransform();
	}

	/**
	 * Get the world linear velocity of a world point attached to this body.
	 * @param worldPoint a point in world coordinates.
	 * @return the world velocity of a point.
	 */
	// djm optimized
	public Vec2 getLinearVelocityFromWorldPoint(final Vec2 worldPoint) {
		final float ax = worldPoint.x - m_sweep.c.x;
		final float ay = worldPoint.y - m_sweep.c.y;
		final float vx = -m_angularVelocity * ay;
		final float vy = m_angularVelocity * ax;
		return new Vec2(m_linearVelocity.x + vx, m_linearVelocity.y + vy);

		/*Vec2 out = new Vec2(worldPoint);
		out.subLocal( m_sweep.c);
		Vec2.crossToOut(m_angularVelocity, out, out);
		out.add(m_linearVelocity);
		return out;*/
		//return m_linearVelocity.add(Vec2.cross(m_angularVelocity, worldPoint.sub(m_sweep.c)));
	}

	/**
	 * Get the world linear velocity of a world point attached to this body.
	 * @param worldPoint a point in world coordinates.
	 * @param out where to put the world velocity of a point.
	 */
	// djm optimized
	public void getLinearVelocityFromWorldPointToOut(final Vec2 worldPoint, final Vec2 out) {
		final float ax = worldPoint.x - m_sweep.c.x;
		final float ay = worldPoint.y - m_sweep.c.y;
		final float vx = -m_angularVelocity * ay;
		final float vy = m_angularVelocity * ax;
		out.set(m_linearVelocity.x + vx, m_linearVelocity.y + vy);
		/*
		out.set( worldPoint);
		out.subLocal( m_sweep.c);
		Vec2.crossToOut( m_angularVelocity, out, out);
		out.add( m_linearVelocity);*/
	}

	/**
	 * Get the world velocity of a local point.
	 * @param localPoint a point in local coordinates.
	 * @return the world velocity of a point.
	 */
	// djm optimized
	public Vec2 getLinearVelocityFromLocalPoint(final Vec2 localPoint) {
		final Vec2 out = new Vec2();
		getWorldLocationToOut(localPoint, out);
		final float ax = out.x - m_sweep.c.x;
		final float ay = out.y - m_sweep.c.y;
		final float vx = -m_angularVelocity * ay;
		final float vy = m_angularVelocity * ax;
		out.x = m_linearVelocity.x + vx;
		out.y = m_linearVelocity.y + vy;
		return out;
		//Vec2 worldLocation = getWorldLocationToOut(localPoint);
		//getLinearVelocityFromWorldPointToOut(worldLocation, worldLocation);
		//return worldLocation;
	}

	/**
	 * Get the world velocity of a local point.
	 * @param localPoint a point in local coordinates.
	 * @param out where to put the world velocity of a point.
	 */
	// djm optimized
	public void getLinearVelocityFromLocalPointToOut(final Vec2 localPoint, final Vec2 out) {
		//getWorldLocationToOut(localPoint, out);
		//getLinearVelocityFromWorldPointToOut(out, out);
		getWorldLocationToOut(localPoint, out);
		final float ax = out.x - m_sweep.c.x;
		final float ay = out.y - m_sweep.c.y;
		final float vx = -m_angularVelocity * ay;
		final float vy = m_angularVelocity * ax;
		out.x = m_linearVelocity.x + vx;
		out.y = m_linearVelocity.y + vy;
	}

	/**
	 * Put this body to sleep so it will stop simulating.
	 * This also sets the velocity to zero.
	 */
	public void putToSleep() {
		m_flags |= e_sleepFlag;
		m_sleepTime = 0.0f;
		m_linearVelocity.setZero();
		m_angularVelocity = 0.0f;
		m_force.setZero();
		m_torque = 0.0f;
	}

	public void setUserData(final Object data) {
		m_userData = data;
	}

	public World getWorld() {
		return m_world;
	}
	
	/**
	 * Get the contact list, represented as a linked list of ContactEdges. Will
	 * return null if no contacts are present.
	 * 
	 * @return the head of the linked list of contacts
	 */
	public ContactEdge getContactList() {
		return m_contactList;
	}
	
	/**
	 * Get the set of bodies in contact with this body.
	 * 
	 * @return all bodies touching this one
	 */
	public Set<Body> getBodiesInContact() {
		Set<Body> mySet = new HashSet<Body>();
		ContactEdge edge = getContactList();
		while (edge != null) {
			if (edge.contact.getManifoldCount() > 0) {
				mySet.add(edge.other);
			}
			edge = edge.next;
		}
		return mySet;
	}
	
	/**
	 * Get the set of bodies connected to this one by a joint.
	 * Note: this does not return the entire island of connected bodies,
	 * only those directly connected to this one.
	 * @return all bodies connected directly to this body by a joint
	 */
	public Set<Body> getConnectedBodies() {
		// TODO djm: pool this
		Set<Body> mySet = new HashSet<Body>();
		JointEdge edge = getJointList();
		while (edge != null) {
			mySet.add(edge.other);
			edge = edge.next;
		}
		return mySet;
	}
	
	/**
	 * Get the set of dynamic bodies connected to this one by a joint.
	 * Note: this does not return the entire island of connected bodies,
	 * only those directly connected to this one.
	 * @return all bodies connected directly to this body by a joint
	 */
	public Set<Body> getConnectedDynamicBodies() {
		// TODO djm: pool this
		Set<Body> mySet = new HashSet<Body>();
		JointEdge edge = getJointList();
		while (edge != null) {
			if (edge.other.isDynamic()) mySet.add(edge.other);
			edge = edge.next;
		}
		return mySet;
	}
	
	/**
	 * Get the island of connected bodies, including the current body.
	 * <em>Warning</em>: will continue walking the joint tree past static bodies,
	 * which may lead to unwanted results esp. if bodies are connected to the ground
	 * body.
	 * @return Set<Body> of all bodies accessible from this one by walking the joint tree
	 */
	public Set<Body> getConnectedBodyIsland() {
		// TODO djm: pool this
		Set<Body> result = new HashSet<Body>();
		result.add(this);
		return getConnectedBodyIsland_impl(this, result);
	}
	
	/* Recursive implementation. */
	private Set<Body> getConnectedBodyIsland_impl(final Body parent, final Set<Body> parentResult) {
		Set<Body> connected = getConnectedBodies();
		for (Body b:connected) {
			if (b == parent || parentResult.contains(b)) continue; //avoid infinite recursion
			parentResult.add(b);
			parentResult.addAll(b.getConnectedBodyIsland_impl(b, parentResult));
		}
		return parentResult;
	}
	
	/**
	 * Get the island of joint-connected dynamic bodies, including the current body.
	 * Stops walking tree if it encounters a static body.
	 * @see Body#getConnectedBodyIsland()
	 * @return Set<Body> of all bodies accessible from this one by walking the joint tree
	 */
	public Set<Body> getConnectedDynamicBodyIsland() {
		// TODO djm: pool this
		Set<Body> result = new HashSet<Body>();
		if (!this.isDynamic()) return result;
		result.add(this);
		return getConnectedDynamicBodyIsland_impl(this, result);
	}
	
	private Set<Body> getConnectedDynamicBodyIsland_impl(final Body parent, final Set<Body> parentResult) {
		// TODO djm: pool this
		Set<Body> connected = getConnectedBodies();
		for (Body b:connected) {
			if (b == parent || !b.isDynamic() || parentResult.contains(b)) continue; //avoid infinite recursion
			parentResult.add(b);
			parentResult.addAll(b.getConnectedDynamicBodyIsland_impl(b, parentResult));
		}
		return parentResult;
	}
	
	/**
	 * Get the island of bodies in contact, including the current body.
	 * <em>Warning</em>: will continue walking the contact tree past static bodies,
	 * which may lead to unwanted results esp. if bodies are touching the ground
	 * body.
	 * @return Set<Body> of all bodies accessible from this one by walking the contact tree
	 */
	public Set<Body> getTouchingBodyIsland() {
		// TODO djm: pool this
		Set<Body> result = new HashSet<Body>();
		result.add(this);
		return getTouchingBodyIsland_impl(this, result);
	}
	
	/* Recursive implementation. */
	private Set<Body> getTouchingBodyIsland_impl(final Body parent, final Set<Body> parentResult) {
		Set<Body> touching = getBodiesInContact();
		for (Body b:touching) {
			if (b == parent || parentResult.contains(b)) continue; //avoid infinite recursion
			parentResult.add(b);
			parentResult.addAll(b.getTouchingBodyIsland_impl(b, parentResult));
		}
		return parentResult;
	}
	
	/**
	 * Get the island of dynamic bodies in contact, including the current body.
	 * Stops walking tree if it encounters a static body.
	 * @return Set<Body> of all bodies accessible from this one by walking the contact tree
	 */
	public Set<Body> getTouchingDynamicBodyIsland() {
		// TODO djm: pool this
		Set<Body> result = new HashSet<Body>();
		result.add(this);
		return getTouchingDynamicBodyIsland_impl(this, result);
	}
	
	/* Recursive implementation. */
	private Set<Body> getTouchingDynamicBodyIsland_impl(final Body parent, final Set<Body> parentResult) {
		// TODO djm: pool this
		Set<Body> touching = getBodiesInContact();
		for (Body b:touching) {
			if (b == parent || !b.isDynamic() || parentResult.contains(b)) continue; //avoid infinite recursion
			parentResult.add(b);
			parentResult.addAll(b.getTouchingDynamicBodyIsland_impl(b, parentResult));
		}
		return parentResult;
	}
	
	/**
	 * @return true if this Body is currently in contact with the passed body
	 */
	public boolean isTouching(Body other) {
		ContactEdge edge = getContactList();
		while (edge != null) {
			if(edge.other == other && edge.contact.getManifoldCount() > 0) return true;
			edge = edge.next;
		}
		return false;
	}

	//defaults seem fine
//	@Override
//	public boolean equals(Object obj) {
//	    return (obj == this);
//	}
//
//	@Override
//	public int hashCode() { 
//		return m_uniqueID;
//	}
	
	public void setLinearDamping(float damping) {
		m_linearDamping = damping;
	}
	public float getLinearDamping() { return m_linearDamping; }
	
	public void setAngularDamping(float damping) {
		m_angularDamping = damping;
	}
	public float getAngularDamping() { return m_angularDamping; }
}

