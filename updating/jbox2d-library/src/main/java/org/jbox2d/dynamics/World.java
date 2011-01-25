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

import java.util.Stack;

import org.jbox2d.callbacks.ContactFilter;
import org.jbox2d.callbacks.ContactListener;
import org.jbox2d.callbacks.DebugDraw;
import org.jbox2d.callbacks.DestructionListener;
import org.jbox2d.callbacks.QueryCallback;
import org.jbox2d.callbacks.RayCastCallback;
import org.jbox2d.callbacks.TreeCallback;
import org.jbox2d.callbacks.TreeRayCastCallback;
import org.jbox2d.collision.AABB;
import org.jbox2d.collision.broadphase.BroadPhase;
import org.jbox2d.collision.broadphase.DynamicTreeNode;
import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.collision.shapes.ShapeType;
import org.jbox2d.common.Color3f;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Sweep;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.contacts.CircleContact;
import org.jbox2d.dynamics.contacts.Contact;
import org.jbox2d.dynamics.contacts.ContactCreator;
import org.jbox2d.dynamics.contacts.ContactEdge;
import org.jbox2d.dynamics.contacts.ContactRegister;
import org.jbox2d.dynamics.contacts.PolygonAndCircleContact;
import org.jbox2d.dynamics.contacts.PolygonContact;
import org.jbox2d.dynamics.contacts.TOISolver;
import org.jbox2d.dynamics.joints.Joint;
import org.jbox2d.dynamics.joints.JointDef;
import org.jbox2d.dynamics.joints.JointEdge;
import org.jbox2d.dynamics.joints.PulleyJoint;
import org.jbox2d.pooling.MutableStack;
import org.jbox2d.pooling.PolygonContactStack;
import org.jbox2d.pooling.WorldPool;
import org.jbox2d.pooling.arrays.Vec2Array;
import org.jbox2d.pooling.stacks.TLStack;
import org.jbox2d.structs.collision.RayCastInput;
import org.jbox2d.structs.collision.RayCastOutput;
import org.jbox2d.structs.collision.TOIInput;
import org.jbox2d.structs.collision.TOIOutput;
import org.jbox2d.structs.collision.TOIOutput.TOIOutputState;

/**
 * The world class manages all physics entities, dynamic simulation,
 * and asynchronous queries. The world also contains efficient memory
 * management facilities.
 * 
 * @author Daniel Murphy
 */
public class World {
	public static final int WORLD_POOL_SIZE = 100;
		
	public static final int NEW_FIXTURE = 0x0001;
	public static final int LOCKED = 0x0002;
	public static final int CLEAR_FORCES = 0x0004;
	
	
	// statistics gathering
	public int activeContacts = 0;
	public int contactPoolCount = 0;
	
	protected int m_flags;
	
	protected ContactManager m_contactManager;
	
	private Body m_bodyList;
	private Joint m_jointList;
	
	private int m_bodyCount;
	private int m_jointCount;
	
	private final Vec2 m_gravity = new Vec2();
	private boolean m_allowSleep;
	
	// private Body m_groundBody;
	
	private DestructionListener m_destructionListener;
	private DebugDraw m_debugDraw;
	
	private final WorldPool pool;
	
	/**
	 * This is used to compute the time step ratio to
	 * support a variable time step.
	 */
	private float m_inv_dt0;
	
	/**
	 * This is for debugging the solver.
	 */
	private boolean m_warmStarting;
	
	/**
	 * This is for debugging the solver.
	 */
	private boolean m_continuousPhysics;
	
	private ContactRegister[][] contactStacks = new ContactRegister[ShapeType.TYPE_COUNT][ShapeType.TYPE_COUNT];
	private boolean c_initialized = false;
	
	/**
	 * Construct a world object.
	 * 
	 * @param gravity
	 *            the world gravity vector.
	 * @param doSleep
	 *            improve performance by not simulating inactive bodies.
	 */
	public World(Vec2 gravity, boolean doSleep) {
		pool = new WorldPool(WORLD_POOL_SIZE);
		m_destructionListener = null;
		m_debugDraw = null;
		
		m_bodyList = null;
		m_jointList = null;
		
		m_bodyCount = 0;
		m_jointCount = 0;
		
		m_warmStarting = true;
		m_continuousPhysics = true;
		
		m_allowSleep = doSleep;
		m_gravity.set(gravity);
		
		m_flags = CLEAR_FORCES;
		
		m_inv_dt0 = 0f;
		
		m_contactManager = new ContactManager(this);
		
		initializeRegisters();
	}
	
	private void addType(MutableStack<Contact> creator, ShapeType type1,
			ShapeType type2) {
		ContactRegister register = new ContactRegister();
		register.creator = creator;
		register.primary = true;
		contactStacks[type1.intValue][type2.intValue] = register;

		if (type1 != type2) {
			ContactRegister register2 = new ContactRegister();
			register2.creator = creator;
			register2.primary = false;
			contactStacks[type2.intValue][type1.intValue] = register2;
		}
	}

	private void initializeRegisters() {
		addType(pool.getCircleContactStack(), ShapeType.CIRCLE, ShapeType.CIRCLE);
		addType(pool.getPolyCircleContactStack(), ShapeType.POLYGON, ShapeType.CIRCLE);
		addType(pool.getPolyContactStack(), ShapeType.POLYGON, ShapeType.POLYGON);
	}

	public Contact popContact(Fixture fixtureA, Fixture fixtureB) {
		final ShapeType type1 = fixtureA.getType();
		final ShapeType type2 = fixtureB.getType();

		final ContactRegister reg = contactStacks[type1.intValue][type2.intValue];
		final MutableStack<Contact> creator = reg.creator;
		if (creator != null) {
			if (reg.primary) {
				Contact c = creator.pop();
				c.init(fixtureA, fixtureB);
				return c;
			} else {
				Contact c = creator.pop();
				c.init(fixtureB, fixtureA);
				return c;
			}
		} else {
			return null;
		}
	}

	public void pushContact(Contact contact) {

		if (contact.m_manifold.pointCount > 0) {
			contact.getFixtureA().getBody().setAwake(true);
			contact.getFixtureB().getBody().setAwake(true);
		}

		ShapeType type1 = contact.getFixtureA().getType();
		ShapeType type2 = contact.getFixtureB().getType();

		MutableStack<Contact> creator = contactStacks[type1.intValue][type2.intValue].creator;
		creator.push(contact);
	}
	
	public WorldPool getPool() {
		return pool;
	}
	
	/**
	 * Register a destruction listener. The listener is owned by you and must
	 * remain in scope.
	 * 
	 * @param listener
	 */
	public void setDestructionListener(DestructionListener listener) {
		m_destructionListener = listener;
	}
	
	/**
	 * Register a contact filter to provide specific control over collision.
	 * Otherwise the default filter is used (_defaultFilter). The listener is
	 * owned by you and must remain in scope.
	 * 
	 * @param filter
	 */
	public void setContactFilter(ContactFilter filter) {
		m_contactManager.m_contactFilter = filter;
	}
	
	/**
	 * Register a contact event listener. The listener is owned by you and must
	 * remain in scope.
	 * 
	 * @param listener
	 */
	public void setContactListener(ContactListener listener) {
		m_contactManager.m_contactListener = listener;
	}
	
	/**
	 * Register a routine for debug drawing. The debug draw functions are called
	 * inside with World.DrawDebugData method. The debug draw object is owned
	 * by you and must remain in scope.
	 * 
	 * @param debugDraw
	 */
	public void setDebugDraw(DebugDraw debugDraw) {
		m_debugDraw = debugDraw;
	}
	
	/**
	 * create a rigid body given a definition. No reference to the definition
	 * is retained.
	 * 
	 * @warning This function is locked during callbacks.
	 * @param def
	 * @return
	 */
	public Body createBody(BodyDef def) {
		assert (isLocked() == false);
		if (isLocked()) {
			return null;
		}
		// TODO djm pooling
		Body b = new Body(def, this);
		
		// add to world doubly linked list
		b.m_prev = null;
		b.m_next = m_bodyList;
		if (m_bodyList != null) {
			m_bodyList.m_prev = b;
		}
		m_bodyList = b;
		++m_bodyCount;
		
		return b;
	}
	
	/**
	 * destroy a rigid body given a definition. No reference to the definition
	 * is retained. This function is locked during callbacks.
	 * 
	 * @warning This automatically deletes all associated shapes and joints.
	 * @warning This function is locked during callbacks.
	 * @param body
	 */
	public void destroyBody(Body body) {
		assert (m_bodyCount > 0);
		assert (isLocked() == false);
		if (isLocked()) {
			return;
		}
		
		// Delete the attached joints.
		JointEdge je = body.m_jointList;
		while (je != null) {
			JointEdge je0 = je;
			je = je.next;
			if (m_destructionListener != null) {
				m_destructionListener.sayGoodbye(je0.joint);
			}
			
			destroyJoint(je0.joint);
		}
		body.m_jointList = null;
		
		// Delete the attached contacts.
		ContactEdge ce = body.m_contactList;
		while (ce != null) {
			ContactEdge ce0 = ce;
			ce = ce.next;
			m_contactManager.destroy(ce0.contact);
		}
		body.m_contactList = null;
		
		Fixture f = body.m_fixtureList;
		while (f != null) {
			Fixture f0 = f;
			f = f.m_next;
			
			if (m_destructionListener != null) {
				m_destructionListener.sayGoodbye(f0);
			}
			
			f0.destroyProxy(m_contactManager.m_broadPhase);
			f0.destroy();
			// TODO djm recycle fixtures (here or in that destroy method)
		}
		body.m_fixtureList = null;
		body.m_fixtureCount = 0;
		
		// Remove world body list.
		if (body.m_prev != null) {
			body.m_prev.m_next = body.m_next;
		}
		
		if (body.m_next != null) {
			body.m_next.m_prev = body.m_prev;
		}
		
		if (body == m_bodyList) {
			m_bodyList = body.m_next;
		}
		
		--m_bodyCount;
		// TODO djm recycle body
	}
	
	/**
	 * create a joint to constrain bodies together. No reference to the definition
	 * is retained. This may cause the connected bodies to cease colliding.
	 * 
	 * @warning This function is locked during callbacks.
	 * @param def
	 * @return
	 */
	public Joint createJoint(JointDef def) {
		assert (isLocked() == false);
		if (isLocked()) {
			return null;
		}
		
		Joint j = Joint.create(this, def);
		
		// Connect to the world list.
		j.m_prev = null;
		j.m_next = m_jointList;
		if (m_jointList != null) {
			m_jointList.m_prev = j;
		}
		m_jointList = j;
		++m_jointCount;
		
		// Connect to the bodies' doubly linked lists.
		j.m_edgeA.joint = j;
		j.m_edgeA.other = j.m_bodyB;
		j.m_edgeA.prev = null;
		j.m_edgeA.next = j.m_bodyA.m_jointList;
		if (j.m_bodyA.m_jointList != null) {
			j.m_bodyA.m_jointList.prev = j.m_edgeA;
		}
		j.m_bodyA.m_jointList = j.m_edgeA;
		
		j.m_edgeB.joint = j;
		j.m_edgeB.other = j.m_bodyA;
		j.m_edgeB.prev = null;
		j.m_edgeB.next = j.m_bodyB.m_jointList;
		if (j.m_bodyB.m_jointList != null) {
			j.m_bodyB.m_jointList.prev = j.m_edgeB;
		}
		j.m_bodyB.m_jointList = j.m_edgeB;
		
		Body bodyA = def.bodyA;
		Body bodyB = def.bodyB;
		
		// If the joint prevents collisions, then flag any contacts for filtering.
		if (def.collideConnected == false) {
			ContactEdge edge = bodyB.getContactList();
			while (edge != null) {
				if (edge.other == bodyA) {
					// Flag the contact for filtering at the next time step (where either
					// body is awake).
					edge.contact.flagForFiltering();
				}
				
				edge = edge.next;
			}
		}
		
		// Note: creating a joint doesn't wake the bodies.
		
		return j;
	}
	
	/**
	 * destroy a joint. This may cause the connected bodies to begin colliding.
	 * 
	 * @warning This function is locked during callbacks.
	 * @param joint
	 */
	public void destroyJoint(Joint j) {
		assert (isLocked() == false);
		if (isLocked()) {
			return;
		}
		
		boolean collideConnected = j.m_collideConnected;
		
		// Remove from the doubly linked list.
		if (j.m_prev != null) {
			j.m_prev.m_next = j.m_next;
		}
		
		if (j.m_next != null) {
			j.m_next.m_prev = j.m_prev;
		}
		
		if (j == m_jointList) {
			m_jointList = j.m_next;
		}
		
		// Disconnect from island graph.
		Body bodyA = j.m_bodyA;
		Body bodyB = j.m_bodyB;
		
		// Wake up connected bodies.
		bodyA.setAwake(true);
		bodyB.setAwake(true);
		
		// Remove from body 1.
		if (j.m_edgeA.prev != null) {
			j.m_edgeA.prev.next = j.m_edgeA.next;
		}
		
		if (j.m_edgeA.next != null) {
			j.m_edgeA.next.prev = j.m_edgeA.prev;
		}
		
		if (j.m_edgeA == bodyA.m_jointList) {
			bodyA.m_jointList = j.m_edgeA.next;
		}
		
		j.m_edgeA.prev = null;
		j.m_edgeA.next = null;
		
		// Remove from body 2
		if (j.m_edgeB.prev != null) {
			j.m_edgeB.prev.next = j.m_edgeB.next;
		}
		
		if (j.m_edgeB.next != null) {
			j.m_edgeB.next.prev = j.m_edgeB.prev;
		}
		
		if (j.m_edgeB == bodyB.m_jointList) {
			bodyB.m_jointList = j.m_edgeB.next;
		}
		
		j.m_edgeB.prev = null;
		j.m_edgeB.next = null;
		
		Joint.destroy(j);
		
		assert (m_jointCount > 0);
		--m_jointCount;
		
		// If the joint prevents collisions, then flag any contacts for filtering.
		if (collideConnected == false) {
			ContactEdge edge = bodyB.getContactList();
			while (edge != null) {
				if (edge.other == bodyA) {
					// Flag the contact for filtering at the next time step (where either
					// body is awake).
					edge.contact.flagForFiltering();
				}
				
				edge = edge.next;
			}
		}
	}
	
	// djm pooling
	private final TimeStep step = new TimeStep();
	
	/**
	 * Take a time step. This performs collision detection, integration,
	 * and constraint solution.
	 * 
	 * @param timeStep
	 *            the amount of time to simulate, this should not vary.
	 * @param velocityIterations
	 *            for the velocity constraint solver.
	 * @param positionIterations
	 *            for the position constraint solver.
	 */
	public void step(float dt, int velocityIterations, int positionIterations) {
		// log.debug("Starting step");
		// If new fixtures were added, we need to find the new contacts.
		if ((m_flags & NEW_FIXTURE) == NEW_FIXTURE) {
			// log.debug("There's a new fixture, lets look for new contacts");
			m_contactManager.findNewContacts();
			m_flags &= ~NEW_FIXTURE;
		}
		
		m_flags |= LOCKED;
		
		step.dt = dt;
		step.velocityIterations = velocityIterations;
		step.positionIterations = positionIterations;
		if (dt > 0.0f) {
			step.inv_dt = 1.0f / dt;
		}
		else {
			step.inv_dt = 0.0f;
		}
		
		step.dtRatio = m_inv_dt0 * dt;
		
		step.warmStarting = m_warmStarting;
		
		// Update contacts. This is where some contacts are destroyed.
		m_contactManager.collide();
		
		// Integrate velocities, solve velocity constraints, and integrate positions.
		if (step.dt > 0.0f) {
			solve(step);
		}
		
		// Handle TOI events.
		if (m_continuousPhysics && step.dt > 0.0f) {
			solveTOI();
		}
		
		if (step.dt > 0.0f) {
			m_inv_dt0 = step.inv_dt;
		}
		
		if ((m_flags & CLEAR_FORCES) == CLEAR_FORCES) {
			clearForces();
		}
		
		m_flags &= ~LOCKED;
		// log.debug("ending step");
	}
	
	/**
	 * Call this after you are done with time steps to clear the forces. You normally
	 * call this after each call to Step, unless you are performing sub-steps. By default,
	 * forces will be automatically cleared, so you don't need to call this function.
	 * 
	 * @see setAutoClearForces
	 */
	public void clearForces() {
		for (Body body = m_bodyList; body != null; body = body.getNext()) {
			body.m_force.setZero();
			body.m_torque = 0.0f;
		}
	}
	
	private final Color3f color = new Color3f();
	private final Transform xf = new Transform();
	private final Vec2 cA = new Vec2();
	private final Vec2 cB = new Vec2();
	private final static Vec2Array avs = new Vec2Array();
	
	/**
	 * Call this to draw shapes and other debug draw data.
	 */
	public void drawDebugData() {
		if (m_debugDraw == null) {
			return;
		}
		
		int flags = m_debugDraw.getFlags();
		
		if ((flags & DebugDraw.e_shapeBit) == DebugDraw.e_shapeBit) {
			for (Body b = m_bodyList; b != null; b = b.getNext()) {
				xf.set(b.getTransform());
				for (Fixture f = b.getFixtureList(); f != null; f = f.getNext()) {
					if (b.isActive() == false) {
						drawShape(f, xf, new Color3f(0.5f, 0.5f, 0.3f));
					}
					else if (b.getType() == BodyType.STATIC) {
						drawShape(f, xf, new Color3f(0.5f, 0.9f, 0.5f));
					}
					else if (b.getType() == BodyType.KINEMATIC) {
						drawShape(f, xf, new Color3f(0.5f, 0.5f, 0.9f));
					}
					else if (b.isAwake() == false) {
						drawShape(f, xf, new Color3f(0.6f, 0.6f, 0.6f));
					}
					else {
						drawShape(f, xf, new Color3f(0.9f, 0.7f, 0.7f));
					}
				}
			}
		}
		
		if ((flags & DebugDraw.e_jointBit) == DebugDraw.e_jointBit) {
			for (Joint j = m_jointList; j != null; j = j.getNext()) {
				drawJoint(j);
			}
		}
		
		if ((flags & DebugDraw.e_pairBit) == DebugDraw.e_pairBit) {
			color.set(0.3f, 0.9f, 0.9f);
			for (Contact c = m_contactManager.m_contactList; c != null; c = c.getNext()) {
				Fixture fixtureA = c.getFixtureA();
				Fixture fixtureB = c.getFixtureB();
				
				fixtureA.getAABB().getCenterToOut(cA);
				fixtureB.getAABB().getCenterToOut(cB);
				
				m_debugDraw.drawSegment(cA, cB, color);
			}
		}
		
		if ((flags & DebugDraw.e_aabbBit) == DebugDraw.e_aabbBit) {
			color.set(0.9f, 0.3f, 0.9f);
			
			for (Body b = m_bodyList; b != null; b = b.getNext()) {
				if (b.isActive() == false) {
					continue;
				}
				
				for (Fixture f = b.getFixtureList(); f != null; f = f.getNext()) {
					AABB aabb = f.m_proxy.aabb;
					Vec2[] vs = avs.get(4);
					vs[0].set(aabb.lowerBound.x, aabb.lowerBound.y);
					vs[1].set(aabb.upperBound.x, aabb.lowerBound.y);
					vs[2].set(aabb.upperBound.x, aabb.upperBound.y);
					vs[3].set(aabb.lowerBound.x, aabb.upperBound.y);
					
					m_debugDraw.drawPolygon(vs, 4, color);
					if ((b.m_flags & Body.e_toiFlag) == Body.e_toiFlag) {
						// log.debug("toi is on");
						Vec2 v = b.getWorldCenter();
						m_debugDraw.drawPoint(v, 5, color);
						// m_debugDraw.drawString(v.x, v.y, "toi is on", color);
					}
				}
			}
		}
		
		if ((flags & DebugDraw.e_centerOfMassBit) == DebugDraw.e_centerOfMassBit) {
			for (Body b = m_bodyList; b != null; b = b.getNext()) {
				xf.set(b.getTransform());
				xf.position.set(b.getWorldCenter());
				m_debugDraw.drawTransform(xf);
			}
		}
		
		if ((flags & DebugDraw.e_dynamicTreeBit) == DebugDraw.e_dynamicTreeBit) {
			m_contactManager.m_broadPhase.drawTree(m_debugDraw);
		}
	}
	
	private final WorldQueryWrapper wqwrapper = new WorldQueryWrapper();
	
	/**
	 * Query the world for all fixtures that potentially overlap the
	 * provided AABB.
	 * 
	 * @param callback
	 *            a user implemented callback class.
	 * @param aabb
	 *            the query box.
	 */
	public void queryAABB(QueryCallback callback, AABB aabb) {
		wqwrapper.broadPhase = m_contactManager.m_broadPhase;
		wqwrapper.callback = callback;
		m_contactManager.m_broadPhase.query(wqwrapper, aabb);
	}
	
	private final WorldRayCastWrapper wrcwrapper = new WorldRayCastWrapper();
	private final RayCastInput input = new RayCastInput();
	
	/**
	 * Ray-cast the world for all fixtures in the path of the ray. Your callback
	 * controls whether you get the closest point, any point, or n-points.
	 * The ray-cast ignores shapes that contain the starting point.
	 * 
	 * @param callback
	 *            a user implemented callback class.
	 * @param point1
	 *            the ray starting point
	 * @param point2
	 *            the ray ending point
	 */
	public void raycast(RayCastCallback callback, Vec2 point1, Vec2 point2) {
		wrcwrapper.broadPhase = m_contactManager.m_broadPhase;
		wrcwrapper.callback = callback;
		input.maxFraction = 1.0f;
		input.p1.set(point1);
		input.p2.set(point2);
		m_contactManager.m_broadPhase.raycast(wrcwrapper, input);
	}
	
	/**
	 * Get the world body list. With the returned body, use Body.getNext to get
	 * the next body in the world list. A null body indicates the end of the list.
	 * 
	 * @return the head of the world body list.
	 */
	public Body getBodyList() {
		return m_bodyList;
	}
	
	/**
	 * Get the world joint list. With the returned joint, use Joint.getNext to get
	 * the next joint in the world list. A null joint indicates the end of the list.
	 * 
	 * @return the head of the world joint list.
	 */
	public Joint getJointList() {
		return m_jointList;
	}
	
	/**
	 * Get the world contact list. With the returned contact, use Contact.getNext to get
	 * the next contact in the world list. A null contact indicates the end of the list.
	 * 
	 * @return the head of the world contact list.
	 * @warning contacts are
	 */
	public Contact getContactList() {
		return m_contactManager.m_contactList;
	}
	
	/**
	 * Enable/disable warm starting. For testing.
	 * 
	 * @param flag
	 */
	public void setWarmStarting(boolean flag) {
		m_warmStarting = flag;
	}
	
	/**
	 * Enable/disable continuous physics. For testing.
	 * 
	 * @param flag
	 */
	public void setContinuousPhysics(boolean flag) {
		m_continuousPhysics = flag;
	}
	
	/**
	 * Get the number of broad-phase proxies.
	 * 
	 * @return
	 */
	public int getProxyCount() {
		return m_contactManager.m_broadPhase.getProxyCount();
	}
	
	/**
	 * Get the number of bodies.
	 * 
	 * @return
	 */
	public int getBodyCount() {
		return m_bodyCount;
	}
	
	/**
	 * Get the number of joints.
	 * 
	 * @return
	 */
	public int getJointCount() {
		return m_jointCount;
	}
	
	/**
	 * Get the number of contacts (each may have 0 or more contact points).
	 * 
	 * @return
	 */
	public int getContactCount() {
		return m_contactManager.m_contactCount;
	}
	
	/**
	 * Change the global gravity vector.
	 * 
	 * @param gravity
	 */
	public void setGravity(Vec2 gravity) {
		m_gravity.set(gravity);
	}
	
	/**
	 * Get the global gravity vector.
	 * 
	 * @return
	 */
	public Vec2 getGravity() {
		return m_gravity;
	}
	
	/**
	 * Is the world locked (in the middle of a time step).
	 * 
	 * @return
	 */
	public boolean isLocked() {
		return (m_flags & LOCKED) == LOCKED;
	}
	
	/**
	 * Set flag to control automatic clearing of forces after each time step.
	 * 
	 * @param flag
	 */
	public void setAutoClearForces(boolean flag) {
		if (flag) {
			m_flags |= CLEAR_FORCES;
		}
		else {
			m_flags &= ~CLEAR_FORCES;
		}
	}
	
	/**
	 * Get the flag that controls automatic clearing of forces after each time step.
	 * 
	 * @return
	 */
	public boolean getAutoClearForces() {
		return (m_flags & CLEAR_FORCES) == CLEAR_FORCES;
	}
	
	private final Island island = new Island();
	private Body[] stack = new Body[10]; // TODO djm find a good initial stack number;
	
	private void solve(TimeStep step) {
		// Size the island for the worst case.
		island.init(m_bodyCount, m_contactManager.m_contactCount, m_jointCount, m_contactManager.m_contactListener);
		
		// Clear all the island flags.
		for (Body b = m_bodyList; b != null; b = b.m_next) {
			b.m_flags &= ~Body.e_islandFlag;
		}
		for (Contact c = m_contactManager.m_contactList; c != null; c = c.m_next) {
			c.m_flags &= ~Contact.ISLAND_FLAG;
		}
		for (Joint j = m_jointList; j != null; j = j.m_next) {
			j.m_islandFlag = false;
		}
		
		// Build and simulate all awake islands.
		int stackSize = m_bodyCount;
		if (stack.length < stackSize) {
			stack = new Body[stackSize];
		}
		for (Body seed = m_bodyList; seed != null; seed = seed.m_next) {
			if ((seed.m_flags & Body.e_islandFlag) == Body.e_islandFlag) {
				continue;
			}
			
			if (seed.isAwake() == false || seed.isActive() == false) {
				continue;
			}
			
			// The seed can be dynamic or kinematic.
			if (seed.getType() == BodyType.STATIC) {
				continue;
			}
			
			// Reset island and stack.
			island.clear();
			int stackCount = 0;
			stack[stackCount++] = seed;
			seed.m_flags |= Body.e_islandFlag;
			
			// Perform a depth first search (DFS) on the constraint graph.
			while (stackCount > 0) {
				// Grab the next body off the stack and add it to the island.
				Body b = stack[--stackCount];
				assert (b.isActive() == true);
				island.add(b);
				
				// Make sure the body is awake.
				b.setAwake(true);
				
				// To keep islands as small as possible, we don't
				// propagate islands across static bodies.
				if (b.getType() == BodyType.STATIC) {
					continue;
				}
				
				// Search all contacts connected to this body.
				for (ContactEdge ce = b.m_contactList; ce != null; ce = ce.next) {
					Contact contact = ce.contact;
					
					// Has this contact already been added to an island?
					if ((contact.m_flags & Contact.ISLAND_FLAG) == Contact.ISLAND_FLAG) {
						continue;
					}
					
					// Is this contact solid and touching?
					if (contact.isEnabled() == false || contact.isTouching() == false) {
						continue;
					}
					
					// Skip sensors.
					boolean sensorA = contact.m_fixtureA.m_isSensor;
					boolean sensorB = contact.m_fixtureB.m_isSensor;
					if (sensorA || sensorB) {
						continue;
					}
					
					island.add(contact);
					contact.m_flags |= Contact.ISLAND_FLAG;
					
					Body other = ce.other;
					
					// Was the other body already added to this island?
					if ((other.m_flags & Body.e_islandFlag) == Body.e_islandFlag) {
						continue;
					}
					
					assert (stackCount < stackSize);
					stack[stackCount++] = other;
					other.m_flags |= Body.e_islandFlag;
				}
				
				// Search all joints connect to this body.
				for (JointEdge je = b.m_jointList; je != null; je = je.next) {
					if (je.joint.m_islandFlag == true) {
						continue;
					}
					
					Body other = je.other;
					
					// Don't simulate joints connected to inactive bodies.
					if (other.isActive() == false) {
						continue;
					}
					
					island.add(je.joint);
					je.joint.m_islandFlag = true;
					
					if ((other.m_flags & Body.e_islandFlag) == Body.e_islandFlag) {
						continue;
					}
					
					assert (stackCount < stackSize);
					stack[stackCount++] = other;
					other.m_flags |= Body.e_islandFlag;
				}
			}
			
			island.solve(step, m_gravity, m_allowSleep);
			
			// Post solve cleanup.
			for (int i = 0; i < island.m_bodyCount; ++i) {
				// Allow static bodies to participate in other islands.
				Body b = island.m_bodies[i];
				if (b.getType() == BodyType.STATIC) {
					b.m_flags &= ~Body.e_islandFlag;
				}
			}
		}
		
		// Synchronize fixtures, check for out of range bodies.
		for (Body b = m_bodyList; b != null; b = b.getNext()) {
			// If a body was not in an island then it did not move.
			if ((b.m_flags & Body.e_islandFlag) == 0) {
				continue;
			}
			
			if (b.getType() == BodyType.STATIC) {
				continue;
			}
			
			// Update fixtures (for broad-phase).
			b.synchronizeFixtures();
		}
		
		// Look for new contacts.
		m_contactManager.findNewContacts();
	}
	
	private void solveTOI() {
		// Prepare all contacts.
		for (Contact c = m_contactManager.m_contactList; c != null; c = c.m_next) {
			// Enable the contact
			c.m_flags |= Contact.ENABLED_FLAG;
			
			// Set the number of TOI events for this contact to zero.
			c.m_toiCount = 0;
		}
		
		// Initialize the TOI flag.
		for (Body body = m_bodyList; body != null; body = body.m_next) {
			// Kinematic, and static bodies will not be affected by the TOI event.
			// If a body was not in an island then it did not move.
			if ((body.m_flags & Body.e_islandFlag) == 0 || body.getType() == BodyType.KINEMATIC
					|| body.getType() == BodyType.STATIC) {
				body.m_flags |= Body.e_toiFlag;
			}
			else {
				body.m_flags &= ~Body.e_toiFlag;
			}
		}
		
		// Collide non-bullets.
		for (Body body = m_bodyList; body != null; body = body.m_next) {
			if ((body.m_flags & Body.e_toiFlag) == Body.e_toiFlag) {
				continue;
			}
			
			if (body.isBullet() == true) {
				continue;
			}
			
			solveTOI(body);
			
			body.m_flags |= Body.e_toiFlag;
		}
		
		// Collide bullets.
		for (Body body = m_bodyList; body != null; body = body.m_next) {
			if ((body.m_flags & Body.e_toiFlag) == Body.e_toiFlag) {
				continue;
			}
			
			if (body.isBullet() == false) {
				continue;
			}
			
			solveTOI(body);
			
			body.m_flags |= Body.e_toiFlag;
		}
	}
	
	private final TOIInput toiInput = new TOIInput();
	private final TOIOutput toiOutput = new TOIOutput();
	private final Sweep backup = new Sweep();
	private final TOISolver toiSolver = new TOISolver();
	
	private void solveTOI(Body body) {
		// Find the minimum contact.
		Contact toiContact = null;
		float toi = 1.0f;
		Body toiOther = null;
		boolean found;
		int count;
		int iter = 0;
		
		boolean bullet = body.isBullet();
		
		// Iterate until all contacts agree on the minimum TOI. We have
		// to iterate because the TOI algorithm may skip some intermediate
		// collisions when objects rotate through each other.
		do {
			count = 0;
			found = false;
			for (ContactEdge ce = body.m_contactList; ce != null; ce = ce.next) {
				if (ce.contact == toiContact) {
					continue;
				}
				
				Body other = ce.other;
				BodyType type = other.getType();
				
				// Only bullets perform TOI with dynamic bodies.
				if (bullet == true) {
					// Bullets only perform TOI with bodies that have their TOI resolved.
					if ((other.m_flags & Body.e_toiFlag) == 0) {
						continue;
					}
					
					// No repeated hits on non-static bodies
					if (type != BodyType.STATIC && (ce.contact.m_flags & Contact.BULLET_HIT_FLAG) != 0) {
						continue;
					}
				}
				else if (type == BodyType.DYNAMIC) {
					continue;
				}
				
				// Check for a disabled contact.
				Contact contact = ce.contact;
				if (contact.isEnabled() == false) {
					continue;
				}
				
				// Prevent infinite looping.
				if (contact.m_toiCount > 10) {
					continue;
				}
				
				Fixture fixtureA = contact.m_fixtureA;
				Fixture fixtureB = contact.m_fixtureB;
				
				// Cull sensors.
				if (fixtureA.isSensor() || fixtureB.isSensor()) {
					continue;
				}
				
				Body bodyA = fixtureA.m_body;
				Body bodyB = fixtureB.m_body;
				
				// Compute the time of impact in interval [0, minTOI]
				toiInput.proxyA.set(fixtureA.getShape());
				toiInput.proxyB.set(fixtureB.getShape());
				toiInput.sweepA.set(bodyA.m_sweep);
				toiInput.sweepB.set(bodyB.m_sweep);
				toiInput.tMax = toi;
				
				pool.getTimeOfImpact().timeOfImpact(toiOutput, toiInput);
				
				if (toiOutput.state == TOIOutputState.TOUCHING && toiOutput.t < toi) {
					toiContact = contact;
					toi = toiOutput.t;
					toiOther = other;
					found = true;
				}
				
				++count;
			}
			
			++iter;
		}
		while (found && count > 1 && iter < 50);
		
		if (toiContact == null) {
			body.advance(1.0f);
			return;
		}
		
		backup.set(body.m_sweep);
		body.advance(toi);
		toiContact.update(m_contactManager.m_contactListener);
		if (toiContact.isEnabled() == false) {
			// Contact disabled. Backup and recurse.
			body.m_sweep.set(backup);
			solveTOI(body);
		}
		
		++toiContact.m_toiCount;
		
		// Update all the valid contacts on this body and build a contact island.
		final Contact[] contacts = new Contact[Settings.maxTOIContacts];
		count = 0;
		for (ContactEdge ce = body.m_contactList; ce != null && count < Settings.maxTOIContacts; ce = ce.next) {
			Body other = ce.other;
			BodyType type = other.getType();
			
			// Only perform correction with static bodies, so the
			// body won't get pushed out of the world.
			if (type == BodyType.DYNAMIC) {
				continue;
			}
			
			// Check for a disabled contact.
			Contact contact = ce.contact;
			if (contact.isEnabled() == false) {
				continue;
			}
			
			Fixture fixtureA = contact.m_fixtureA;
			Fixture fixtureB = contact.m_fixtureB;
			
			// Cull sensors.
			if (fixtureA.isSensor() || fixtureB.isSensor()) {
				continue;
			}
			
			// The contact likely has some new contact points. The listener
			// gives the user a chance to disable the contact.
			if (contact != toiContact) {
				contact.update(m_contactManager.m_contactListener);
			}
			
			// Did the user disable the contact?
			if (contact.isEnabled() == false) {
				// Skip this contact.
				continue;
			}
			
			if (contact.isTouching() == false) {
				continue;
			}
			
			contacts[count] = contact;
			++count;
		}
		
		// Reduce the TOI body's overlap with the contact island.
		toiSolver.initialize(contacts, count, body);
		
		float k_toiBaumgarte = 0.75f;
		// boolean solved = false;
		for (int i = 0; i < 20; ++i) {
			boolean contactsOkay = toiSolver.solve(k_toiBaumgarte);
			if (contactsOkay) {
				// solved = true;
				break;
			}
		}
		
		if (toiOther.getType() != BodyType.STATIC) {
			toiContact.m_flags |= Contact.BULLET_HIT_FLAG;
		}
	}
	
	private void drawJoint(Joint joint) {
		Body bodyA = joint.getBodyA();
		Body bodyB = joint.getBodyB();
		Transform xf1 = bodyA.getTransform();
		Transform xf2 = bodyB.getTransform();
		Vec2 x1 = xf1.position;
		Vec2 x2 = xf2.position;
		Vec2 p1 = pool.popVec2();
		Vec2 p2 = pool.popVec2();
		joint.getAnchorA(p1);
		joint.getAnchorB(p2);
		
		color.set(0.5f, 0.8f, 0.8f);
		
		switch (joint.getType()) {
			// TODO djm write after writing joints
			case DISTANCE :
				m_debugDraw.drawSegment(p1, p2, color);
				break;
			
			case PULLEY : {
				PulleyJoint pulley = (PulleyJoint) joint;
				Vec2 s1 = pulley.getGroundAnchorA();
				Vec2 s2 = pulley.getGroundAnchorB();
				m_debugDraw.drawSegment(s1, p1, color);
				m_debugDraw.drawSegment(s2, p2, color);
				m_debugDraw.drawSegment(s1, s2, color);
			}
				break;
			case CONSTANT_VOLUME :
			case MOUSE :
				// don't draw this
				break;
			default :
				m_debugDraw.drawSegment(x1, p1, color);
				m_debugDraw.drawSegment(p1, p2, color);
				m_debugDraw.drawSegment(x2, p2, color);
		}
		pool.pushVec2(2);
	}
	
	private final Vec2 center = new Vec2();
	private final Vec2 axis = new Vec2();
	private final Vec2Array tlvertices = new Vec2Array();
	
	private void drawShape(Fixture fixture, Transform xf, Color3f color) {
		switch (fixture.getType()) {
			case CIRCLE : {
				CircleShape circle = (CircleShape) fixture.getShape();
				
				// Vec2 center = Mul(xf, circle.m_p);
				Transform.mulToOut(xf, circle.m_p, center);
				float radius = circle.m_radius;
				axis.set(xf.R.col1);
				
				m_debugDraw.drawSolidCircle(center, radius, axis, color);
			}
				break;
			
			case POLYGON : {
				PolygonShape poly = (PolygonShape) fixture.getShape();
				int vertexCount = poly.m_vertexCount;
				assert (vertexCount <= Settings.maxPolygonVertices);
				Vec2[] vertices = tlvertices.get(Settings.maxPolygonVertices);
				
				for (int i = 0; i < vertexCount; ++i) {
					// vertices[i] = Mul(xf, poly.m_vertices[i]);
					Transform.mulToOut(xf, poly.m_vertices[i], vertices[i]);
				}
				
				m_debugDraw.drawSolidPolygon(vertices, vertexCount, color);
			}
				break;
		}
	}
}

class WorldQueryWrapper implements TreeCallback {
	public boolean treeCallback(DynamicTreeNode node) {
		Fixture fixture = (Fixture) node.userData;
		return callback.reportFixture(fixture);
	}
	
	BroadPhase broadPhase;
	QueryCallback callback;
};

class WorldRayCastWrapper implements TreeRayCastCallback {
	
	// djm pooling
	private final RayCastOutput output = new RayCastOutput();
	private final Vec2 temp = new Vec2();
	private final Vec2 point = new Vec2();
	
	public float raycastCallback(RayCastInput input, DynamicTreeNode node) {
		Object userData = node.userData;
		Fixture fixture = (Fixture) userData;
		boolean hit = fixture.raycast(output, input);
		
		if (hit) {
			float fraction = output.fraction;
			// Vec2 point = (1.0f - fraction) * input.p1 + fraction * input.p2;
			temp.set(input.p2).mulLocal(fraction);
			point.set(input.p1).mulLocal(1 - fraction).addLocal(temp);
			return callback.reportFixture(fixture, point, output.normal, fraction);
		}
		
		return input.maxFraction;
	}
	
	BroadPhase broadPhase;
	RayCastCallback callback;
};
