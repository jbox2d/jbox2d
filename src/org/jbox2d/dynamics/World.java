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

import java.util.ArrayList;
import org.jbox2d.collision.AABB;
import org.jbox2d.collision.BroadPhase;
import org.jbox2d.collision.OBB;
import org.jbox2d.collision.Pair;
import org.jbox2d.collision.PairManager;
import org.jbox2d.collision.Proxy;
import org.jbox2d.collision.Segment;
import org.jbox2d.collision.SegmentCollide;
import org.jbox2d.collision.SortKeyFunc;
import org.jbox2d.collision.TOI;
import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.collision.shapes.PointShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.collision.shapes.ShapeType;
import org.jbox2d.common.Color3f;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.RaycastResult;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;
import org.jbox2d.dynamics.contacts.Contact;
import org.jbox2d.dynamics.contacts.ContactEdge;
import org.jbox2d.dynamics.controllers.Controller;
import org.jbox2d.dynamics.controllers.ControllerDef;
import org.jbox2d.dynamics.controllers.ControllerEdge;
import org.jbox2d.dynamics.joints.ConstantVolumeJoint;
import org.jbox2d.dynamics.joints.Joint;
import org.jbox2d.dynamics.joints.JointDef;
import org.jbox2d.dynamics.joints.JointEdge;
import org.jbox2d.dynamics.joints.JointType;
import org.jbox2d.dynamics.joints.PulleyJoint;
import org.jbox2d.pooling.TLTimeStep;
import org.jbox2d.pooling.stacks.IslandStack;
import org.jbox2d.pooling.stacks.TimeStepStack;


//Updated to rev 56->118->142->150 of b2World.cpp/.h

/**
 * The world that physics takes place in.
 * <BR><BR>
 * To the extent that it is possible, avoid accessing members
 * directly, as in a future version their accessibility may
 * be rolled back - as un-Java as that is, we must follow
 * upstream C++ conventions, and for now everything is public
 * to speed development of Box2d, but it is subject to change.
 * You're warned!
 */
public class World {
	boolean m_lock;

	BroadPhase m_broadPhase;

	ContactManager m_contactManager;

	Body m_bodyList;

	/** Do not access, won't be useful! */
	Contact m_contactList;

	Joint m_jointList;
	
	Controller m_controllerList;
	
	int m_controllerCount;

	int m_bodyCount;

	int m_contactCount;

	int m_jointCount;

	Vec2 m_gravity;

	boolean m_allowSleep;

	Body m_groundBody;

	int m_positionIterationCount;

	/** Should we apply position correction? */
	boolean m_positionCorrection;
	/** Should we use warm-starting?  Improves stability in stacking scenarios. */
	boolean m_warmStarting;
	/** Should we enable continuous collision detection? */
	boolean m_continuousPhysics;

	DestructionListener m_destructionListener;
	BoundaryListener m_boundaryListener;
	ContactFilter m_contactFilter;
	ContactListener m_contactListener;
	DebugDraw m_debugDraw;

	boolean m_drawDebugData;

	private float m_inv_dt0;

	private final ArrayList<Steppable> postStepList;

	private boolean autoDebugDraw = true;
	
	
	/**
	 * @return the autoDebugDraw
	 */
	public boolean isAutoDebugDraw() {
		return autoDebugDraw;
	}

	/**
	 * @param autoDebugDraw the autoDebugDraw to set
	 */
	public void setAutoDebugDraw(boolean autoDebugDraw) {
		this.autoDebugDraw = autoDebugDraw;
	}

	public void setDrawDebugData(final boolean tf) {
		m_drawDebugData = tf;
	}

	public boolean isDrawingDebugData() {
		return m_drawDebugData;
	}

	/** Get the number of bodies. */
	public int getBodyCount() {
		return m_bodyCount;
	}

	/** Get the number of joints. */
	public int getJointCount() {
		return m_jointCount;
	}

	/** Get the number of contacts (each may have 0 or more contact points). */
	public int getContactCount() {
		return m_contactCount;
	}

	/** Change the global gravity vector. */
	public void setGravity(final Vec2 gravity) {
		m_gravity = gravity;
	}

	/** Get a clone of the global gravity vector.
	 * @return Clone of gravity vector
	 */
	public Vec2 getGravity() {
		return m_gravity.clone();
	}

	/** The world provides a single static ground body with no collision shapes.
	 *	You can use this to simplify the creation of joints and static shapes.
	 */
	public Body getGroundBody() {
		return m_groundBody;
	}

	/**
	 * Get the world body list. With the returned body, use Body.getNext() to get
	 * the next body in the world list. A NULL body indicates the end of the list.
	 * @return the head of the world body list.
	 */
	public Body getBodyList() {
		return m_bodyList;
	}

	/**
	 * Get the world joint list. With the returned joint, use Joint.getNext() to get
	 * the next joint in the world list. A NULL joint indicates the end of the list.
	 * @return the head of the world joint list.
	 */
	public Joint getJointList() {
		return m_jointList;
	}

	/**
	 * Construct a world object.
	 * @param worldAABB a bounding box that completely encompasses all your shapes.
	 * @param gravity the world gravity vector.
	 * @param doSleep improve performance by not simulating inactive bodies.
	 */
	public World(final AABB worldAABB, final Vec2 gravity, final boolean doSleep) {
		m_positionCorrection = true;
		m_warmStarting = true;
		m_continuousPhysics = true;
		m_destructionListener = null;
		m_boundaryListener = null;
		m_contactFilter = ContactFilter.DEFAULT_FILTER;//&b2_defaultFilter;
		m_contactListener = null;
		m_debugDraw = null;

		m_inv_dt0 = 0.0f;

		m_bodyList = null;
		m_contactList = null;
		m_jointList = null;
		m_controllerList = null;

		m_bodyCount = 0;
		m_contactCount = 0;
		m_jointCount = 0;
		m_controllerCount = 0;

		m_lock = false;

		m_allowSleep = doSleep;

		m_gravity = gravity;

		m_contactManager = new ContactManager();
		m_contactManager.m_world = this;
		m_broadPhase = new BroadPhase(worldAABB, m_contactManager);

		final BodyDef bd = new BodyDef();
		m_groundBody = createBody(bd);
		postStepList = new ArrayList<Steppable>();
		setDrawDebugData(true);
	}

	/** Register a destruction listener. */
	public void setDestructionListener(final DestructionListener listener) {
		m_destructionListener = listener;
	}

	/** Register a broad-phase boundary listener. */
	public void setBoundaryListener(final BoundaryListener listener) {
		m_boundaryListener = listener;
	}

	/** Register a contact event listener */
	public void setContactListener(final ContactListener listener) {
		m_contactListener = listener;
	}


	/**
	 *  Register a contact filter to provide specific control over collision.
	 *  Otherwise the default filter is used (b2_defaultFilter).
	 */
	public void setContactFilter(final ContactFilter filter) {
		m_contactFilter = filter;
	}

	/**
	 * Register a routine for debug drawing. The debug draw functions are called
	 * inside the World.step() method, so make sure your renderer is ready to
	 * consume draw commands when you call step().
	 */
	public void setDebugDraw(final DebugDraw debugDraw) {
		m_debugDraw = debugDraw;
	}

	public DebugDraw getDebugDraw() {
		return m_debugDraw;
	}


	/**
	 * Create a body given a definition. No reference to the definition
	 * is retained.  Body will be static unless mass is nonzero.
	 * <BR><em>Warning</em>: This function is locked during callbacks.
	 */
	public Body createBody(final BodyDef def) {
		assert(m_lock == false);
		if (m_lock == true) {
			return null;
		}

		final Body b = new Body(def, this);

		// Add to world doubly linked list.
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
	 * Destroy a rigid body given a definition. No reference to the definition
	 * is retained. This function is locked during callbacks.
	 * <BR><em>Warning</em>: This automatically deletes all associated shapes and joints.
	 * <BR><em>Warning</em>: This function is locked during callbacks.
	 */
	public void destroyBody(final Body b) {
		assert(m_bodyCount > 0);
		assert(m_lock == false);
		if (m_lock == true) {
			return;
		}

		// Delete the attached joints.
		JointEdge jn = b.m_jointList;
		while (jn != null) {
			final JointEdge jn0 = jn;
			jn = jn.next;

			if (m_destructionListener != null){
				m_destructionListener.sayGoodbye(jn0.joint);
			}

			destroyJoint(jn0.joint);
		}
		
		//Detach controllers attached to this body
		ControllerEdge ce = b.m_controllerList;
		while(ce != null) {
			ControllerEdge ce0 = ce;
			ce = ce.nextController;

			ce0.controller.removeBody(b);
		}

		// Delete the attached shapes. This destroys broad-phase
		// proxies and pairs, leading to the destruction of contacts.
		Shape s = b.m_shapeList;
		while (s != null) {
			final Shape s0 = s;
			s = s.m_next;

			if (m_destructionListener != null) {
				m_destructionListener.sayGoodbye(s0);
			}

			s0.destroyProxy(m_broadPhase);
			Shape.destroy(s0);
		}

		// Remove world body list.
		if (b.m_prev != null) {
			b.m_prev.m_next = b.m_next;
		}

		if (b.m_next != null) {
			b.m_next.m_prev = b.m_prev;
		}

		if (b == m_bodyList) {
			m_bodyList = b.m_next;
		}

		--m_bodyCount;
		//b->~b2Body();
	}

	/**
	 * Create a joint to constrain bodies together. No reference to the definition
	 * is retained. This may cause the connected bodies to cease colliding.
	 * <BR><em>Warning</em> This function is locked during callbacks.
	 */
	public Joint createJoint(final JointDef def) {
		assert(m_lock == false);

		final Joint j = Joint.create(def);

		// Connect to the world list.
		j.m_prev = null;
		j.m_next = m_jointList;
		if (m_jointList != null) {
			m_jointList.m_prev = j;
		}
		m_jointList = j;
		++m_jointCount;

		// Connect to the bodies' doubly linked lists
		j.m_node1.joint = j;
		j.m_node1.other = j.m_body2;
		j.m_node1.prev = null;
		j.m_node1.next = j.m_body1.m_jointList;
		if (j.m_body1.m_jointList != null) {
			j.m_body1.m_jointList.prev = j.m_node1;
		}
		j.m_body1.m_jointList = j.m_node1;

		j.m_node2.joint = j;
		j.m_node2.other = j.m_body1;
		j.m_node2.prev = null;
		j.m_node2.next = j.m_body2.m_jointList;
		if (j.m_body2.m_jointList != null) {
			j.m_body2.m_jointList.prev = j.m_node2;
		}
		j.m_body2.m_jointList = j.m_node2;

		// If the joint prevents collisions, then reset collision filtering
		if (def.collideConnected == false) {
			// Reset the proxies on the body with the minimum number of shapes.
			final Body b = def.body1.m_shapeCount < def.body2.m_shapeCount ? def.body1
			                                                               : def.body2;
			for (Shape s = b.m_shapeList; s != null; s = s.m_next) {
				s.refilterProxy(m_broadPhase, b.getMemberXForm());
			}
		}

		return j;
	}

	/**
	 * Destroy a joint. This may cause the connected bodies to begin colliding.
	 * <BR><em>Warning</em>: This function is locked during callbacks.
	 */
	public void destroyJoint(final Joint j) {
		assert(m_lock == false);

		final boolean collideConnected = j.m_collideConnected;

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
		final Body body1 = j.m_body1;
		final Body body2 = j.m_body2;

		// Wake up connected bodies.
		body1.wakeUp();
		body2.wakeUp();

		// Remove from body 1
		if (j.m_node1.prev != null) {
			j.m_node1.prev.next = j.m_node1.next;
		}

		if (j.m_node1.next != null) {
			j.m_node1.next.prev = j.m_node1.prev;
		}

		if (j.m_node1 == body1.m_jointList) {
			body1.m_jointList = j.m_node1.next;
		}

		j.m_node1.prev = null;
		j.m_node1.next = null;

		// Remove from body 2
		if (j.m_node2.prev != null) {
			j.m_node2.prev.next = j.m_node2.next;
		}

		if (j.m_node2.next != null) {
			j.m_node2.next.prev = j.m_node2.prev;
		}

		if (j.m_node2 == body2.m_jointList) {
			body2.m_jointList = j.m_node2.next;
		}

		j.m_node2.prev = null;
		j.m_node2.next = null;

		Joint.destroy(j);

		assert m_jointCount > 0;
		--m_jointCount;

		// If the joint prevents collisions, then reset collision filtering.
		if (collideConnected == false) {
			// Reset the proxies on the body with the minimum number of shapes.
			final Body b = body1.m_shapeCount < body2.m_shapeCount ? body1 : body2;
			for (Shape s = b.m_shapeList; s != null; s = s.m_next) {
				s.refilterProxy(m_broadPhase, b.getMemberXForm());
			}
		}
	}
	
	public Controller createController( final ControllerDef def) {
		Controller controller = def.create();

		controller.m_next = m_controllerList;
		controller.m_prev = null;

		if (m_controllerList != null) {
			m_controllerList.m_prev = controller;
		}
		
		m_controllerList = controller;
		++m_controllerCount;

		controller.m_world = this;

		return controller;
	}
	
	public void destroyController(Controller controller) {
		assert(m_controllerCount>0);
		
		if(controller.m_next != null)
		{
			controller.m_next.m_prev = controller.m_prev;
		}

		if(controller.m_prev != null)
		{
			controller.m_prev.m_next = controller.m_next;
		}

		if(controller == m_controllerList)
		{
			m_controllerList = controller.m_next;
		}

		--m_controllerCount;
	}

	// djm pooling
	private static final TLTimeStep tlStep = new TLTimeStep();
	/**
	 * Take a time step. This performs collision detection, integration,
	 * and constraint solution.
	 * @param dt the amount of time to simulate, this should not vary.
	 * @param iterations the number of iterations to be used by the constraint solver.
	 */
	public void step(final float dt, final int iterations) {
		m_lock = true;

		final TimeStep step = tlStep.get();
		step.dt = dt;
		step.maxIterations	= iterations;
		if (dt > 0.0f) {
			step.inv_dt = 1.0f / dt;
		} else {
			step.inv_dt = 0.0f;
		}

		step.dtRatio = m_inv_dt0 * dt;

		step.positionCorrection = m_positionCorrection;
		step.warmStarting = m_warmStarting;

		// Update contacts.
		m_contactManager.collide();

		// Integrate velocities, solve velocity constraints, and integrate positions.
		if (step.dt > 0.0f) {
			solve(step);
		}

		// Handle TOI events.
		if (m_continuousPhysics && step.dt > 0.0f) {
			solveTOI(step);
		}

		// Draw debug information.
		if(autoDebugDraw){
			drawDebugData();			
		}

		m_inv_dt0 = step.inv_dt;
		m_lock = false;
		
		postStep(dt,iterations);
	}


	/** Goes through the registered postStep functions and calls them. */
	private void postStep(final float dt, final int iterations) {
		for (final Steppable s:postStepList) {
			s.step(dt,iterations);
		}
	}

	/**
	 * Registers a Steppable object to be stepped
	 * immediately following the physics step, once
	 * the locks are lifted.
	 * @param s
	 */
	public void registerPostStep(final Steppable s) {
		postStepList.add(s);
	}

	/**
	 * Unregisters a method from post-stepping.
	 * Fails silently if method is not found.
	 * @param s
	 */
	public void unregisterPostStep(final Steppable s) {
		if (postStepList != null) {
			postStepList.remove(s);
		}
	}

	/** Re-filter a shape. This re-runs contact filtering on a shape. */
	public void refilter(final Shape shape) {
		shape.refilterProxy(m_broadPhase, shape.getBody().getMemberXForm());
	}

	/**
	 * Query the world for all shapes that potentially overlap the
	 * provided AABB up to max count.
	 * The number of shapes found is returned.
	 * @param aabb the query box.
	 * @param maxCount the capacity of the shapes array.
	 * @return array of shapes overlapped, up to maxCount in length
	 */
	public Shape[] query(final AABB aabb, final int maxCount) {
		final Object[] objs = m_broadPhase.query(aabb, maxCount);
		final Shape[] ret = new Shape[objs.length];
		System.arraycopy(objs, 0, ret, 0, objs.length);
		//for (int i=0; i<ret.length; ++i) {
		//	ret[i] = (Shape)(objs[i]);
		//}

		return ret;
	}


	//--------------- Internals Below -------------------
	// Internal yet public to make life easier.

	// Java note: sorry, guys, we have to keep this stuff public until
	// the C++ version does otherwise so that we can maintain the engine...

	// djm pooling
	private static final IslandStack islands = new IslandStack();
	
	/** For internal use */
	public void solve(final TimeStep step) {
		m_positionIterationCount = 0;
		
		// Step all controllers
		for(Controller controller = m_controllerList; controller != null; controller = controller.m_next) {
			controller.step(step);
		}

		// Size the island for the worst case.
		final Island island = islands.get();
		island.init(m_bodyCount, m_contactCount, m_jointCount, m_contactListener);

		// Clear all the island flags.
		for (Body b = m_bodyList; b != null; b = b.m_next) {
			b.m_flags &= ~Body.e_islandFlag;
		}
		for (Contact c = m_contactList; c != null; c = c.m_next) {
			c.m_flags &= ~Contact.e_islandFlag;
		}
		for (Joint j = m_jointList; j != null; j = j.m_next) {
			j.m_islandFlag = false;
		}

		// Build and simulate all awake islands.
		final int stackSize = m_bodyCount;
		final Body[] stack = new Body[stackSize];
		for (Body seed = m_bodyList; seed != null; seed = seed.m_next) {
			if ( (seed.m_flags & (Body.e_islandFlag | Body.e_sleepFlag | Body.e_frozenFlag)) > 0){
				continue;
			}

			if (seed.isStatic()) {
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
				final Body b = stack[--stackCount];
				island.add(b);

				// Make sure the body is awake.
				b.m_flags &= ~Body.e_sleepFlag;

				// To keep islands as small as possible, we don't
				// propagate islands across static bodies.
				if (b.isStatic()) {
					continue;
				}

				// Search all contacts connected to this body.
				for ( ContactEdge cn = b.m_contactList; cn != null; cn = cn.next) {
					// Has this contact already been added to an island?
					if ( (cn.contact.m_flags & (Contact.e_islandFlag | Contact.e_nonSolidFlag)) > 0) {
						continue;
					}

					// Is this contact touching?
					if (cn.contact.getManifoldCount() == 0) {
						continue;
					}

					island.add(cn.contact);
					cn.contact.m_flags |= Contact.e_islandFlag;

					// Was the other body already added to this island?
					final Body other = cn.other;
					if ((other.m_flags & Body.e_islandFlag) > 0) {
						continue;
					}

					assert stackCount < stackSize;
					stack[stackCount++] = other;
					other.m_flags |= Body.e_islandFlag;
				}

				// Search all joints connect to this body.
				for ( JointEdge jn = b.m_jointList; jn != null; jn = jn.next) {
					if (jn.joint.m_islandFlag == true) {
						continue;
					}

					island.add(jn.joint);
					jn.joint.m_islandFlag = true;

					final Body other = jn.other;
					if ((other.m_flags & Body.e_islandFlag) > 0) {
						continue;
					}

					assert (stackCount < stackSize);
					stack[stackCount++] = other;
					other.m_flags |= Body.e_islandFlag;
				}
			}

			island.solve(step, m_gravity, m_positionCorrection, m_allowSleep);

			m_positionIterationCount = MathUtils.max(m_positionIterationCount, Island.m_positionIterationCount);

			// Post solve cleanup.
			for (int i = 0; i < island.m_bodyCount; ++i) {
				// Allow static bodies to participate in other islands.
				final Body b = island.m_bodies[i];
				if (b.isStatic()) {
					b.m_flags &= ~Body.e_islandFlag;
				}
			}
		}

		//m_broadPhase.commit();

		// Synchronize shapes, check for out of range bodies.
		for (Body b = m_bodyList; b != null; b = b.getNext()) {
			if ( (b.m_flags & (Body.e_sleepFlag | Body.e_frozenFlag)) != 0) {
				continue;
			}

			if (b.isStatic()) {
				continue;
			}

			// Update shapes (for broad-phase). If the shapes go out of
			// the world AABB then shapes and contacts may be destroyed,
			// including contacts that are
			final boolean inRange = b.synchronizeShapes();

			// Did the body's shapes leave the world?
			if (inRange == false && m_boundaryListener != null) {
				m_boundaryListener.violation(b);
			}
		}

		// Commit shape proxy movements to the broad-phase so that new contacts are created.
		// Also, some contacts can be destroyed.
		m_broadPhase.commit();
		
		islands.recycle(island);
	}

	
	// djm pooling
	private static final TimeStepStack steps = new TimeStepStack();
	
	/** For internal use: find TOI contacts and solve them. */
	public void solveTOI(final TimeStep step) {
		// Reserve an island and a stack for TOI island solution.
		// djm do we always have to make a new island? or can we make
		// it static?
		
		// Size the island for the worst case.
		final Island island = islands.get();
		island.init(m_bodyCount, Settings.maxTOIContactsPerIsland, Settings.maxTOIJointsPerIsland, m_contactListener);

		//Simple one pass queue
		//Relies on the fact that we're only making one pass
		//through and each body can only be pushed/popped once.
		//To push:
		//  queue[queueStart+queueSize++] = newElement
		//To pop:
		//	poppedElement = queue[queueStart++];
		//  --queueSize;
		final int queueCapacity = m_bodyCount;
		final Body[] queue = new Body[queueCapacity];

		for (Body b = m_bodyList; b != null; b = b.m_next) {
			b.m_flags &= ~Body.e_islandFlag;
			b.m_sweep.t0 = 0.0f;
		}

		for (Contact c = m_contactList; c != null; c = c.m_next) {
			// Invalidate TOI
			c.m_flags &= ~(Contact.e_toiFlag | Contact.e_islandFlag);
		}

		for (Joint j = m_jointList; j != null; j = j.m_next) {
			j.m_islandFlag = false;
		}

		// Find TOI events and solve them.
		while (true) {
			// Find the first TOI.
			Contact minContact = null;
			float minTOI = 1.0f;

			for (Contact c = m_contactList; c != null; c = c.m_next) {
				if ((c.m_flags & (Contact.e_slowFlag | Contact.e_nonSolidFlag)) != 0) {
					continue;
				}

				// TODO_ERIN keep a counter on the contact, only respond to M TOIs per contact.
				float toi = 1.0f;
				if ((c.m_flags & Contact.e_toiFlag) != 0) {
					// This contact has a valid cached TOI.
					toi = c.m_toi;
				} else {
					// Compute the TOI for this contact.
					final Shape s1 = c.getShape1();
					final Shape s2 = c.getShape2();
					final Body b1 = s1.getBody();
					final Body b2 = s2.getBody();

					if ((b1.isStatic() || b1.isSleeping()) && (b2.isStatic() || b2.isSleeping())) {
						continue;
					}

					// Put the sweeps onto the same time interval.
					float t0 = b1.m_sweep.t0;

					if (b1.m_sweep.t0 < b2.m_sweep.t0) {
						t0 = b2.m_sweep.t0;
						b1.m_sweep.advance(t0);
					} else if (b2.m_sweep.t0 < b1.m_sweep.t0) {
						t0 = b1.m_sweep.t0;
						b2.m_sweep.advance(t0);
					}
					assert(t0 < 1.0f);

					// Compute the time of impact.
					toi = TOI.timeOfImpact(c.m_shape1, b1.m_sweep, c.m_shape2, b2.m_sweep);
					//System.out.println(toi);
					assert(0.0f <= toi && toi <= 1.0f);

					if (toi > 0.0f && toi < 1.0f) {
						toi = MathUtils.min((1.0f - toi) * t0 + toi, 1.0f);
					}

					c.m_toi = toi;
					c.m_flags |= Contact.e_toiFlag;
				}

				if (Settings.EPSILON < toi && toi < minTOI) {
					// This is the minimum TOI found so far.
					minContact = c;
					minTOI = toi;

				}


			}

			if (minContact == null || 1.0f - 100.0f * Settings.EPSILON < minTOI) {
				// No more TOI events. Done!
				break;
			}

			// Advance the bodies to the TOI.
			final Shape s1 = minContact.getShape1();
			final Shape s2 = minContact.getShape2();
			final Body b1 = s1.getBody();
			final Body b2 = s2.getBody();
			b1.advance(minTOI);
			b2.advance(minTOI);

			// The TOI contact likely has some new contact points.
			minContact.update(m_contactListener);
			minContact.m_flags &= ~Contact.e_toiFlag;

			if (minContact.getManifoldCount() == 0) {
				// This shouldn't happen. Numerical error?
				//b2Assert(false);
				continue;
			}

			// Build the TOI island. We need a dynamic seed.
			Body seed = b1;
			if (seed.isStatic()) {
				seed = b2;
			}

			// Reset island and queue.
			island.clear();
			//int stackCount = 0;
			int queueStart = 0; //starting index for queue
			int queueSize = 0;  //elements in queue
			queue[queueStart+queueSize++] = seed;
			seed.m_flags |= Body.e_islandFlag;

			// Perform a breadth first search (BFS) on the contact/joint graph.
			while (queueSize > 0) {
				// Grab the head body off the queue and add it to the island.
				final Body b = queue[queueStart++];
				--queueSize;

				island.add(b);

				// Make sure the body is awake.
				b.m_flags &= ~Body.e_sleepFlag;

				// To keep islands as small as possible, we don't
				// propagate islands across static bodies.
				if (b.isStatic()) {
					continue;
				}

				// Search all contacts connected to this body.
				for (ContactEdge cn = b.m_contactList; cn != null; cn = cn.next) {
					// Does the TOI island still have space for contacts?
					if (island.m_contactCount == island.m_contactCapacity) {
						continue;
					}

					// Has this contact already been added to an island? Skip slow or non-solid contacts.
					if ( (cn.contact.m_flags & (Contact.e_islandFlag | Contact.e_slowFlag | Contact.e_nonSolidFlag)) != 0) {
						continue;
					}

					// Is this contact touching? For performance we are not updating this contact.
					if (cn.contact.getManifoldCount() == 0) {
						continue;
					}

					island.add(cn.contact);
					cn.contact.m_flags |= Contact.e_islandFlag;
					// Update other body.
					final Body other = cn.other;

					// Was the other body already added to this island?
					if ((other.m_flags & Body.e_islandFlag) != 0) {
						continue;
					}

					// March forward, this can do no harm since this is the min TOI.
					if (other.isStatic() == false) {
						other.advance(minTOI);
						other.wakeUp();
					}

					//push to the queue
					assert(queueSize < queueCapacity);
					queue[queueStart+queueSize++] = other;
					other.m_flags |= Body.e_islandFlag;

				}

				// Search all joints connect to this body.
				for ( JointEdge jn = b.m_jointList; jn != null; jn = jn.next) {
					if (island.m_jointCount == island.m_jointCapacity) {
						continue;
					}

					if (jn.joint.m_islandFlag == true) {
						continue;
					}

					island.add(jn.joint);

					jn.joint.m_islandFlag = true;

					final Body other = jn.other;
					if ((other.m_flags & Body.e_islandFlag) > 0) {
						continue;
					}

					if (other.isStatic() == false) {
						//System.out.println(minTOI);
						other.advance(minTOI);
						other.wakeUp();
					}

					assert (queueSize < queueCapacity);
					queue[queueStart+queueSize++] = other;
					other.m_flags |= Body.e_islandFlag;
				}

			}

			final TimeStep subStep = steps.get();
			subStep.warmStarting = false;
			subStep.dt = (1.0f - minTOI) * step.dt;
			assert(subStep.dt > Settings.EPSILON);
			subStep.inv_dt = 1.0f / subStep.dt;
			subStep.maxIterations = step.maxIterations;

			island.solveTOI(subStep);
			steps.recycle(subStep);
			
			// Post solve cleanup.
			for (int i = 0; i < island.m_bodyCount; ++i) {
				// Allow bodies to participate in future TOI islands.
				final Body b = island.m_bodies[i];
				b.m_flags &= ~Body.e_islandFlag;

				if ( (b.m_flags & (Body.e_sleepFlag | Body.e_frozenFlag)) != 0) {
					continue;
				}

				if (b.isStatic()) {
					continue;
				}

				// Update shapes (for broad-phase). If the shapes go out of
				// the world AABB then shapes and contacts may be destroyed,
				// including contacts that are
				final boolean inRange = b.synchronizeShapes();

				// Did the body's shapes leave the world?
				if (inRange == false && m_boundaryListener != null) {
					m_boundaryListener.violation(b);
				}

				// Invalidate all contact TOIs associated with this body. Some of these
				// may not be in the island because they were not touching.
				for (ContactEdge cn = b.m_contactList; cn != null; cn = cn.next) {
					cn.contact.m_flags &= ~Contact.e_toiFlag;
				}

			}

			for (int i = 0; i < island.m_contactCount; ++i) {
				// Allow contacts to participate in future TOI islands.

				final Contact c = island.m_contacts[i];
				c.m_flags &= ~(Contact.e_toiFlag | Contact.e_islandFlag);
			}

			for (int i=0; i < island.m_jointCount; ++i) {
				final Joint j = island.m_joints[i];
				j.m_islandFlag = false;
			}

			// Commit shape proxy movements to the broad-phase so that new contacts are created.
			// Also, some contacts can be destroyed.
			m_broadPhase.commit();
		}
		islands.recycle(island);
	}
	
	// NOTE this corresponds to the liquid test, so the debugdraw can draw
	// the liquid particles correctly.  They should be the same.
	private static Integer LIQUID_INT = new Integer(12345);
	private float liquidLength = .12f;
	private float averageLinearVel = -1;
	// djm pooled
	private final Color3f coreColor = new Color3f(255f*0.9f, 255f*0.6f, 255f*0.6f);
	private final Vec2 drawingCenter = new Vec2();
	private final Vec2 liquidOffset = new Vec2();
	private final Vec2 circCenterMoved = new Vec2();
	private final Color3f liquidColor = new Color3f(80.0f,80.0f,255f);
	private final Vec2 segLeft = new Vec2();
	private final Vec2 segRight = new Vec2();
	/** For internal use */
	public void drawShape(final Shape shape, final XForm xf, final Color3f color, final boolean core) {

		if (shape.getType() == ShapeType.CIRCLE_SHAPE) {
			final CircleShape circle = (CircleShape)shape;

			XForm.mulToOut(xf, circle.getMemberLocalPosition(), drawingCenter);
			final float radius = circle.getRadius();
			final Vec2 axis = xf.R.col1;
			
			if (circle.getUserData() != null && circle.getUserData().equals(LIQUID_INT)) {
				Body b = circle.getBody();
				liquidOffset.set(b.m_linearVelocity);
				float linVelLength = b.m_linearVelocity.length();
				if(averageLinearVel == -1){
					averageLinearVel = linVelLength;
				}else{
					averageLinearVel = .98f * averageLinearVel + .02f * linVelLength;
				}
				liquidOffset.mulLocal( liquidLength/averageLinearVel/2);
				circCenterMoved.set(drawingCenter).addLocal( liquidOffset);
				drawingCenter.subLocal(liquidOffset);
				m_debugDraw.drawSegment(drawingCenter, circCenterMoved, liquidColor);
				return;
			}

			m_debugDraw.drawSolidCircle(drawingCenter, radius, axis, color);

			if (core) {
				m_debugDraw.drawCircle(drawingCenter, radius - Settings.toiSlop, coreColor);
			}
		} else if (shape.getType() == ShapeType.POINT_SHAPE) {
			final PointShape point = (PointShape)shape;

			XForm.mulToOut(xf, point.getMemberLocalPosition(), drawingCenter);

			//m_debugDraw.drawSolidCircle(center, radius, axis, color);
			m_debugDraw.drawPoint(drawingCenter, 0.0f, color);

		} else if (shape.getType() == ShapeType.POLYGON_SHAPE) {
			final PolygonShape poly = (PolygonShape)shape;
			final int vertexCount = poly.getVertexCount();
			final Vec2[] localVertices = poly.getVertices();

			assert(vertexCount <= Settings.maxPolygonVertices);
			final Vec2[] vertices = new Vec2[vertexCount];

			for (int i = 0; i < vertexCount; ++i) {
				// djm these aren't instantiated so we need to be creating
				// these.  To get rid of these instantiations, we would need
				// to change the DebugDraw so you give it local vertices and the
				// XForm to transform them with
				vertices[i] = XForm.mul(xf, localVertices[i]);
			}

			m_debugDraw.drawSolidPolygon(vertices, vertexCount, color);

			if (core) {
				final Vec2[] localCoreVertices = poly.getCoreVertices();
				for (int i = 0; i < vertexCount; ++i) {
					// djm same as above
					vertices[i] = XForm.mul(xf, localCoreVertices[i]);
				}
				m_debugDraw.drawPolygon(vertices, vertexCount, coreColor);
			}

		} else if (shape.getType() == ShapeType.EDGE_SHAPE) {
			final EdgeShape edge = (EdgeShape) shape;
			XForm.mulToOut( xf, edge.getVertex1(), segLeft);
			XForm.mulToOut( xf, edge.getVertex2(), segRight);
			m_debugDraw.drawSegment(segLeft, segRight, color);

			if (core) {
				XForm.mulToOut( xf, edge.getCoreVertex1(), segLeft);
				XForm.mulToOut( xf, edge.getCoreVertex2(), segRight);
				m_debugDraw.drawSegment(segLeft, segRight, coreColor);
			}

		}
	}

	// djm pooled
	private final Color3f jointColor = new Color3f(255f*0.5f, 255f*0.8f, 255f*0.8f);
	/** For internal use */
	public void drawJoint(final Joint joint) {
		final Body b1 = joint.getBody1();
		final Body b2 = joint.getBody2();
		final XForm xf1 = b1.getMemberXForm();
		final XForm xf2 = b2.getMemberXForm();
		final Vec2 x1 = xf1.position;
		final Vec2 x2 = xf2.position;
		final Vec2 p1 = joint.getAnchor1();
		final Vec2 p2 = joint.getAnchor2();

		final JointType type = joint.getType();

		if (type == JointType.DISTANCE_JOINT) {
			m_debugDraw.drawSegment(p1, p2, jointColor);
		} else if (type == JointType.PULLEY_JOINT) {
			final PulleyJoint pulley = (PulleyJoint)joint;
			final Vec2 s1 = pulley.getGroundAnchor1();
			final Vec2 s2 = pulley.getGroundAnchor2();
			m_debugDraw.drawSegment(s1, p1, jointColor);
			m_debugDraw.drawSegment(s2, p2, jointColor);
			m_debugDraw.drawSegment(s1, s2, jointColor);
		} else if (type == JointType.MOUSE_JOINT) {
			//Don't draw mouse joint
		} else if (type == JointType.CONSTANT_VOLUME_JOINT) {
			final ConstantVolumeJoint cvj = (ConstantVolumeJoint)joint;
			final Body[] bodies = cvj.getBodies();
			for (int i=0; i<bodies.length; ++i) {
				final int next = (i==bodies.length-1)?0:i+1;
				// TODO decide how to handle these calls
				final Vec2 first = bodies[i].getMemberWorldCenter();
				final Vec2 nextV = bodies[next].getMemberWorldCenter();
				m_debugDraw.drawSegment(first, nextV, jointColor);
			}

		} else {
			m_debugDraw.drawSegment(x1, p1, jointColor);
			m_debugDraw.drawSegment(p1, p2, jointColor);
			m_debugDraw.drawSegment(x2, p2, jointColor);
		}
	}

	// djm pooled
	private final Color3f staticColor = new Color3f(255f*0.5f, 255f*0.9f, 255f*0.5f);
	private final Color3f sleepingColor = new Color3f(255f*0.5f, 255f*0.5f, 255f*0.9f);
	private final Color3f activeColor = new Color3f(255f*0.9f, 255f*0.9f, 255f*0.9f);
	private final Color3f pairColor = new Color3f(255f*0.9f, 255f*0.9f, 255f*0.3f);
	private final Color3f aabbColor = new Color3f(255f*0.9f, 255f*0.3f,255f* 0.9f);
	private final Color3f obbColor =  new Color3f(0.5f, 0.3f, 0.5f);
	private final Color3f worldColor = new Color3f(255.0f*0.3f, 255.0f*0.9f, 255.0f*0.9f);
	private final AABB pairB1 = new AABB();
	private final AABB pairB2 = new AABB();
	private final Vec2 pairX1 = new Vec2();
	private final Vec2 pairX2 = new Vec2();
	private final AABB aabbB = new AABB();
	private final Vec2[] cornerVecs = {
			new Vec2(),
			new Vec2(),
			new Vec2(),
			new Vec2()
	};
	/** For internal use */
	public void drawDebugData() {
		if (m_debugDraw == null || m_drawDebugData == false) {
			return;
		}

		final int flags = m_debugDraw.getFlags();

		if ( (flags & DebugDraw.e_shapeBit) != 0) {

			final boolean core = (flags & DebugDraw.e_coreShapeBit) == DebugDraw.e_coreShapeBit;

			for (Body b = m_bodyList; b != null; b = b.getNext()) {
				final XForm xf = b.getMemberXForm();

				for (Shape s = b.getShapeList(); s != null; s = s.getNext()) {
					//if (s.isSensor()) continue;

					if (b.isStatic()) {
						drawShape(s, xf, staticColor, core);
					}
					else if (b.isSleeping()) {
						drawShape(s, xf, sleepingColor, core);
					} else {
						drawShape(s, xf, activeColor, core);
					}
				}
			}
		}

		if ( (flags & DebugDraw.e_jointBit) != 0) {
			for (Joint j = m_jointList; j != null; j = j.getNext()) {
				if (j.getType() != JointType.MOUSE_JOINT) {
					drawJoint(j);
				}
			}
		}

		if ( (flags & DebugDraw.e_pairBit) != 0) {
			final BroadPhase bp = m_broadPhase;
			// djm eh just keep this
			final Vec2 invQ = new Vec2(0.0f, 0.0f);
			invQ.set(1.0f / bp.m_quantizationFactor.x, 1.0f / bp.m_quantizationFactor.y);

			for (int i = 0; i < PairManager.TABLE_CAPACITY; ++i) {
				int index = bp.m_pairManager.m_hashTable[i];
				while (index != PairManager.NULL_PAIR) {
					final Pair pair = bp.m_pairManager.m_pairs[index];
					final Proxy p1 = bp.m_proxyPool[pair.proxyId1];
					final Proxy p2 = bp.m_proxyPool[pair.proxyId2];


					pairB1.lowerBound.x = bp.m_worldAABB.lowerBound.x + invQ.x * bp.m_bounds[0][p1.lowerBounds[0]].value;
					pairB1.lowerBound.y = bp.m_worldAABB.lowerBound.y + invQ.y * bp.m_bounds[1][p1.lowerBounds[1]].value;
					pairB1.upperBound.x = bp.m_worldAABB.lowerBound.x + invQ.x * bp.m_bounds[0][p1.upperBounds[0]].value;
					pairB1.upperBound.y = bp.m_worldAABB.lowerBound.y + invQ.y * bp.m_bounds[1][p1.upperBounds[1]].value;
					pairB2.lowerBound.x = bp.m_worldAABB.lowerBound.x + invQ.x * bp.m_bounds[0][p2.lowerBounds[0]].value;
					pairB2.lowerBound.y = bp.m_worldAABB.lowerBound.y + invQ.y * bp.m_bounds[1][p2.lowerBounds[1]].value;
					pairB2.upperBound.x = bp.m_worldAABB.lowerBound.x + invQ.x * bp.m_bounds[0][p2.upperBounds[0]].value;
					pairB2.upperBound.y = bp.m_worldAABB.lowerBound.y + invQ.y * bp.m_bounds[1][p2.upperBounds[1]].value;

					pairX1.x = 0.5f * (pairB1.lowerBound.x + pairB1.upperBound.x);
					pairX1.y = 0.5f * (pairB1.lowerBound.y + pairB1.upperBound.y);
					pairX2.x = 0.5f * (pairB2.lowerBound.x + pairB2.upperBound.x);
					pairX2.y = 0.5f * (pairB2.lowerBound.y + pairB2.upperBound.y);

					m_debugDraw.drawSegment(pairX1, pairX1, pairColor);

					index = pair.next;
				}
			}
		}

		if ( (flags & DebugDraw.e_controllerBit) != 0)
		{
			for (Controller c = m_controllerList; c!=null; c= c.getNext())
			{
				c.draw(m_debugDraw);
			}
		}

		final BroadPhase bp = m_broadPhase;
		final Vec2 worldLower = bp.m_worldAABB.lowerBound;
		final Vec2 worldUpper = bp.m_worldAABB.upperBound;

		if ( (flags & DebugDraw.e_aabbBit) != 0) {

			final Vec2 invQ = new Vec2();
			invQ.set(1.0f / bp.m_quantizationFactor.x, 1.0f / bp.m_quantizationFactor.y);

			for (int i = 0; i < Settings.maxProxies; ++i) {
				final Proxy p = bp.m_proxyPool[i];
				if (p.isValid() == false) {
					continue;
				}


				aabbB.lowerBound.x = worldLower.x + invQ.x * bp.m_bounds[0][p.lowerBounds[0]].value;
				aabbB.lowerBound.y = worldLower.y + invQ.y * bp.m_bounds[1][p.lowerBounds[1]].value;
				aabbB.upperBound.x = worldLower.x + invQ.x * bp.m_bounds[0][p.upperBounds[0]].value;
				aabbB.upperBound.y = worldLower.y + invQ.y * bp.m_bounds[1][p.upperBounds[1]].value;

				cornerVecs[0].set(aabbB.lowerBound.x, aabbB.lowerBound.y);
				cornerVecs[1].set(aabbB.upperBound.x, aabbB.lowerBound.y);
				cornerVecs[2].set(aabbB.upperBound.x, aabbB.upperBound.y);
				cornerVecs[3].set(aabbB.lowerBound.x, aabbB.upperBound.y);

				m_debugDraw.drawPolygon(cornerVecs, 4, aabbColor);
			}
		}

		cornerVecs[0].set(worldLower.x, worldLower.y);
		cornerVecs[1].set(worldUpper.x, worldLower.y);
		cornerVecs[2].set(worldUpper.x, worldUpper.y);
		cornerVecs[3].set(worldLower.x, worldUpper.y);
		m_debugDraw.drawPolygon(cornerVecs, 4, worldColor);

		if ( (flags & DebugDraw.e_obbBit) != 0) {

			for (Body b = m_bodyList; b != null; b = b.getNext()) {
				// TODO figure out a better way to handle this
				final XForm xf = b.getMemberXForm();

				for (Shape s = b.getShapeList(); s != null; s = s.getNext()) {
					if (s.getType() != ShapeType.POLYGON_SHAPE) {
						continue;
					}

					final PolygonShape poly = (PolygonShape)s;
					final OBB obb = poly.getOBB();
					final Vec2 h = obb.extents;

					cornerVecs[0].set(-h.x, -h.y);
					cornerVecs[1].set( h.x, -h.y);
					cornerVecs[2].set( h.x,  h.y);
					cornerVecs[3].set(-h.x,  h.y);

					for (int i = 0; i < cornerVecs.length; ++i) {
						Mat22.mulToOut(obb.R, cornerVecs[i], cornerVecs[i]);
						XForm.mulToOut( xf, cornerVecs[i], cornerVecs[i]);
						//vs[i] = obb.center.add(Mat22.mul(obb.R, vs[i]));
						//vs[i] = XForm.mul(xf, vs[i]);
					}

					m_debugDraw.drawPolygon(cornerVecs, 4, obbColor);
				}
			}
		}

		if ( (flags & DebugDraw.e_centerOfMassBit) != 0) {
			for (Body b = m_bodyList; b != null; b = b.getNext()) {
				// TODO handle this differently
				final XForm xf = b.getMemberXForm();
				xf.position = b.getMemberWorldCenter();
				m_debugDraw.drawXForm(xf);
			}
		}
	}

	/** Enable/disable warm starting. For testing. */
	public void setWarmStarting(final boolean flag) { m_warmStarting = flag; }

	/** Enable/disable position correction. For testing. */
	public void setPositionCorrection(final boolean flag) { m_positionCorrection = flag; }

	/** Enable/disable continuous physics. For testing. */
	public void setContinuousPhysics(final boolean flag) { m_continuousPhysics = flag; }

	/** Perform validation of internal data structures. */
	public void validate() {
		m_broadPhase.validate();
	}

	/** Get the number of broad-phase proxies. */
	public int getProxyCount() {
		return m_broadPhase.m_proxyCount;
	}

	/** Get the number of broad-phase pairs. */
	public int getPairCount() {
		return m_broadPhase.m_pairManager.m_pairCount;
	}

	/** Get the world bounding box. */
	public AABB getWorldAABB() {
		return m_broadPhase.m_worldAABB;
	}

	/** Return true if the bounding box is within range of the world AABB. */
	public boolean inRange(final AABB aabb) {
		return m_broadPhase.inRange(aabb);
	}
	
	Segment m_raycastSegment;
	Vec2 m_raycastNormal;
	Object m_raycastUserData;
	boolean m_raycastSolidShape;
	
	/** 
	 * Query the world for all fixtures that intersect a given segment. You provide a shape
	 * pointer buffer of specified size. The number of shapes found is returned, and the buffer
	 * is filled in order of intersection
	 * @param segment defines the begin and end point of the ray cast, from p1 to p2.
	 * @param shapes a user allocated shape pointer array of size maxCount (or greater).
	 * @param maxCount the capacity of the shapes array
	 * @param solidShapes determines if shapes that the ray starts in are counted as hits.
	 * @param userData passed through the worlds contact filter, with method RayCollide. This can be used to filter valid shapes
	 * @return the number of shapes found
	 */
	public int raycast(Segment segment, Shape[] shapes, int maxCount, boolean solidShapes, Object userData)
	{
		m_raycastSegment = segment;
		m_raycastUserData = userData;
		m_raycastSolidShape = solidShapes;

		Object[] results = new Object[maxCount];

		int count = m_broadPhase.querySegment(segment,results,maxCount, raycastSortKey);

		for (int i = 0; i < count; ++i)
		{
			shapes[i] = (Shape)results[i];
		}

		return count;
	}

	/** 
	 * Performs a ray-cast as with {@link #raycast(Segment, Shape[], int, boolean, Object)}, finding the first intersecting shape
	 * @param segment defines the begin and end point of the ray cast, from p1 to p2
	 * @param lambda returns the hit fraction. You can use this to compute the contact point
	 * p = (1 - lambda) * segment.p1 + lambda * segment.p2.
	 * @param normal returns the normal at the contact point. If there is no intersection, the normal
	 * is not set.
	 * @param solidShapes determines if shapes that the ray starts in are counted as hits.
	 * @returns the colliding shape shape, or null if not found
	 * @see #raycast(Segment, Shape[], int, boolean, Object)
	 */
	public Shape raycastOne(Segment segment, RaycastResult result, boolean solidShapes, Object userData)
	{
		int maxCount = 1;
		Shape[] shapes = new Shape[maxCount];

		int count = raycast(segment, shapes, maxCount, solidShapes, userData);

		if(count==0)
			return null;

		assert(count==1);

		//Redundantly do TestSegment a second time, as the previous one's results are inaccessible
//		System.out.println("Before final test, testing shape  " + shapes[0].getType());
//		System.out.println(Arrays.toString(shapes));
		shapes[0].testSegment(shapes[0].getBody().getMemberXForm(),result,segment,1.0f);
//		System.out.println("Got here, lambda = " + result.lambda);
		//We already know it returns true
		return shapes[0];
	}
	
	private SortKeyFunc raycastSortKey = new SortKeyFunc() {
		public float apply(Object shape) {
			return raycastSortKeyFunc(shape);
		}
	};
	
	private float raycastSortKeyFunc(Object data)
	{
		Shape shape = (Shape)data;
		Body body = shape.getBody();
		World world = body.getWorld();

		if (world.m_contactFilter!=null && !world.m_contactFilter.rayCollide(world.m_raycastUserData,shape))
		{
			return -1;
		}

		RaycastResult result = new RaycastResult();
		SegmentCollide collide = shape.testSegment(body.getMemberXForm(),result, world.m_raycastSegment, 1.0f);
				//&lambda, &world->m_raycastNormal, *world->m_raycastSegment, 1);
		float lambda = result.lambda;
		
		if (world.m_raycastSolidShape && collide == SegmentCollide.MISS_COLLIDE)
		{
			return -1;
		}

		if (!world.m_raycastSolidShape && collide != SegmentCollide.HIT_COLLIDE)
		{
			return -1;
		}

		return lambda;
	}
}
