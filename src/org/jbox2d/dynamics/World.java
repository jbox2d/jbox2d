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

import org.jbox2d.common.Color3f;

import org.jbox2d.collision.AABB;
import org.jbox2d.collision.BroadPhase;
import org.jbox2d.collision.CircleShape;
import org.jbox2d.collision.OBB;
import org.jbox2d.collision.Pair;
import org.jbox2d.collision.PairManager;
import org.jbox2d.collision.PolygonShape;
import org.jbox2d.collision.Proxy;
import org.jbox2d.collision.Shape;
import org.jbox2d.collision.ShapeType;
import org.jbox2d.collision.TOI;
import org.jbox2d.common.*;
import org.jbox2d.dynamics.contacts.Contact;
import org.jbox2d.dynamics.contacts.ContactEdge;
import org.jbox2d.dynamics.joints.*;



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
	
	private float m_inv_dt0;

	private ArrayList<Steppable> postStepList;

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
	public void setGravity(Vec2 gravity) {
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
	public World(AABB worldAABB, Vec2 gravity, boolean doSleep) {
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

        m_bodyCount = 0;
        m_contactCount = 0;
        m_jointCount = 0;
        
        m_lock = false;

        m_allowSleep = doSleep;

        m_gravity = gravity;

        m_contactManager = new ContactManager();
        m_contactManager.m_world = this;
        m_broadPhase = new BroadPhase(worldAABB, m_contactManager);

        BodyDef bd = new BodyDef();
        m_groundBody = createBody(bd);
        postStepList = new ArrayList<Steppable>();
    }

	/** Register a destruction listener. */
    public void setDestructionListener(DestructionListener listener) {
        m_destructionListener = listener;
    }
    
    /** Register a broad-phase boundary listener. */
    public void setBoundaryListener(BoundaryListener listener) {
    	m_boundaryListener = listener;
    }

    /** Register a contact event listener */
    public void setContactListener(ContactListener listener) {
    	m_contactListener = listener;
    }
    

    /**
     *  Register a contact filter to provide specific control over collision.
	 *  Otherwise the default filter is used (b2_defaultFilter).
     */
    public void setContactFilter(ContactFilter filter) {
    	m_contactFilter = filter;
    }
    
    /**
	 * Register a routine for debug drawing. The debug draw functions are called
	 * inside the World.step() method, so make sure your renderer is ready to
	 * consume draw commands when you call step().
	 */
    public void setDebugDraw(DebugDraw debugDraw) {
    	m_debugDraw = debugDraw;
    }
    
	
	/** 
	 * Create a body given a definition. No reference to the definition
	 * is retained.  Body will be static unless mass is nonzero.
	 * <BR><em>Warning</em>: This function is locked during callbacks.
	 */
	public Body createBody(BodyDef def) {
		assert(m_lock == false);
		if (m_lock == true) {
			return null;
		}

		Body b = new Body(def, this);

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
	public void destroyBody(Body b) {
		assert(m_bodyCount > 0);
		assert(m_lock == false);
		if (m_lock == true) {
			return;
		}

		// Delete the attached joints.
		JointEdge jn = b.m_jointList;
		while (jn != null) {
			JointEdge jn0 = jn;
			jn = jn.next;

			if (m_destructionListener != null){
				m_destructionListener.sayGoodbye(jn0.joint);
			}

			destroyJoint(jn0.joint);
		}

		// Delete the attached shapes. This destroys broad-phase
		// proxies and pairs, leading to the destruction of contacts.
		Shape s = b.m_shapeList;
		while (s != null) {
			Shape s0 = s;
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
    public Joint createJoint(JointDef def) {
    	assert(m_lock == false);
    	
        Joint j = Joint.create(def);

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
            Body b = def.body1.m_shapeCount < def.body2.m_shapeCount ? def.body1
                    : def.body2;
            for (Shape s = b.m_shapeList; s != null; s = s.m_next) {
                s.refilterProxy(m_broadPhase, b.getXForm());
            }
        }

        return j;
    }
    
	/**
	 * Destroy a joint. This may cause the connected bodies to begin colliding.
	 * <BR><em>Warning</em>: This function is locked during callbacks.
	 */
	public void destroyJoint(Joint j) {
    	assert(m_lock == false);
    	
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
        Body body1 = j.m_body1;
        Body body2 = j.m_body2;

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
            Body b = body1.m_shapeCount < body2.m_shapeCount ? body1 : body2;
            for (Shape s = b.m_shapeList; s != null; s = s.m_next) {
                s.refilterProxy(m_broadPhase, b.getXForm());
            }
        }
    }
	
	/**
	 * Take a time step. This performs collision detection, integration,
	 * and constraint solution.
	 * @param dt the amount of time to simulate, this should not vary.
	 * @param iterations the number of iterations to be used by the constraint solver.
	 */
    public void step(float dt, int iterations) {
    	m_lock = true;

    	TimeStep step = new TimeStep();
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
    	drawDebugData();

    	m_inv_dt0 = step.inv_dt;
    	m_lock = false;
    	
    	postStep(dt,iterations);
    }
    
    
    /** Goes through the registered postStep functions and calls them. */
    private void postStep(float dt, int iterations) {
    	for (Steppable s:postStepList) {
    		s.step(dt,iterations);
    	}
    }
    
    /**
     * Registers a Steppable object to be stepped
     * immediately following the physics step, once
     * the locks are lifted.
     * @param s
     */
    public void registerPostStep(Steppable s) {
    	postStepList.add(s);
    }
    
    /**
     * Unregisters a method from post-stepping.
     * Fails silently if method is not found.
     * @param s
     */
    public void unregisterPostStep(Steppable s) {
    	postStepList.remove(s);
    }
    
    /** Re-filter a shape. This re-runs contact filtering on a shape. */
    public void refilter(Shape shape) {
    	shape.refilterProxy(m_broadPhase, shape.getBody().getXForm());
    }
	
	/**
	 * Query the world for all shapes that potentially overlap the
	 * provided AABB up to max count.
	 * The number of shapes found is returned.
	 * @param aabb the query box.
	 * @param maxCount the capacity of the shapes array.
	 * @return array of shapes overlapped, up to maxCount in length
	 */
    public Shape[] query(AABB aabb, int maxCount) {
        Object[] objs = m_broadPhase.query(aabb, maxCount);
        Shape[] ret = new Shape[objs.length];
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
    
    /** For internal use */
    public void solve(TimeStep step) {
    	m_positionIterationCount = 0;
    	
        // Size the island for the worst case.
    	Island island = new Island(m_bodyCount, m_contactCount, m_jointCount, m_contactListener);

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
        int stackSize = m_bodyCount;
        Body[] stack = new Body[stackSize];
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
                Body b = stack[--stackCount];
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
    				Body other = cn.other;
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

                    Body other = jn.other;
                    if ((other.m_flags & Body.e_islandFlag) > 0) {
                        continue;
                    }

                    assert (stackCount < stackSize);
                    stack[stackCount++] = other;
                    other.m_flags |= Body.e_islandFlag;
                }
            }

            island.solve(step, m_gravity, m_positionCorrection, m_allowSleep);
    		
            m_positionIterationCount = Math.max(m_positionIterationCount, Island.m_positionIterationCount); 

            // Post solve cleanup.
    		for (int i = 0; i < island.m_bodyCount; ++i) {
    			// Allow static bodies to participate in other islands.
    			Body b = island.m_bodies[i];
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
    		boolean inRange = b.synchronizeShapes();

    		// Did the body's shapes leave the world?
    		if (inRange == false && m_boundaryListener != null) {
    			m_boundaryListener.violation(b);
    		}
    	}

    	// Commit shape proxy movements to the broad-phase so that new contacts are created.
    	// Also, some contacts can be destroyed.
    	m_broadPhase.commit();
        
    }
    
    
    /** For internal use: find TOI contacts and solve them. */
    public void solveTOI(TimeStep step) {
    	// Reserve an island and a stack for TOI island solution.
    	Island island = new Island(m_bodyCount, Settings.maxTOIContactsPerIsland, 0, m_contactListener);
    	int stackSize = m_bodyCount;
    	Body[] stack = new Body[stackSize];

    	for (Body b = m_bodyList; b != null; b = b.m_next) {
    		b.m_flags &= ~Body.e_islandFlag;
    		b.m_sweep.t0 = 0.0f;
    	}

    	for (Contact c = m_contactList; c != null; c = c.m_next) {
    		// Invalidate TOI
    		c.m_flags &= ~(Contact.e_toiFlag | Contact.e_islandFlag);
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
    				Shape s1 = c.getShape1();
    				Shape s2 = c.getShape2();
    				Body b1 = s1.getBody();
    				Body b2 = s2.getBody();

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
    				assert(0.0f <= toi && toi <= 1.0f);
    				
    				if (toi > 0.0f && toi < 1.0f) {
    					toi = Math.min((1.0f - toi) * t0 + toi, 1.0f);
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
    		Shape s1 = minContact.getShape1();
    		Shape s2 = minContact.getShape2();
    		Body b1 = s1.getBody();
    		Body b2 = s2.getBody();
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

    		// Reset island and stack.
    		island.clear();
    		int stackCount = 0;
    		stack[stackCount++] = seed;
    		seed.m_flags |= Body.e_islandFlag;

    		// Perform a depth first search (DFS) on the contact graph.
    		while (stackCount > 0) {
    			// Grab the next body off the stack and add it to the island.
    			Body b = stack[--stackCount];
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
    				Body other = cn.other;

    				// Was the other body already added to this island?
    				if ((other.m_flags & Body.e_islandFlag) != 0) {
    					continue;
    				}

    				// March forward, this can do no harm since this is the min TOI.
    				if (other.isStatic() == false) {
    					other.advance(minTOI);
    					other.wakeUp();
    				}

    				assert(stackCount < stackSize);
    				stack[stackCount++] = other;
    				other.m_flags |= Body.e_islandFlag;

    			}
    		}

    		TimeStep subStep = new TimeStep();
    		subStep.dt = (1.0f - minTOI) * step.dt;
    		assert(subStep.dt > Settings.EPSILON);
    		subStep.inv_dt = 1.0f / subStep.dt;
    		subStep.maxIterations = step.maxIterations;

    		island.solveTOI(subStep);
    		
    		// Post solve cleanup.
    		for (int i = 0; i < island.m_bodyCount; ++i) {
    			// Allow bodies to participate in future TOI islands.
    			Body b = island.m_bodies[i];
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
    			boolean inRange = b.synchronizeShapes();

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

    			Contact c = island.m_contacts[i];
    			c.m_flags &= ~(Contact.e_toiFlag | Contact.e_islandFlag);
    		}

    		// Commit shape proxy movements to the broad-phase so that new contacts are created.
    		// Also, some contacts can be destroyed.
    		m_broadPhase.commit();
    	}

    }

    /** For internal use */
    public void drawShape(Shape shape, XForm xf, Color3f color, boolean core) {
    	Color3f coreColor = new Color3f(255f*0.9f, 255f*0.6f, 255f*0.6f);
    	
    	if (shape.getType() == ShapeType.CIRCLE_SHAPE) {
    			CircleShape circle = (CircleShape)shape;

    			Vec2 center = XForm.mul(xf, circle.getLocalPosition());
    			float radius = circle.getRadius();
    			Vec2 axis = xf.R.col1;

    			m_debugDraw.drawSolidCircle(center, radius, axis, color);

    			if (core) {
    				m_debugDraw.drawCircle(center, radius - Settings.toiSlop, coreColor);
    			}
    	} else if (shape.getType() == ShapeType.POLYGON_SHAPE) {
    			PolygonShape poly = (PolygonShape)shape;
    			int vertexCount = poly.getVertexCount();
    			Vec2[] localVertices = poly.getVertices();
    			
    			assert(vertexCount <= Settings.maxPolygonVertices);
    			Vec2[] vertices = new Vec2[vertexCount];

    			for (int i = 0; i < vertexCount; ++i) {
    				vertices[i] = XForm.mul(xf, localVertices[i]);
    			}

    			m_debugDraw.drawSolidPolygon(vertices, vertexCount, color);

    			if (core) {
    				Vec2[] localCoreVertices = poly.getCoreVertices();
    				for (int i = 0; i < vertexCount; ++i) {
    					vertices[i] = XForm.mul(xf, localCoreVertices[i]);
    				}
    				m_debugDraw.drawPolygon(vertices, vertexCount, coreColor);
    			}
    		
    	}
    }
    
    /** For internal use */
    public void drawJoint(Joint joint) {
    	Body b1 = joint.getBody1();
    	Body b2 = joint.getBody2();
    	XForm xf1 = b1.getXForm();
    	XForm xf2 = b2.getXForm();
    	Vec2 x1 = xf1.position;
    	Vec2 x2 = xf2.position;
    	Vec2 p1 = joint.getAnchor1();
    	Vec2 p2 = joint.getAnchor2();

    	Color3f color = new Color3f(255f*0.5f, 255f*0.8f, 255f*0.8f);

    	JointType type = joint.getType();
    	
    	if (type == JointType.DISTANCE_JOINT) {
    		m_debugDraw.drawSegment(p1, p2, color);
    	} else if (type == JointType.PULLEY_JOINT) {
    		PulleyJoint pulley = (PulleyJoint)joint;
    		Vec2 s1 = pulley.getGroundAnchor1();
    		Vec2 s2 = pulley.getGroundAnchor2();
    		m_debugDraw.drawSegment(s1, p1, color);
    		m_debugDraw.drawSegment(s2, p2, color);
    		m_debugDraw.drawSegment(s1, s2, color);
    	} else if (type == JointType.MOUSE_JOINT) {
    		//Don't draw mouse joint
    	} else {
    		m_debugDraw.drawSegment(x1, p1, color);
    		m_debugDraw.drawSegment(p1, p2, color);
    		m_debugDraw.drawSegment(x2, p2, color);
    	}
    }

    /** For internal use */
    public void drawDebugData() {
    	if (m_debugDraw == null) {
    		return;
    	}

    	int flags = m_debugDraw.getFlags();

    	if ( (flags & DebugDraw.e_shapeBit) != 0) {
    		
    		boolean core = (flags & DebugDraw.e_coreShapeBit) == DebugDraw.e_coreShapeBit;

    		for (Body b = m_bodyList; b != null; b = b.getNext()) {
    			XForm xf = b.getXForm();
    			for (Shape s = b.getShapeList(); s != null; s = s.getNext()) {
    				if (s.isSensor()) continue;
    				if (b.isStatic()) {
    					drawShape(s, xf, new Color3f(255f*0.5f, 255f*0.9f, 255f*0.5f), core);
    				}
    				else if (b.isSleeping()) {
    					drawShape(s, xf, new Color3f(255f*0.5f, 255f*0.5f, 255f*0.9f), core);
    				} else {
    					drawShape(s, xf, new Color3f(255f*0.9f, 255f*0.9f, 255f*0.9f), core);
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
    		BroadPhase bp = m_broadPhase;
    		Vec2 invQ = new Vec2(0.0f, 0.0f);
    		invQ.set(1.0f / bp.m_quantizationFactor.x, 1.0f / bp.m_quantizationFactor.y);
    		Color3f color = new Color3f(255f*0.9f, 255f*0.9f, 255f*0.3f);

    		for (int i = 0; i < PairManager.TABLE_CAPACITY; ++i) {
    			int index = bp.m_pairManager.m_hashTable[i];
    			while (index != PairManager.NULL_PAIR) {
    				Pair pair = bp.m_pairManager.m_pairs[index];
    				Proxy p1 = bp.m_proxyPool[pair.proxyId1];
    				Proxy p2 = bp.m_proxyPool[pair.proxyId2];

    				AABB b1 = new AABB();
    				AABB b2 = new AABB();
    				b1.lowerBound.x = bp.m_worldAABB.lowerBound.x + invQ.x * bp.m_bounds[0][p1.lowerBounds[0]].value;
    				b1.lowerBound.y = bp.m_worldAABB.lowerBound.y + invQ.y * bp.m_bounds[1][p1.lowerBounds[1]].value;
    				b1.upperBound.x = bp.m_worldAABB.lowerBound.x + invQ.x * bp.m_bounds[0][p1.upperBounds[0]].value;
    				b1.upperBound.y = bp.m_worldAABB.lowerBound.y + invQ.y * bp.m_bounds[1][p1.upperBounds[1]].value;
    				b2.lowerBound.x = bp.m_worldAABB.lowerBound.x + invQ.x * bp.m_bounds[0][p2.lowerBounds[0]].value;
    				b2.lowerBound.y = bp.m_worldAABB.lowerBound.y + invQ.y * bp.m_bounds[1][p2.lowerBounds[1]].value;
    				b2.upperBound.x = bp.m_worldAABB.lowerBound.x + invQ.x * bp.m_bounds[0][p2.upperBounds[0]].value;
    				b2.upperBound.y = bp.m_worldAABB.lowerBound.y + invQ.y * bp.m_bounds[1][p2.upperBounds[1]].value;

    				Vec2 x1 = new Vec2(0.5f * (b1.lowerBound.x + b1.upperBound.x),
    								   0.5f * (b1.lowerBound.y + b1.upperBound.y));
    				Vec2 x2 = new Vec2(0.5f * (b2.lowerBound.x + b2.upperBound.x),
    								   0.5f * (b2.lowerBound.y + b2.upperBound.y));

    				m_debugDraw.drawSegment(x1, x2, color);

    				index = pair.next;
    			}
    		}
    	}


		BroadPhase bp = m_broadPhase;
		Vec2 worldLower = bp.m_worldAABB.lowerBound;
		Vec2 worldUpper = bp.m_worldAABB.upperBound;
		
    	if ( (flags & DebugDraw.e_aabbBit) != 0) {

    		Vec2 invQ = new Vec2();
    		invQ.set(1.0f / bp.m_quantizationFactor.x, 1.0f / bp.m_quantizationFactor.y);
    		Color3f color = new Color3f(255f*0.9f, 255f*0.3f,255f* 0.9f);
    		for (int i = 0; i < Settings.maxProxies; ++i) {
    			Proxy p = bp.m_proxyPool[i];
    			if (p.isValid() == false) {
    				continue;
    			}

    			AABB b = new AABB();
    			b.lowerBound.x = worldLower.x + invQ.x * bp.m_bounds[0][p.lowerBounds[0]].value;
    			b.lowerBound.y = worldLower.y + invQ.y * bp.m_bounds[1][p.lowerBounds[1]].value;
    			b.upperBound.x = worldLower.x + invQ.x * bp.m_bounds[0][p.upperBounds[0]].value;
    			b.upperBound.y = worldLower.y + invQ.y * bp.m_bounds[1][p.upperBounds[1]].value;

    			Vec2[] vs = new Vec2[4];
    			vs[0] = new Vec2(b.lowerBound.x, b.lowerBound.y);
    			vs[1] = new Vec2(b.upperBound.x, b.lowerBound.y);
    			vs[2] = new Vec2(b.upperBound.x, b.upperBound.y);
    			vs[3] = new Vec2(b.lowerBound.x, b.upperBound.y);

    			m_debugDraw.drawPolygon(vs, 4, color);
    		}

    		
    	}
    	
    	Vec2[] vsw = new Vec2[4];
		vsw[0]= new Vec2(worldLower.x, worldLower.y);
		vsw[1]= new Vec2(worldUpper.x, worldLower.y);
		vsw[2]= new Vec2(worldUpper.x, worldUpper.y);
		vsw[3]= new Vec2(worldLower.x, worldUpper.y);
		m_debugDraw.drawPolygon(vsw, 4, new Color3f(255.0f*0.3f, 255.0f*0.9f, 255.0f*0.9f));

    	if ( (flags & DebugDraw.e_obbBit) != 0) {
    		Color3f color = new Color3f(0.5f, 0.3f, 0.5f);

    		for (Body b = m_bodyList; b != null; b = b.getNext()) {
    			XForm xf = b.getXForm();
    			for (Shape s = b.getShapeList(); s != null; s = s.getNext()) {
    				if (s.getType() != ShapeType.POLYGON_SHAPE) {
    					continue;
    				}

    				PolygonShape poly = (PolygonShape)s;
    				OBB obb = poly.getOBB();
    				Vec2 h = obb.extents;
    				Vec2[] vs = new Vec2[4];
    				vs[0] = new Vec2(-h.x, -h.y);
    				vs[1] = new Vec2( h.x, -h.y);
    				vs[2] = new Vec2( h.x,  h.y);
    				vs[3] = new Vec2(-h.x,  h.y);

    				for (int i = 0; i < 4; ++i) {
    					vs[i] = obb.center.add(Mat22.mul(obb.R, vs[i]));
    					vs[i] = XForm.mul(xf, vs[i]);
    				}

    				m_debugDraw.drawPolygon(vs, 4, color);
    			}
    		}
    	}

    	if ( (flags & DebugDraw.e_centerOfMassBit) != 0) {
    		for (Body b = m_bodyList; b != null; b = b.getNext()) {
    			XForm xf = b.getXForm();
    			xf.position = b.getWorldCenter();
    			m_debugDraw.drawXForm(xf);
    		}
    	}
    }

    /** Enable/disable warm starting. For testing. */
	public void setWarmStarting(boolean flag) { m_warmStarting = flag; }

	/** Enable/disable position correction. For testing. */
	public void setPositionCorrection(boolean flag) { m_positionCorrection = flag; }

	/** Enable/disable continuous physics. For testing. */
	public void setContinuousPhysics(boolean flag) { m_continuousPhysics = flag; }
	
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
	public boolean inRange(AABB aabb) {
		return m_broadPhase.inRange(aabb);
	}
}
