/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/ 
 * Box2D homepage: http://www.gphysics.com
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
package dynamics;

import collision.AABB;
import collision.BroadPhase;
import collision.Shape;

import common.Vec2;

import dynamics.contacts.Contact;
import dynamics.contacts.ContactNode;
import dynamics.joints.Joint;
import dynamics.joints.JointDef;
import dynamics.joints.JointNode;
import dynamics.CollisionFilter;

//Updated to rev 56 of b2World.cpp/.h

public class World {
    public BroadPhase m_broadPhase;

    ContactManager m_contactManager;

    public Body m_bodyList;

    public Contact m_contactList;

    public Joint m_jointList;

    public int m_bodyCount;

    public int m_contactCount;

    public int m_jointCount;

    public Body m_bodyDestroyList;

    public Vec2 m_gravity;

    public boolean m_allowSleep;

    public Body m_groundBody;

    public WorldListener m_listener;
    
    public CollisionFilter m_filter;
    
    public int m_positionIterationCount;

    public static boolean ENABLE_POSITION_CORRECTION;

    public static boolean ENABLE_WARM_STARTING;

    public Body getGroundBody() {
        return m_groundBody;
    }

    public Body getBodyList() {
        return m_bodyList;
    }

    public Joint getJointList() {
        return m_jointList;
    }

    public Contact getContactList() {
        return m_contactList;
    }

    public World(AABB worldAABB, Vec2 gravity, boolean doSleep) {
        m_listener = null;
        m_filter = CollisionFilter.DEFAULT_FILTER;

        m_bodyList = null;
        m_contactList = null;
        m_jointList = null;

        m_bodyCount = 0;
        m_contactCount = 0;
        m_jointCount = 0;

        m_bodyDestroyList = null;

        m_allowSleep = doSleep;

        m_gravity = gravity;

        m_contactManager = new ContactManager();
        m_contactManager.m_world = this;
        m_broadPhase = new BroadPhase(worldAABB, m_contactManager);

        BodyDef bd = new BodyDef();
        m_groundBody = createBody(bd);
    }

    public void setListener(WorldListener listener) {
        m_listener = listener;
    }
    
    public void setFilter(CollisionFilter filter) {
        m_filter = filter;
    }
    
    public Body createBody(BodyDef description) {
        Body b = new Body(description, this);
        b.m_prev = null;

        b.m_next = m_bodyList;
        if (m_bodyList != null) {
            m_bodyList.m_prev = b;
        }
        m_bodyList = b;
        ++m_bodyCount;

        return b;
    }

    // Body destruction is deferred to make contact processing more robust.
    public void destroyBody(Body b) {
        if ((b.m_flags & Body.e_destroyFlag) > 0) {
            return;
        }

        // Remove from normal body list
        if (b.m_prev != null) {
            b.m_prev.m_next = b.m_next;
        }

        if (b.m_next != null) {
            b.m_next.m_prev = b.m_prev;
        }

        if (b == m_bodyList) {
            m_bodyList = b.m_next;
        }

        b.m_flags |= Body.e_destroyFlag;
        assert (m_bodyCount > 0);
        --m_bodyCount;

        // Add to the deferred destruction list.
        b.m_prev = null;
        b.m_next = m_bodyDestroyList;
        m_bodyDestroyList = b;
    }

    public void cleanBodyList() {
        m_contactManager.m_destroyImmediate = true;

        Body b = m_bodyDestroyList;

        while (b != null) {
            assert ((b.m_flags & Body.e_destroyFlag) != 0);

            // Preserve the next pointer.
            Body b0 = b;
            b = b.m_next;

            // Delete the attached joints
            JointNode jn = b0.m_jointList;
            while (jn != null) {
                JointNode jn0 = jn;
                jn = jn.next;

                if (m_listener != null) {
                    m_listener.notifyJointDestroyed(jn0.joint);
                }

                destroyJoint(jn0.joint);
            }
            b0.destructor();
        }

        // Reset the list
        m_bodyDestroyList = null;

        m_contactManager.m_destroyImmediate = false;
    }

    public Joint createJoint(JointDef def) {
        Joint j = Joint.create(def);

        // Connect to the world list.
        j.m_prev = null;
        j.m_next = m_jointList;
        if (m_jointList != null) {
            m_jointList.m_prev = j;
        }
        m_jointList = j;
        ++m_jointCount;

        // Connect to the bodies
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
                s.resetProxy(m_broadPhase);
            }
        }

        return j;
    }

    public void destroyJoint(Joint j) {
        boolean collideConnected = j.m_collideConnected;

        // Remove from the world.
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

        // Wake up touching bodies.
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
                s.resetProxy(m_broadPhase);
            }
        }
    }

    public void step(float dt, int iterations) {
        TimeStep step = new TimeStep();
        step.dt = dt;
        step.iterations = iterations;
        if (dt > 0.0f) {
            step.inv_dt = 1.0f / dt;
        }
        else {
            step.inv_dt = 0.0f;
        }

        // Handle deferred contact destruction
        m_contactManager.cleanContactList();

        // Handle deferred body destruction
        cleanBodyList();

        // Update contacts.
        m_contactManager.collide();

        // Size the island for the worst case.
        Island island = new Island(m_bodyCount, m_contactCount, m_jointCount);

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
            if ((seed.m_flags & (Body.e_staticFlag | Body.e_islandFlag
                    | Body.e_sleepFlag | Body.e_frozenFlag)) > 0) {
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
                if ((b.m_flags & Body.e_staticFlag) > 0) {
                    continue;
                }

                // Search all contacts connected to this body.
                for (ContactNode cn = b.m_contactList; cn != null; cn = cn.next) {
                    if ((cn.contact.m_flags & Contact.e_islandFlag) > 0) {
                        continue;
                    }

                    island.add(cn.contact);
                    cn.contact.m_flags |= Contact.e_islandFlag;

                    Body other = cn.other;
                    if ((other.m_flags & Body.e_islandFlag) > 0) {
                        continue;
                    }

                    assert stackCount < stackSize;
                    stack[stackCount++] = other;
                    other.m_flags |= Body.e_islandFlag;
                }

                // Search all joints connect to this body.
                for (JointNode jn = b.m_jointList; jn != null; jn = jn.next) {
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

            island.solve(step, m_gravity);
            
            m_positionIterationCount = Math.max(m_positionIterationCount, island.m_positionIterationCount); 
            
            if (m_allowSleep) {
                island.updateSleep(dt);
            }

            // Post solve cleanup
            for (int i = 0; i < island.m_bodyCount; ++i) {
                // Allow static bodies to participate in other islands
                Body b = island.m_bodies[i];
                if ((b.m_flags & Body.e_staticFlag) > 0) {
                    b.m_flags &= ~Body.e_islandFlag;
                }

                // Handle newly frozen bodies
                if (b.isFrozen() && m_listener != null) {
                    BoundaryResponse response = m_listener
                            .notifyBoundaryViolated(b);
                    if (response == BoundaryResponse.DESTROY_BODY) {
                        destroyBody(b);
                        b = null;
                        island.m_bodies[i] = null;
                    }
                }
            }
        }

        m_broadPhase.commit();
        
        
//        #if 0
//        for (b2Contact* c = m_contactList; c; c = c->GetNext())
//        {
//            b2Shape* shape1 = c->GetShape1();
//            b2Shape* shape2 = c->GetShape2();
//            b2Body* body1 = shape1->GetBody();
//            b2Body* body2 = shape2->GetBody();
//
//            if (body1->IsSleeping() && body2->IsSleeping())
//            {
//                continue;
//            }
//
//            b2Conservative(shape1, shape2);
//        }
//    #endif
        
    }

    public Shape[] query(AABB aabb, int maxCount) {
        Object[] objs = m_broadPhase.query(aabb, maxCount);
        Shape[] ret = new Shape[objs.length];

        System.arraycopy(objs, 0, ret, 0, objs.length);

        return ret;
    }
}
