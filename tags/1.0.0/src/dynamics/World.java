package dynamics;

import collision.AABB;
import collision.BroadPhase;
import collision.Shape;

import common.Vec2;

import dynamics.contacts.Contact;
import dynamics.contacts.ContactNode;
import dynamics.joints.Joint;
import dynamics.joints.JointDescription;
import dynamics.joints.JointNode;

public class World {
    public BroadPhase m_broadPhase;

    ContactManager m_contactManager;

    public Body m_bodyList;

    public Contact m_contactList;

    public Joint m_jointList;

    public int m_bodyCount;

    public int m_contactCount;

    public int m_jointCount;

    public Vec2 m_gravity;

    boolean m_doSleep;

    public Body m_groundBody;

    public static boolean s_enablePositionCorrection;

    public static boolean s_enableWarmStarting;

    public World(AABB worldAABB, Vec2 gravity, boolean doSleep) {
        m_bodyList = null;
        m_contactList = null;
        m_jointList = null;

        m_bodyCount = 0;
        m_contactCount = 0;
        m_jointCount = 0;

        m_doSleep = doSleep;

        m_gravity = gravity;

        m_contactManager = new ContactManager();
        m_contactManager.m_world = this;
        m_broadPhase = new BroadPhase(worldAABB, m_contactManager);

        BodyDescription bd = new BodyDescription();
        m_groundBody = CreateBody(bd);
    }

    public Body CreateBody(BodyDescription description) {
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

    // ewjordan: this function changed by 1.2.0 - see
    // ewjordan/Dynamics/b2World.java
    public void DestroyBody(Body b) {
        // Delete the attached joints
        JointNode jn = b.m_jointList;
        while (jn != null) {
            JointNode jn0 = jn;
            jn = jn.next;

            // Detach jn0 from the other body.
            Body other = jn0.other;
            other.wakeUp();

            JointNode node = other.m_jointList;
            boolean found = false;
            while (node != null) {
                if (node == jn0) {
                    node = node.next;
                    found = true;
                    break;
                }
                else {
                    node = node.next;
                }
            }
            assert found == true;

            // Remove joint from world list.
            Joint j = jn0.joint;
            if (j.m_prev != null) {
                j.m_prev.m_next = j.m_next;
            }

            if (j.m_next != null) {
                j.m_next.m_prev = j.m_prev;
            }

            if (j == m_jointList) {
                m_jointList = j.m_next;
            }

            // b2Joint::Destroy(j, &m_blockAllocator);
            assert m_jointCount > 0;
            --m_jointCount;
        }

        // Remove body from world list.
        if (b.m_prev != null) {
            b.m_prev.m_next = b.m_next;
        }

        if (b.m_next != null) {
            b.m_next.m_prev = b.m_prev;
        }

        if (b == m_bodyList) {
            m_bodyList = b.m_next;
        }
        assert m_bodyCount > 0;
        --m_bodyCount;
    }

    public Joint CreateJoint(JointDescription description) {
        Joint j = Joint.Create(description);

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

        return j;
    }

    public void DestroyJoint(Joint j) {
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

        assert m_jointCount > 0;
        --m_jointCount;
    }

    public void Step(float dt, int iterations) {
        // Create and/or update contacts.
        m_contactManager.Collide();

        // Size the island for the worst case.
        Island island = new Island(m_bodyCount, m_contactCount, m_jointCount);

        // Clear all the island flags.
        for (Body b = m_bodyList; b != null; b = b.m_next) {
            b.m_islandFlag = false;
        }
        for (Contact c = m_contactList; c != null; c = c.m_next) {
            c.m_islandFlag = false;
        }
        for (Joint j = m_jointList; j != null; j = j.m_next) {
            j.m_islandFlag = false;
        }

        // Build and simulate all awake islands.
        int stackSize = m_bodyCount;
        Body[] stack = new Body[stackSize];
        for (Body seed = m_bodyList; seed != null; seed = seed.m_next) {
            if (seed.m_invMass == 0.0f || seed.m_islandFlag == true
                    || seed.m_isSleeping == true) {
                continue;
            }

            // Reset island and stack.
            island.Clear();
            int stackCount = 0;
            stack[stackCount++] = seed;
            seed.m_islandFlag = true;

            // Perform a depth first search (DFS) on the constraint graph.
            while (stackCount > 0) {
                // Grab the next body off the stack and add it to the island.
                Body b = stack[--stackCount];
                island.Add(b);

                // Make sure the body is awake.
                b.m_isSleeping = false;

                // To keep islands as small as possible, we don't
                // propagate islands across static bodies.
                if (b.m_invMass == 0.0f) {
                    continue;
                }

                // Search all contacts connected to this body.
                for (ContactNode cn = b.m_contactList; cn != null; cn = cn.next) {
                    if (cn.contact.m_islandFlag == true) {
                        continue;
                    }

                    island.Add(cn.contact);
                    cn.contact.m_islandFlag = true;

                    Body other = cn.other;
                    if (other.m_islandFlag == true) {
                        continue;
                    }

                    assert stackCount < stackSize;
                    stack[stackCount++] = other;
                    other.m_islandFlag = true;
                }

                // Search all joints connect to this body.
                for (JointNode jn = b.m_jointList; jn != null; jn = jn.next) {
                    if (jn.joint.m_islandFlag == true) {
                        continue;
                    }

                    island.Add(jn.joint);
                    jn.joint.m_islandFlag = true;

                    Body other = jn.other;
                    if (other.m_islandFlag == true) {
                        continue;
                    }

                    assert (stackCount < stackSize);
                    stack[stackCount++] = other;
                    other.m_islandFlag = true;
                }
            }

            island.Solve(m_gravity, iterations, dt);
            island.UpdateSleep(dt);

            // Allow static bodies to participate in other islands.
            for (int i = 0; i < island.m_bodyCount; ++i) {
                Body b = island.m_bodies[i];
                if (b.m_invMass == 0.0f) {
                    b.m_islandFlag = false;
                }
            }
        }

        m_broadPhase.Flush();
    }

    public Shape[] Query(AABB aabb, int maxCount) {
        Object[] objs = m_broadPhase.Query(aabb, maxCount);
        Shape[] ret = new Shape[objs.length];

        System.arraycopy(objs, 0, ret, 0, objs.length);

        return ret;
    }
}
