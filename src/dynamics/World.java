package dynamics;

import java.util.List;

import collision.AABB;
import collision.BroadPhase;

import common.Vec2;

import dynamics.contacts.Contact;
import dynamics.joints.Joint;

public class World {
	BroadPhase m_broadPhase;
	ContactManager m_contactManager;

	Body m_bodyList;
	Contact m_contactList;
	Joint m_jointList;

	int m_bodyCount;
	int m_contactCount;
	int m_jointCount;

	Vec2 m_gravity;
	boolean m_doSleep;

	Body groundBody;

	static int s_enablePositionCorrection;
	static int s_enableWarmStarting;

	public World(AABB worldAABB, Vec2 gravity, boolean doSleep)
	{
		m_bodyList = null;
		m_contactList = null;
		m_jointList = null;

		m_bodyCount = 0;
		m_contactCount = 0;
		m_jointCount = 0;

		m_doSleep = doSleep;

		m_gravity = gravity;

		m_contactManager.m_world = this;
		m_broadPhase = new BroadPhase(worldAABB, m_contactManager);

		b2BodyDescription bd;
		m_groundBody = CreateBody(&bd);
	}

// b2World::~b2World()
// {
// DestroyBody(m_groundBody);
// delete m_broadPhase;
// }

	Body CreateBody(BodyDescription description)
	{
		Body b = new Body(description, this);
		b.m_prev = null;
		
		b.m_next = m_bodyList;
		if (m_bodyList!=null)
		{
			m_bodyList.m_prev = b;
		}
		m_bodyList = b;
		++m_bodyCount;

		return b;
	}

	void b2World::DestroyBody(b2Body* b)
	{
		// Delete the attached joints
		b2JointNode* jn = b.m_jointList;
		while (jn)
		{
			b2JointNode* jn0 = jn;
			jn = jn.next;

			// Detach jn0 from the other body.
			b2Body* other = jn0.other;
			other.WakeUp();
			b2JointNode** node = &other.m_jointList;
			bool found = false;
			while (*node)
			{
				if (*node == jn0)
				{
					*node = (*node).next;
					found = true;
					break;
				}
				else
				{
					node = &(*node).next;
				}
			}
			b2Assert(found == true);

			// Remove joint from world list.
			b2Joint* j = jn0.joint;
			if (j.m_prev)
			{
				j.m_prev.m_next = j.m_next;
			}

			if (j.m_next)
			{
				j.m_next.m_prev = j.m_prev;
			}

			if (j == m_jointList)
			{
				m_jointList = j.m_next;
			}

			b2Joint::Destroy(j, &m_blockAllocator);
			b2Assert(m_jointCount > 0);
			--m_jointCount;
		}

		// Remove body from world list.
		if (b.m_prev)
		{
			b.m_prev.m_next = b.m_next;
		}

		if (b.m_next)
		{
			b.m_next.m_prev = b.m_prev;
		}

		if (b == m_bodyList)
		{
			m_bodyList = b.m_next;
		}
		b2Assert(m_bodyCount > 0);
		--m_bodyCount;

		b.~b2Body();
		m_blockAllocator.Free(b, sizeof(b2Body));
	}

	b2Joint* b2World::CreateJoint(const b2JointDescription* description)
	{
		b2Joint* j = b2Joint::Create(description, &m_blockAllocator);

		// Connect to the world list.
		j.m_prev = null;
		j.m_next = m_jointList;
		if (m_jointList)
		{
			m_jointList.m_prev = j;
		}
		m_jointList = j;
		++m_jointCount;

		// Connect to the bodies
		j.m_node1.joint = j;
		j.m_node1.other = j.m_body2;
		j.m_node1.prev = null;
		j.m_node1.next = j.m_body1.m_jointList;
		if (j.m_body1.m_jointList) j.m_body1.m_jointList.prev = &j.m_node1;
		j.m_body1.m_jointList = &j.m_node1;

		j.m_node2.joint = j;
		j.m_node2.other = j.m_body1;
		j.m_node2.prev = null;
		j.m_node2.next = j.m_body2.m_jointList;
		if (j.m_body2.m_jointList) j.m_body2.m_jointList.prev = &j.m_node2;
		j.m_body2.m_jointList = &j.m_node2;

		return j;
	}

	void b2World::DestroyJoint(b2Joint* j)
	{
		// Remove from the world.
		if (j.m_prev)
		{
			j.m_prev.m_next = j.m_next;
		}

		if (j.m_next)
		{
			j.m_next.m_prev = j.m_prev;
		}

		if (j == m_jointList)
		{
			m_jointList = j.m_next;
		}

		// Disconnect from island graph.
		b2Body* body1 = j.m_body1;
		b2Body* body2 = j.m_body2;

		// Wake up touching bodies.
		body1.WakeUp();
		body2.WakeUp();

		// Remove from body 1
		if (j.m_node1.prev)
		{
			j.m_node1.prev.next = j.m_node1.next;
		}

		if (j.m_node1.next)
		{
			j.m_node1.next.prev = j.m_node1.prev;
		}

		if (&j.m_node1 == body1.m_jointList)
		{
			body1.m_jointList = j.m_node1.next;
		}

		j.m_node1.prev = null;
		j.m_node1.next = null;

		// Remove from body 2
		if (j.m_node2.prev)
		{
			j.m_node2.prev.next = j.m_node2.next;
		}

		if (j.m_node2.next)
		{
			j.m_node2.next.prev = j.m_node2.prev;
		}

		if (&j.m_node2 == body2.m_jointList)
		{
			body2.m_jointList = j.m_node2.next;
		}

		j.m_node2.prev = null;
		j.m_node2.next = null;

		b2Joint::Destroy(j, &m_blockAllocator);

		b2Assert(m_jointCount > 0);
		--m_jointCount;
	}

	void b2World::Step(float32 dt, int32 iterations)
	{
		// Create and/or update contacts.
		m_contactManager.Collide();

		// Size the island for the worst case.
		b2Island island(m_bodyCount, m_contactCount, m_jointCount, &m_stackAllocator);

		// Clear all the island flags.
		for (b2Body* b = m_bodyList; b; b = b.m_next)
		{
			b.m_islandFlag = false;
		}
		for (b2Contact* c = m_contactList; c; c = c.m_next)
		{
			c.m_islandFlag = false;
		}
		for (b2Joint* j = m_jointList; j; j = j.m_next)
		{
			j.m_islandFlag = false;
		}
		
		// Build and simulate all awake islands.
		int32 stackSize = m_bodyCount;
		b2Body** stack = (b2Body**)m_stackAllocator.Allocate(stackSize * sizeof(b2Body*));
		for (b2Body* seed = m_bodyList; seed; seed = seed.m_next)
		{
			if (seed.m_invMass == 0.0f ||
				seed.m_islandFlag == true ||
				seed.m_isSleeping == true)
			{
				continue;
			}

			// Reset island and stack.
			island.Clear();
			int32 stackCount = 0;
			stack[stackCount++] = seed;
			seed.m_islandFlag = true;

			// Perform a depth first search (DFS) on the constraint graph.
			while (stackCount > 0)
			{
				// Grab the next body off the stack and add it to the island.
				b2Body* b = stack[--stackCount];
				island.Add(b);

				// Make sure the body is awake.
				b.m_isSleeping = false;

				// To keep islands as small as possible, we don't
				// propagate islands across static bodies.
				if (b.m_invMass == 0.0f)
				{
					continue;
				}

				// Search all contacts connected to this body.
				for (b2ContactNode* cn = b.m_contactList; cn; cn = cn.next)
				{
					if (cn.contact.m_islandFlag == true)
					{
						continue;
					}

					island.Add(cn.contact);
					cn.contact.m_islandFlag = true;

					b2Body* other = cn.other;
					if (other.m_islandFlag == true)
					{
						continue;
					}

					b2Assert(stackCount < stackSize);
					stack[stackCount++] = other;
					other.m_islandFlag = true;
				}

				// Search all joints connect to this body.
				for (b2JointNode* jn = b.m_jointList; jn; jn = jn.next)
				{
					if (jn.joint.m_islandFlag == true)
					{
						continue;
					}

					island.Add(jn.joint);
					jn.joint.m_islandFlag = true;

					b2Body* other = jn.other;
					if (other.m_islandFlag == true)
					{
						continue;
					}

					b2Assert(stackCount < stackSize);
					stack[stackCount++] = other;
					other.m_islandFlag = true;
				}
			}

			island.Solve(m_gravity, iterations, dt);
			island.UpdateSleep(dt);

			// Allow static bodies to participate in other islands.
			for (int32 i = 0; i < island.m_bodyCount; ++i)
			{
				b2Body* b = island.m_bodies[i];
				if (b.m_invMass == 0.0f)
				{
					b.m_islandFlag = false;
				}
			}
		}

		m_broadPhase.Flush();

		m_stackAllocator.Free(stack);
	}

	int32 b2World::Query(const b2AABB& aabb, b2Shape** shapes, int32 maxCount)
	{
		void** results = (void**)m_stackAllocator.Allocate(maxCount * sizeof(void*));

		int32 count = m_broadPhase.Query(aabb, results, maxCount);

		for (int32 i = 0; i < count; ++i)
		{
			shapes[i] = (b2Shape*)results[i];
		}

		m_stackAllocator.Free(results);
		return count;
	}

}
