/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

//#ifndef B2_WORLD_H
//#define B2_WORLD_H

//#include "../Common/b2Math.h"
//#include "../Common/b2BlockAllocator.h"
//#include "../Common/b2StackAllocator.h"
//#include "b2ContactManager.h"
//#include "b2WorldCallbacks.h"

//struct b2AABB;
//struct b2BodyDef;
//struct b2JointDef;
//struct b2Body;
//struct b2Joint;
//struct b2Shape;
//struct b2Contact;
//class b2BroadPhase;

class b2World {

	
	//--------------- Members Below -------------------
	
	public b2BlockAllocator m_blockAllocator;
	public b2StackAllocator m_stackAllocator;
	
	public b2BroadPhase m_broadPhase;
	public b2ContactManager m_contactManager;
	
	public b2Body m_bodyList; //each of these is the start of linked list
	public b2Contact m_contactList;
	public b2Joint m_jointList;	
	
	public int32 m_bodyCount;
	public int32 m_contactCount;
	public int32 m_jointCount;
	
	public b2Vec2 m_gravity;
	public boolean m_doSleep;
	
	public b2Body m_groundBody;
	
	public b2JointDestroyedCallback m_jointDestroyedCallback;
	
	public static int32 s_enablePositionCorrection;
	public static int32 s_enableWarmStarting;
	
	public int32 s_enablePositionCorrection = 1;
	public int32 s_enableWarmStarting = 1;
	
	public b2World(b2AABB worldAABB, b2Vec2 gravity, boolean doSleep){
		m_jointDestroyedCallback = null;
		
		m_bodyList = null;
		m_contactList = null;
		m_jointList = null;
		
		m_bodyCount = 0;
		m_contactCount = 0;
		m_jointCount = 0;
		
		m_doSleep = doSleep;
		
		m_gravity = gravity;
		
		m_contactManager.m_world = this;
		m_broadPhase = new b2BroadPhase(worldAABB, m_contactManager);
		
		b2BodyDef bd = new BodyDef();
		m_groundBody = CreateBody(bd);
	}
	
	//We're leaving destructors in by default for now, as
	//some might perform important cleanup tasks.  Will
	//remove as needed once rest of code is cleaned up for Java.
	//[esp. if never explicitly called]
	public void b2WorldDestructor(){
		//DestroyBody(m_groundBody);
		//delete m_broadPhase;
	}
	
	// Set a callback to notify you when a joint is implicitly destroyed
	// when an attached body is destroyed.
	public void SetJointDestroyedCallback(b2JointDestroyedCallback callback){
		m_jointDestroyedCallback = callback;
	}
	
	public b2Body CreateBody(b2BodyDef def){
		//Since we're not implementing the block allocator, we'll just use new.
		//Might want to do something more subtle in the future...
		//void* mem = m_blockAllocator.Allocate(sizeof(b2Body));
		//b2Body* b = new (mem) b2Body(def, this);
		b2body b = new b2Body(def, this);
		b.m_prev = null;
		
		b.m_next = m_bodyList;
		if (m_bodyList != null){
			m_bodyList.m_prev = b;
		}
		m_bodyList = b;
		++m_bodyCount;
		
		return b;
	}
	
	
	public void DestroyBody(b2Body b){
		// Delete the attached joints
		b2JointNode jn = b.m_jointList;
		while (jn != null){
			b2JointNode jn0 = jn;
			jn = jn.next;
			
			if (m_jointDestroyedCallback){
				m_jointDestroyedCallback.Notify(jn0.joint);
			}
			
			DestroyJoint(jn0.joint);
		}
		
		// Remove body from world list.
		if (b.m_prev != null){
			b.m_prev.m_next = b.m_next;
		}
		
		if (b.m_next != null){
			b.m_next.m_prev = b.m_prev;
		}
		
		if (b == m_bodyList){
			m_bodyList = b.m_next;
		}
		
		b2Assert(m_bodyCount > 0);
		--m_bodyCount;
		
		b.b2BodyDestructor();
		//m_blockAllocator.Free(b, sizeof(b2Body));
	}
	
	public b2Joint CreateJoint(b2JointDef def){
		b2Joint j = b2Joint::Create(def);//, &m_blockAllocator);//no block allocator in Java
		
		// Connect to the world list.
		j.m_prev = null;
		j.m_next = m_jointList;
		if (m_jointList != null){
			m_jointList.m_prev = j;
		}
		m_jointList = j;
		++m_jointCount;
		
		// Connect to the bodies
		j.m_node1.joint = j;
		j.m_node1.other = j.m_body2;
		j.m_node1.prev = null;
		j.m_node1.next = j.m_body1.m_jointList;
		if (j.m_body1.m_jointList != null) j.m_body1.m_jointList.prev = j.m_node1;
		j.m_body1.m_jointList = j.m_node1;
		
		j.m_node2.joint = j;
		j.m_node2.other = j.m_body1;
		j.m_node2.prev = null;
		j.m_node2.next = j.m_body2.m_jointList;
		if (j.m_body2.m_jointList != null) j.m_body2.m_jointList.prev = j.m_node2;
		j.m_body2.m_jointList = j.m_node2;
		
		return j;
	}
	
	public void DestroyJoint(b2Joint j){
		// Remove from the world.
		if (j.m_prev != null){
			j.m_prev.m_next = j.m_next;
		}
		
		if (j.m_next){
			j.m_next.m_prev = j.m_prev;
		}
		
		if (j == m_jointList){
			m_jointList = j.m_next;
		}
		
		// Disconnect from island graph.
		b2Body body1 = j.m_body1;
		b2Body body2 = j.m_body2;
		
		// Wake up touching bodies.
		body1.WakeUp();
		body2.WakeUp();
		
		// Remove from body 1
		if (j.m_node1.prev != null){
			j.m_node1.prev.next = j.m_node1.next;
		}
		
		if (j.m_node1.next){
			j.m_node1.next.prev = j.m_node1.prev;
		}
		
		if (j.m_node1 == body1.m_jointList){
			body1.m_jointList = j.m_node1.next;
		}
		
		j.m_node1.prev = null;
		j.m_node1.next = null;
		
		// Remove from body 2
		if (j.m_node2.prev != null){
			j.m_node2.prev.next = j.m_node2.next;
		}
		
		if (j.m_node2.next != null){
			j.m_node2.next.prev = j.m_node2.prev;
		}
		
		if (j.m_node2 == body2.m_jointList){
			body2.m_jointList = j.m_node2.next;
		}
		
		j.m_node2.prev = null;
		j.m_node2.next = null;
		
		b2Joint.Destroy(j);//, &m_blockAllocator);
		
		b2Assert(m_jointCount > 0);
		--m_jointCount;
	}
	
	// The world provides a single ground body with no collision shapes. You
	// can use this to simplify the creation of joints.
	public b2Body GetGroundBody(){
		return m_groundBody;
	}
	
	public void Step(float32 dt, int32 iterations){
		// Create and/or update contacts.
		m_contactManager.Collide();
		
		// Size the island for the worst case.
		b2Island island(m_bodyCount, m_contactCount, m_jointCount, &m_stackAllocator);
		
		// Clear all the island flags.
		for (b2Body b = m_bodyList; b != null; b = b.m_next){
			b.m_islandFlag = false;
		}
		for (b2Contact c = m_contactList; c != null; c = c.m_next){
			c.m_islandFlag = false;
		}
		for (b2Joint j = m_jointList; j != null; j = j.m_next){
			j.m_islandFlag = false;
		}
		
		// Build and simulate all awake islands.
		int32 stackSize = m_bodyCount;
		//b2Body** stack = (b2Body**)m_stackAllocator.Allocate(stackSize * sizeof(b2Body*));
		b2Body[] stack = new b2Body[stackSize];
		for (b2Body seed = m_bodyList; seed != null; seed = seed.m_next){
			if (seed.m_invMass == 0.0f ||
				seed.m_islandFlag == true ||
				seed.m_isSleeping == true){
				continue;
			}
			
			// Reset island and stack.
			island.Clear();
			int32 stackCount = 0;
			stack[stackCount++] = seed;
			seed->m_islandFlag = true;
			
			// Perform a depth first search (DFS) on the constraint graph.
			while (stackCount > 0){
				// Grab the next body off the stack and add it to the island.
				b2Body b = stack[--stackCount];
				island.Add(b);
				
				// Make sure the body is awake.
				b.m_isSleeping = false;
				
				// To keep islands as small as possible, we don't
				// propagate islands across static bodies.
				if (b.m_invMass == 0.0f){
					continue;
				}
				
				// Search all contacts connected to this body.
				for (b2ContactNode cn = b.m_contactList; cn != null; cn = cn.next){
					if (cn.contact.m_islandFlag == true){
						continue;
					}
					
					island.Add(cn.contact);
					cn.contact.m_islandFlag = true;
					
					b2Body other = cn.other;
					if (other.m_islandFlag == true){
						continue;
					}
					
					b2Assert(stackCount < stackSize);
					stack[stackCount++] = other;
					other.m_islandFlag = true;
				}
				
				// Search all joints connect to this body.
				for (b2JointNode jn = b.m_jointList; jn != null; jn = jn.next){
					if (jn.joint.m_islandFlag == true){
						continue;
					}
					
					island.Add(jn.joint);
					jn.joint.m_islandFlag = true;
					
					b2Body other = jn.other;
					if (other.m_islandFlag == true){
						continue;
					}
					
					b2Assert(stackCount < stackSize);
					stack[stackCount++] = other;
					other.m_islandFlag = true;
				}
			}
			
			island.Solve(m_gravity, iterations, dt);
			if (m_doSleep){
				island.UpdateSleep(dt);
			}
			
			// Allow static bodies to participate in other islands.
			for (int32 i = 0; i < island.m_bodyCount; ++i){
				b2Body b = island.m_bodies[i];
				if (b.m_invMass == 0.0f){
					b.m_islandFlag = false;
				}
			}
		}
		
		m_broadPhase.Flush();
		
		//m_stackAllocator.Free(stack);
	}
	
	// Query the world for all shapes that potentially overlap the
	// provided AABB. You provide a shape pointer buffer of specified
	// size. The number of shapes found is returned.
	public int32 Query(b2AABB aabb, b2Shape[] shapes, int32 maxCount){
		//void** results = (void**)m_stackAllocator.Allocate(maxCount * sizeof(void*));
		Object[] results = new Object[maxCount]; //I think that's the best translation we can do of the void* array
		
		int32 count = m_broadPhase.Query(aabb, results, maxCount);
		
		for (int32 i = 0; i < count; ++i){
			shapes[i] = (b2Shape)results[i]; //hmm, why is this b2Shape cast valid?  In BroadPhase results is just a seemingly generic object array...
		}
		
//		m_stackAllocator.Free(results);
		return count;
	}
	
	// You can use these to iterate over all the bodies, joints, and contacts.
	public b2Body GetBodyList(){
		return m_bodyList;
	}
	
	public b2Joint GetJointList(){
		return m_jointList;
	}
	
	public b2Contact GetContactList(){
		return m_contactList;
	}
	
}



//#endif


//#include "b2World.h"
//#include "b2Body.h"
//#include "b2Island.h"
//#include "Joints/b2Joint.h"
//#include "Contacts/b2Contact.h"
//#include "../Collision/b2Collision.h"
//#include "../Collision/b2Shape.h"
//#include <new.h>





