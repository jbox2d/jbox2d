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



//#include "b2ContactManager.h"
//#include "b2World.h"
//#include "b2Body.h"
//#ifndef B2_CONTACT_MANAGER_H
//#define B2_CONTACT_MANAGER_H

//#include "../Collision/b2BroadPhase.h"
//#include "../Dynamics/Contacts/b2NullContact.h"

//struct b2World;

class b2ContactManager extends b2PairCallback{
	public b2ContactManager(){
		super();
		m_world = null;
		m_nullContact = new b2NullContact();
	}
	
	// Implements PairCallback
	// This is a callback from the broadphase when two AABB proxies begin
	// to overlap. We create a b2Contact to manage the narrow phase.
	public Object PairAdded(Object proxyUserData1, Object proxyUserData2)
	{
		b2Shape shape1 = (b2Shape)proxyUserData1;
		b2Shape shape2 = (b2Shape)proxyUserData2;
		
		b2Body body1 = shape1.m_body;
		b2Body body2 = shape2.m_body;
		
		if (body1.IsStatic() && body2.IsStatic()){
			return m_nullContact;
		}
		
		if (shape1.m_body == shape2.m_body){
			return m_nullContact;
		}
		
		// Ensure that body2 is dynamic (body1 is static or dynamic).
		if (body2.m_invMass == 0.0f){
			//b2Swap(shape1, shape2);
			b2Shape buff = shape1;
			shape1 = shape2;
			shape2 = buff;
			//b2Swap(body1, body2);
			b2Body buffb = body1;
			body1 = body2;
			body2 = buffb;
		}
		
		if (body2.IsConnected(body1)){
			return m_nullContact;
		}
		
		// Call the factory.
		b2Contact contact = b2Contact.Create(shape1, shape2);//, m_world.m_blockAllocator);
		
		if (contact == null){
			return m_nullContact;
		} else{
			// Insert into the world.
			contact.m_prev = null;
			contact.m_next = m_world.m_contactList;
			if (m_world.m_contactList != null){
				m_world.m_contactList.m_prev = contact;
			}
			m_world.m_contactList = contact;
			++(m_world.m_contactCount);
		}
		
		return contact;
	}
	
	// Implements PairCallback
	// This is a callback from the broadphase when two AABB proxies cease
	// to overlap. We destroy the b2Contact.
	public void PairRemoved(Object proxyUserData1, Object proxyUserData2, Object pairUserData){
		NOT_USED(proxyUserData1);
		NOT_USED(proxyUserData2);
		
		b2Contact c = (b2Contact)pairUserData;
		if (c != m_nullContact){
			// Remove from the world.
			if (c.m_prev){
				c.m_prev.m_next = c.m_next;
			}
			
			if (c.m_next){
				c.m_next.m_prev = c.m_prev;
			}
			
			if (c == m_world.m_contactList){
				m_world.m_contactList = c.m_next;
			}
			
			if (c.GetManifoldCount() > 0){
				b2Body body1 = c.m_shape1.m_body;
				b2Body body2 = c.m_shape2.m_body;
				
				// Wake up touching bodies.
				body1.WakeUp();
				body2.WakeUp();
				
				// Disconnect from island graph.
				// Remove from body 1
				if (c.m_node1.prev){
					c.m_node1.prev.next = c.m_node1.next;
				}
				
				if (c.m_node1.next){
					c.m_node1.next.prev = c.m_node1.prev;
				}
				
				if (c.m_node1 == body1.m_contactList){
					body1.m_contactList = c.m_node1.next;
				}
				
				c.m_node1.prev = null;
				c.m_node1.next = null;
				
				// Remove from body 2
				if (c.m_node2.prev){
					c.m_node2.prev.next = c.m_node2.next;
				}
				
				if (c.m_node2.next){
					c.m_node2.next.prev = c.m_node2.prev;
				}
				
				if (c.m_node2 == body2.m_contactList){
					body2.m_contactList = c.m_node2.next;
				}
				
				c.m_node2.prev = null;
				c.m_node2.next = null;
			}
			
			// Call the factory.
			b2Contact.Destroy(c);//, &m_world->m_blockAllocator);
			--(m_world.m_contactCount);
		}
	}
	
	
	// This is the top level collision call for the time step. Here
	// all the narrow phase collision is processed for the world
	// contact list.
	public void Collide(){
		for (b2Contact c = m_world.m_contactList; c != null; c = c.m_next){
			if (c.m_shape1.m_body.IsSleeping() &&
				c.m_shape2.m_body.IsSleeping()){
				continue;
			}
			
			int32 oldCount = c.GetManifoldCount();
			c.Evaluate();
			
			int32 newCount = c.GetManifoldCount();
			
			if (oldCount == 0 && newCount > 0){
				// Connect to island graph.
				
				b2Body body1 = c.m_shape1.m_body;
				b2Body body2 = c.m_shape2.m_body;
				
				// Connect to body 1
				c.m_node1.contact = c;
				c.m_node1.other = body2;
				
				c.m_node1.prev = null;
				c.m_node1.next = body1.m_contactList;
				if (c.m_node1.next != null){
					c.m_node1.next.prev = c.m_node1;
				}
				body1.m_contactList = c.m_node1;
				
				// Connect to body 2
				c.m_node2.contact = c;
				c.m_node2.other = body1;
				
				c.m_node2.prev = null;
				c.m_node2.next = body2.m_contactList;
				if (c.m_node2.next != null){
					c.m_node2.next.prev = c.m_node2;
				}
				body2.m_contactList = c.m_node2;
			} else if (oldCount > 0 && newCount == 0){
				// Disconnect from island graph.
				b2Body body1 = c.m_shape1.m_body;
				b2Body body2 = c.m_shape2.m_body;
				
				// Remove from body 1
				if (c.m_node1.prev){
					c.m_node1.prev.next = c.m_node1.next;
				}
				
				if (c.m_node1.next){
					c.m_node1.next.prev = c.m_node1.prev;
				}
				
				if (c.m_node1 == body1.m_contactList){
					body1.m_contactList = c.m_node1.next;
				}
				
				c.m_node1.prev = null;
				c.m_node1.next = null;
				
				// Remove from body 2
				if (c.m_node2.prev){
					c.m_node2.prev.next = c.m_node2.next;
				}
				
				if (c.m_node2.next){
					c.m_node2.next.prev = c.m_node2.prev;
				}
				
				if (c.m_node2 == body2.m_contactList){
					body2.m_contactList = c.m_node2.next;
				}
				
				c.m_node2.prev = null;
				c.m_node2.next = null;
			}
		}
	}
	
	public b2World m_world;
	
	// This lets us provide broadphase proxy pair user data for
	// contacts that shouldn't exist.
	public b2NullContact m_nullContact;
};

//#endif






