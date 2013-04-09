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

//#include "b2Joint.h"
//#include "b2DistanceJoint.h"
//#include "b2MouseJoint.h"
//#include "b2RevoluteJoint.h"
//#include "b2PrismaticJoint.h"
//#include "../../Common/b2BlockAllocator.h"
//#include "../../Dynamics/b2Body.h"

//#include <new.h>

//#ifndef JOINT_H
//#define JOINT_H

//#include "../../Common/b2Math.h"

//struct b2Body;
//struct b2Joint;
//class b2BlockAllocator;


//Java note: deal with enums
enum b2JointType
{
	e_unknownJoint,
	e_revoluteJoint,
	e_prismaticJoint,
	e_distanceJoint,
	e_pulleyJoint,
	e_mouseJoint
};

enum b2LimitState
{
	e_inactiveLimit,
	e_atLowerLimit,
	e_atUpperLimit,
	e_equalLimits
};

class b2Jacobian{
	
	public b2Vec2 linear1;
	public float32 angular1;
	public b2Vec2 linear2;
	public float32 angular2;

	public b2Jacobian(){
		linear1 = new b2Vec2();
		linear2 = new b2Vec2();
		angular1 = 0f;
		angular2 = 0f;
	}
	
	public void SetZero(){
		linear1.SetZero(); angular1 = 0.0f;
		linear2.SetZero(); angular2 = 0.0f;
	}
	
	public void Set(b2Vec2 x1, float32 a1, b2Vec2 x2, float32 a2){
		linear1 = new b2Vec2(x1); 
		linear2 = new b2Vec2(x2); 
		angular1 = a1;
		angular2 = a2;
	}
	
	public float32 Compute(b2Vec2 x1, float32 a1, b2Vec2 x2, float32 a2){
		return b2Math.b2Dot(linear1, x1) + angular1 * a1 + b2Math.b2Dot(linear2, x2) + angular2 * a2;
	}
};

class b2JointNode{
	public b2Body other;
	public b2Joint joint;
	public b2JointNode prev;
	public b2JointNode next;
	
	public b2JointNode(){
		other = null;
		joint = null;
		prev = null;
		next = null;
	}
};

class b2JointDef{
	public b2JointDef() {
		type = e_unknownJoint;
		userData = null;
		body1 = null;
		body2 = null;
		collideConnected = false;
	}

	public b2JointType type;
	public Object userData;
	public b2Body body1;
	public b2Body body2;
	public boolean collideConnected;
};

abstract class b2Joint{
	public abstract b2Vec2 GetAnchor1();
	public abstract b2Vec2 GetAnchor2();

	public b2Joint GetNext(){
		return m_next;
	}

	public Object GetUserData(){
		return m_userData;
	}

	//--------------- Internals Below -------------------

	public static b2Joint Create(b2JointDef def){//, b2BlockAllocator* allocator){
		b2Joint joint = null;
		
		switch (def.type){
			case e_distanceJoint:
			{
				//void* mem = allocator->Allocate(sizeof(b2DistanceJoint));
				joint = new b2DistanceJoint((b2DistanceJointDef)def);
			}
				break;
				
			case e_mouseJoint:
			{
				//void* mem = allocator->Allocate(sizeof(b2MouseJoint));
				joint = new b2MouseJoint((b2MouseJointDef)def);
			}
				break;
				
			case e_prismaticJoint:
			{
				//void* mem = allocator->Allocate(sizeof(b2PrismaticJoint));
				joint = new b2PrismaticJoint((b2PrismaticJointDef)def);
			}
				break;
				
			case e_revoluteJoint:
			{
				//void* mem = allocator->Allocate(sizeof(b2RevoluteJoint));
				joint = new b2RevoluteJoint((b2RevoluteJointDef)def);
			}
				break;
				
			default:
				b2Assert(false);
				break;
		}
		
		return joint;
	}
	
	public void Destroy(b2Joint joint){//, b2BlockAllocator* allocator)
		joint.b2JointDestructor();
		/*
		switch (joint->m_type)
		 {
			case e_distanceJoint:
				allocator->Free(joint, sizeof(b2DistanceJoint));
				break;
				
			case e_mouseJoint:
				allocator->Free(joint, sizeof(b2MouseJoint));
				break;
				
			case e_prismaticJoint:
				allocator->Free(joint, sizeof(b2PrismaticJoint));
				break;
				
			case e_revoluteJoint:
				allocator->Free(joint, sizeof(b2RevoluteJoint));
				break;
				
			default:
				b2Assert(false);
				break;
		 }
		 */
	}

	public b2Joint(b2JointDef def){
		m_type = def.type;
		m_prev = null;
		m_next = null;
		m_body1 = def.body1;
		m_body2 = def.body2;
		m_collideConnected = def.collideConnected;
		m_islandFlag = false;
		m_userData = def.userData;
	}
	
	public void b2JointDestructor() {}

	public abstract void PreSolve();
	public abstract void SolveVelocityConstraints(float32 dt);

	// This returns true if the position errors are within tolerance.
	public abstract boolean SolvePositionConstraints();

	public b2JointType m_type;
	public b2Joint* m_prev;
	public b2Joint* m_next;
	public b2JointNode m_node1;
	public b2JointNode m_node2;
	public b2Body* m_body1;
	public b2Body* m_body2;

	public bool m_islandFlag;
	public bool m_collideConnected;

	public Object m_userData;
};











#endif
