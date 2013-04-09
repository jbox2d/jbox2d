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

//Java note: might want to consider static import of b2Math, I'm having to
//qualify a lot of calls to it and it's starting to annoy me...

//#include "b2Body.h"
//#include "b2World.h"
//#include "Joints/b2Joint.h"
//#include "Contacts/b2Contact.h"
//#include "../Collision/b2Shape.h"

//#ifndef BODY_H
//#define BODY_H

//#include "../Common/b2Math.h"
//#include "../Dynamics/Joints/b2Joint.h"
//#include "../Collision/b2Shape.h"

//#include <string.h>

//struct b2Joint;
//struct b2Contact;
//struct b2World;
//struct b2JointNode;
//struct b2ContactNode;

class b2BodyDef{
	public b2BodyDef(){
		shapes = new b2ShapeDef[b2_maxShapesPerBody];
		for (int i=0; i<shapes.length; i++){
			shapes[i] = null;
		}
		userData = null;
//		memset(shapes, 0, sizeof(shapes));
		position = new b2Vec2(0.0f, 0.0f);
		rotation = 0.0f;
		linearVelocity = new b2Vec2(0.0f, 0.0f);
		angularVelocity = 0.0f;
		allowSleep = true;
		isSleeping = false;
	}

	public Object userData; // was void*
	public b2ShapeDef[] shapes;//[b2_maxShapesPerBody];
	public b2Vec2 position;
	public float32 rotation;
	public b2Vec2 linearVelocity;
	public float32 angularVelocity;
	public boolean allowSleep;
	public boolean isSleeping;

	public void AddShape(b2ShapeDef shape){
		for (int32 i = 0; i < b2_maxShapesPerBody; ++i){
			if (shapes[i] == null){
				shapes[i] = shape;
				break;
			}
		}
	}
}

// A rigid body. Internal computation are done in terms
// of the center of mass position. The center of mass may
// be offset from the body's origin.
class b2Body{
	// Set the position of the body's origin and rotation (radians).
	// This breaks any contacts and wakes the other bodies.
	public void SetOriginPosition(b2Vec2 position, float rotation){
		m_rotation = rotation;
		m_R.Set(m_rotation);
		m_position = b2Math.add(position, b2Mul(m_R, m_center));
		
		for (b2Shape s = m_shapeList; s != null; s = s.m_next){
			s.m_position = b2Math.add(m_position, b2Mul(m_R, s.m_localPosition));
			s.m_rotation = m_rotation + s.m_localRotation;
			s.m_R.Set(s.m_rotation);
			s.UpdateProxy();
		}
		
		m_world.m_broadPhase.Flush();
	}
	
	// Get the position of the body's origin. The body's origin does not
	// necessarily coincide with the center of mass. It depends on how the
	// shapes are created.
	public b2Vec2 GetOriginPosition(){
		return b2Math.subtract(m_position,b2Mul(m_R, m_center));
	}

	// Set the position of the body's center of mass and rotation (radians).
	// This breaks any contacts and wakes the other bodies.
	public void SetCenterPosition(b2Vec2 position, float rotation){
		m_rotation = rotation;
		m_R.Set(m_rotation);
		m_position.set(position);
		
		for (b2Shape s = m_shapeList; s != null; s = s.m_next){
			s.m_position = b2Math.add(m_position, b2Mul(m_R, s.m_localPosition));
			s.m_rotation = m_rotation + s.m_localRotation;
			s.m_R.Set(s.m_rotation);
			s.UpdateProxy();
		}
		
		m_world.m_broadPhase.Flush();
	}
	
	// Get the position of the body's center of mass. The body's center of mass
	// does not necessarily coincide with the body's origin. It depends on how the
	// shapes are created.
	public b2Vec2 GetCenterPosition(){
		return m_position;
	}

	// Get the rotation in radians.
	public float32 GetRotation(){
		return m_rotation;
	}

	public b2Mat22 GetRotationMatrix(){
		return m_R;
	}

	// Set/Get the linear velocity of the center of mass.
	public void SetLinearVelocity(b2Vec2 v){
		m_linearVelocity = v;
	}

	public b2Vec2 GetLinearVelocity(){
		return m_linearVelocity;
	}

	// Set/Get the angular velocity.
	public void SetAngularVelocity(float32 w){
		m_angularVelocity = w;
	}
	
	public float32 GetAngularVelocity(){
		return m_angularVelocity;
	}

	// Apply a force at a world point. Additive.
	public void ApplyForce(b2Vec2 force, b2Vec2 point){
		if (m_isSleeping == false){
			m_force.add(force);
			m_torque += b2Math.b2Cross( b2Math.subtract(point,m_position), force);
		}
	}

	// Apply a torque. Additive.
	public void ApplyTorque(float32 torque){
		if (m_isSleeping == false){
			m_torque += torque;
		}
	}

	// Apply an impulse at a point. This immediately modifies the velocity.
	public void ApplyImpulse(b2Vec2 impulse, b2Vec2 point){
		if (m_isSleeping == false){
			m_linearVelocity.add( b2Math.multiply(m_invMass,impulse));
			m_angularVelocity += m_invI * b2Math.b2Cross( b2Math.subtract(point, m_position), impulse);
		}
	}

	public float32 GetMass(){
		return m_mass;
	}
	
	public float32 GetInertia(){
		return m_I;
	}

	// Get the world coordinates of a point give the local coordinates
	// relative to the body's center of mass.
	public b2Vec2 GetWorldPoint(b2Vec2 localPoint){
		return b2Math.add(m_position,b2Math.b2Mul(m_R, localPoint));
	}
	
	// Get the world coordinates of a vector given the local coordinates.
	public b2Vec2 GetWorldVector(b2Vec2 localVector){
		return b2Mul(m_R, localVector);
	}
	
	
	// Returns a local point relative to the center of mass given a world point.
	public b2Vec2 GetLocalPoint(b2Vec2 worldPoint){
		return b2MulT(m_R, b2Math.subtract(worldPoint, m_position));
	}
	
	// Returns a local vector given a world vector.
	public b2Vec2 GetLocalVector(b2Vec2 worldVector){
		return b2MulT(m_R, worldVector);
	}
	
	// Is this body static (immovable)?
	public boolean IsStatic(){
		return m_invMass == 0.0f;
	}

	// Is this body sleeping (not simulating).
	public boolean IsSleeping(){
		return m_isSleeping;
	}
	
	// You can disable sleeping on this particular body.
	public void AllowSleeping(boolean flag){
		m_allowSleep = flag;
		if (flag == false){
			WakeUp();
		}
	}

	// Wake up this body so it will begin simulating.
	public void WakeUp(){
		m_isSleeping = false;
		m_sleepTime = 0.0f;
	}

	// Get the list of all shapes attached to this body.
	public b2Shape GetShapeList(){
		return m_shapeList;
	}

	// Get the next body in the world's body list.
	public b2Body GetNext(){
		return m_next;
	}
	

	//Java note: this definition was omitted in C++ version,
	//let's see if we can get away with leaving it out...
	//void* GetUserData();
	


	//--------------- Internals Below -------------------

	public b2Body(b2BodyDef bd, b2World world){
		m_position = new b2Vec2(bd.position);
		m_rotation = bd.rotation;
		m_R.Set(m_rotation);
		m_world = world;
		
		m_force = new b2Vec2(0.0f, 0.0f);
		m_torque = 0.0f;
		
		m_mass = 0.0f;
		
		b2MassData[] massDatas = new b2MassData[b2_maxShapesPerBody]; //not inited here - see below
		
		// Compute the shape mass properties, the bodies total mass and COM.
		m_center = new b2Vec2(0.0f, 0.0f);
		for (int32 i = 0; i < b2_maxShapesPerBody; ++i){
			b2ShapeDef sd = bd.shapes[i];
			if (sd == null) break;
			
			massDatas[i] = new b2MassData();
			b2MassData massData = massDatas[i];
			sd.ComputeMass(massData); //check, it might be better to just use massDatas[i] directly
			m_mass += massData.mass;
			m_center.add( b2Math.multiply(massData.mass, b2Math.add(sd.localPosition, massData.center) ) );
		}
		
		// Compute center of mass, and shift the origin to the COM.
		if (m_mass > 0.0f){
			m_center.multiply(1.0f / m_mass);
			m_position.add(b2Mul(m_R, m_center));
		}
		
		// Compute the moment of inertia.
		m_I = 0.0f;
		for (int32 i = 0; i < b2_maxShapesPerBody; ++i){
			b2ShapeDef sd = bd.shapes[i];
			if (sd == null) break;
			b2MassData massData = massDatas[i];
			//Use parallel axis thm to get contribution to cm inertia
			m_I += massData.I;
			b2Vec2 r = b2Math.add(sd.localPosition, b2Math.subtract(massData.center, m_center) );
			m_I += massData.mass * b2Dot(r, r);
		}
		
		if (m_mass > 0.0f){
			m_invMass = 1.0f / m_mass;
		} else{
			m_invMass = 0.0f;
		}
		
		if (m_I > 0.0f){
			m_invI = 1.0f / m_I;
		} else{
			m_invI = 0.0f;
		}
		
		// Compute the center of mass velocity.
		m_linearVelocity = b2Math.add(bd.linearVelocity, b2Cross(bd.angularVelocity, m_center));
		m_angularVelocity = bd.angularVelocity;
		
		m_jointList = null;
		m_contactList = null;
		m_prev = null;
		m_next = null;
		
		// Create the shapes.
		m_shapeList = null;
		for (int32 i = 0; i < b2_maxShapesPerBody; ++i){
			b2ShapeDef sd = bd.shapes[i];
			if (sd == null) break;
			b2Shape shape = b2Shape.Create(sd, this, m_center);
			shape.m_next = m_shapeList;
			m_shapeList = shape;
		}
		
		m_sleepTime = 0.0f;
		m_allowSleep = bd.allowSleep;
		m_isSleeping = bd.isSleeping;
		if (m_isSleeping == true || m_invMass == 0.0f){
			m_linearVelocity.Set(0.0f, 0.0f);
			m_angularVelocity = 0.0f;
		}
		
		//Maybe this should be copied instead of referenced?
		//In C++ version the set was to bd->userData, so the
		//pointer was dereferenced, but I forget what happens
		//to a void* upon assignment...
		m_userData = bd.userData;
	}
	
	public void b2BodyDestructor(){
		b2Shape s = m_shapeList;
		while (s != null){
			b2Shape s0 = s;
			s = s.m_next;
			b2Shape.Destroy(s0);
		}
	}

	public void SynchronizeShapes(){
		for (b2Shape s = m_shapeList; s != null; s = s.m_next){
			s.m_position = b2Math.add(m_position, b2Mul(m_R, s.m_localPosition));
			s.m_rotation = m_rotation + s.m_localRotation;
			s.m_R.Set(s.m_rotation);
			s.UpdateProxy();
		}
	}

	public boolean IsConnected(b2Body other){
		for (b2JointNode jn = m_jointList; jn != null; jn = jn.next){
			if (jn.other == other)
				return jn.joint.m_collideConnected;
		}
		return false;
	}

	public b2Vec2 m_position;	// center of mass position
	public float32 m_rotation;
	public b2Mat22 m_R;

	public b2Vec2 m_linearVelocity;
	public float32 m_angularVelocity;

	public b2Vec2 m_force;
	public float32 m_torque;

	public b2Vec2 m_center;	// local vector from client origin to center of mass

	public b2World m_world;
	public b2Body m_prev;
	public b2Body m_next;

	public b2Shape m_shapeList;

	public b2JointNode m_jointList;
	public b2ContactNode m_contactList;

	public float32 m_mass, m_invMass;
	public float32 m_I, m_invI;

	public float32 m_sleepTime;
	
	public bool m_allowSleep;
	public bool m_isSleeping;

	public bool m_islandFlag;

	public Object m_userData; //was void*
}



//#endif
