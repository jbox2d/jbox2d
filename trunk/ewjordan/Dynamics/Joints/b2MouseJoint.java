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

#include "b2MouseJoint.h"
#include "../../Dynamics/b2Body.h"

#include <stdio.h>

// p = attached point, m = mouse point
// C = norm(p - m) - L
// u = (p - m) / norm(p - m)
// Cdot = dot(u, v + cross(w, r))
//      = [u^T cross(r, u)^T] [v; w]
// K = J * invM * JT
//   = [u^T cross(r, u)^T][invMass  0][u]
//                        [0     invI][cross(r, u)]
//   = [u^T cross(r, u)^T][invMass*u]
//                        [invI * cross(r, u)]
//   = invMass + invI * cross(r, u) * cross(r, u)


b2MouseJoint::b2MouseJoint(const b2MouseJointDef* def)
: b2Joint(def)
{
	m_target = def->target;
	m_localAnchor = b2MulT(m_body2->m_R, m_target - m_body2->m_position);

	m_motorForce = def->motorForce;
	m_length = def->length;
	m_beta = def->beta;

	m_impulse = 0.0f;
}

void b2MouseJoint::SetTarget(const b2Vec2& target)
{
	m_body2->WakeUp();
	m_target = target;
}

void b2MouseJoint::PreSolve()
{
	b2Body* body = m_body2;

	// Compute the effective mass matrix.
	b2Vec2 r = b2Mul(body->m_R, m_localAnchor);
	m_u = body->m_position + r - m_target;

	// Handle singularity.
	float32 length = m_u.Length();
	if (length > FLT_EPSILON)
	{
		m_u *= 1.0f / length;
	}
	else
	{
		m_u.Set(0.0f, 1.0f);
	}

	m_positionError = length - m_length;

	float32 cru = b2Cross(r, m_u);
	m_mEff = body->m_invMass + body->m_invI * cru * cru;
	b2Assert(m_mEff > FLT_EPSILON);
	m_mEff = 1.0f / m_mEff;

	// Warm starting.
	b2Vec2 P = m_impulse * m_u;
	body->m_linearVelocity += body->m_invMass * P;
	body->m_angularVelocity += body->m_invI * b2Cross(r, P);
}

void b2MouseJoint::SolveVelocityConstraints(float32 dt)
{
	b2Body* body = m_body2;

	b2Vec2 r = b2Mul(body->m_R, m_localAnchor);

	// Cdot = dot(u, v + cross(w, r))
	float32 Cdot = b2Dot(m_u, body->m_linearVelocity + b2Cross(body->m_angularVelocity, r));
	float32 impulse = -m_mEff * (Cdot + m_beta / dt * m_positionError);

	float32 oldImpulse = m_impulse;
	m_impulse = b2Clamp(m_impulse + impulse, -dt * m_motorForce, 0.0f);
	impulse = m_impulse - oldImpulse;

	b2Vec2 P = impulse * m_u;
	body->m_linearVelocity += body->m_invMass * P;
	body->m_angularVelocity += body->m_invI * b2Cross(r, P);
}

b2Vec2 b2MouseJoint::GetAnchor1() const
{
	return m_target;
}

b2Vec2 b2MouseJoint::GetAnchor2() const
{
	return m_body2->m_position + b2Mul(m_body2->m_R, m_localAnchor);
}
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

#ifndef B2_MOUSE_JOINT_H
#define B2_MOUSE_JOINT_H

#include "b2Joint.h"

struct b2Body;
class b2BlockAllocator;

struct b2MouseJointDef : public b2JointDef
{
	b2MouseJointDef()
	{
		type = e_mouseJoint;
		target.Set(0.0f, 0.0f);
		motorForce = 0.0f;
		beta = 0.2f;
		length = 1.0f;
	}

	b2Vec2 target;
	float32 beta;
	float32 motorForce;
	float32 length;
};

struct b2MouseJoint : public b2Joint
{
	b2Vec2 GetAnchor1() const;
	b2Vec2 GetAnchor2() const;
	float32 GetMotorForce(float32 inv_dt) const { return m_impulse * inv_dt; }
	void SetTarget(const b2Vec2& target);

	//--------------- Internals Below -------------------

	b2MouseJoint(const b2MouseJointDef* def);

	void PreSolve();
	void SolveVelocityConstraints(float32 dt);
	bool SolvePositionConstraints() { return true; }

	b2Vec2 m_localAnchor;
	b2Vec2 m_target;
	b2Vec2 m_u;
	float32 m_positionError;
	float32 m_impulse;

	float32 m_mEff;		// effective mass
	float32 m_motorForce;
	float32 m_length;
	float32 m_beta;
};

#endif
