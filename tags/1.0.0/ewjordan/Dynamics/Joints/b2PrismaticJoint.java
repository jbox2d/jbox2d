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

#include "b2PrismaticJoint.h"
#include "../../Dynamics/b2Body.h"

// Linear constraint (point-to-line)
// d = p2 - p1 = x2 + r2 - x1 - r1
// C = dot(ay1, d)
// Cdot = dot(d, cross(w1, ay1)) + dot(ay1, v2 + cross(w2, r2) - v1 - cross(w1, r1))
//      = -dot(ay1, v1) - dot(cross(d + r1, ay1), w1) + dot(ay1, v2) + dot(cross(r2, ay1), v2)
// J = [-ay1 -cross(d+r1,ay1) ay1 cross(r2,ay1)]
//
// Angular constraint
// C = a2 - a1 + a_initial
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]

// Motor/Limit linear constraint
// C = dot(ax1, d)
// Cdot = = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
// J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]

b2PrismaticJoint::b2PrismaticJoint(const b2PrismaticJointDef* def)
: b2Joint(def)
{
	m_localAnchor1 = b2MulT(m_body1->m_R, def->anchorPoint - m_body1->m_position);
	m_localAnchor2 = b2MulT(m_body2->m_R, def->anchorPoint - m_body2->m_position);
	m_localXAxis1 = b2MulT(m_body1->m_R, def->axis);
	m_localYAxis1 = b2Cross(1.0f, m_localXAxis1);
	m_initialAngle = m_body2->m_rotation - m_body1->m_rotation;

	m_linearJacobian.SetZero();
	m_linearMass = 0.0f;
	m_linearImpulse = 0.0f;

	m_angularMass = 0.0f;
	m_angularImpulse = 0.0f;

	m_motorJacobian.SetZero();
	m_motorMass = 0.0;
	m_motorImpulse = 0.0f;
	m_limitImpulse = 0.0f;
	m_limitPositionImpulse = 0.0f;

	m_lowerTranslation = def->lowerTranslation;
	m_upperTranslation = def->upperTranslation;
	m_maxMotorForce = def->motorForce;
	m_motorSpeed = def->motorSpeed;
	m_enableLimit = def->enableLimit;
	m_enableMotor = def->enableMotor;
}

void b2PrismaticJoint::PreSolve()
{
	b2Body* b1 = m_body1;
	b2Body* b2 = m_body2;

	// Compute the effective masses.
	b2Vec2 r1 = b2Mul(b1->m_R, m_localAnchor1);
	b2Vec2 r2 = b2Mul(b2->m_R, m_localAnchor2);

	float32 invMass1 = b1->m_invMass, invMass2 = b2->m_invMass;
	float32 invI1 = b1->m_invI, invI2 = b2->m_invI;

	// Compute point to line constraint effective mass.
	// J = [-ay1 -cross(d+r1,ay1) ay1 cross(r2,ay1)]
	b2Vec2 ay1 = b2Mul(b1->m_R, m_localYAxis1);
	b2Vec2 e = b2->m_position + r2 - b1->m_position;

	m_linearJacobian.Set(-ay1, -b2Cross(e, ay1), ay1, b2Cross(r2, ay1));
	m_linearMass =	invMass1 + invI1 * m_linearJacobian.angular1 * m_linearJacobian.angular1 +
					invMass2 + invI2 * m_linearJacobian.angular2 * m_linearJacobian.angular2;
	b2Assert(m_linearMass > FLT_EPSILON);
	m_linearMass = 1.0f / m_linearMass;

	// Compute angular constraint effective mass.
	m_angularMass = 1.0f / (invI1 + invI2);

	// Compute motor and limit terms.
	if (m_enableLimit || m_enableMotor)
	{
		// The motor and limit share a Jacobian and effective mass.
		b2Vec2 ax1 = b2Mul(b1->m_R, m_localXAxis1);
		m_motorJacobian.Set(-ax1, -b2Cross(e, ax1), ax1, b2Cross(r2, ax1));
		m_motorMass =	invMass1 + invI1 * m_motorJacobian.angular1 * m_motorJacobian.angular1 +
						invMass2 + invI2 * m_motorJacobian.angular2 * m_motorJacobian.angular2;
		b2Assert(m_motorMass > FLT_EPSILON);
		m_motorMass = 1.0f / m_motorMass;

		if (m_enableLimit)
		{
			b2Vec2 d = e - r1;	// p2 - p1
			float32 jointTranslation = b2Dot(ax1, d);
			if (b2Abs(m_upperTranslation - m_lowerTranslation) < 2.0f * b2_linearSlop)
			{
				m_limitState = e_equalLimits;
			}
			else if (jointTranslation <= m_lowerTranslation)
			{
				if (m_limitState != e_atLowerLimit)
				{
					m_limitImpulse = 0.0f;
				}
				m_limitState = e_atLowerLimit;
			}
			else if (jointTranslation >= m_upperTranslation)
			{
				if (m_limitState != e_atUpperLimit)
				{
					m_limitImpulse = 0.0f;
				}
				m_limitState = e_atUpperLimit;
			}
			else
			{
				m_limitState = e_inactiveLimit;
				m_limitImpulse = 0.0f;
			}
		}
	}

	if (m_enableMotor == false)
	{
		m_motorImpulse = 0.0f;
	}

	if (m_enableLimit == false)
	{
		m_limitImpulse = 0.0f;
	}

	// Warm starting.
	b2Vec2 P1 = m_linearImpulse * m_linearJacobian.linear1 + (m_motorImpulse + m_limitImpulse) * m_motorJacobian.linear1;
	b2Vec2 P2 = m_linearImpulse * m_linearJacobian.linear2 + (m_motorImpulse + m_limitImpulse) * m_motorJacobian.linear2;
	float32 L1 = m_linearImpulse * m_linearJacobian.angular1 - m_angularImpulse + (m_motorImpulse + m_limitImpulse) * m_motorJacobian.angular1;
	float32 L2 = m_linearImpulse * m_linearJacobian.angular2 + m_angularImpulse + (m_motorImpulse + m_limitImpulse) * m_motorJacobian.angular2;

	b1->m_linearVelocity += invMass1 * P1;
	b1->m_angularVelocity += invI1 * L1;

	b2->m_linearVelocity += invMass2 * P2;
	b2->m_angularVelocity += invI2 * L2;

	m_limitPositionImpulse = 0.0f;
}

void b2PrismaticJoint::SolveVelocityConstraints(float32 dt)
{
	b2Body* b1 = m_body1;
	b2Body* b2 = m_body2;

	float32 invMass1 = b1->m_invMass, invMass2 = b2->m_invMass;
	float32 invI1 = b1->m_invI, invI2 = b2->m_invI;

	// Solve linear constraint.
	float32 linearCdot = m_linearJacobian.Compute(b1->m_linearVelocity, b1->m_angularVelocity, b2->m_linearVelocity, b2->m_angularVelocity);
	float32 linearImpulse = -m_linearMass * linearCdot;
	m_linearImpulse += linearImpulse;

	b1->m_linearVelocity += (invMass1 * linearImpulse) * m_linearJacobian.linear1;
	b1->m_angularVelocity += invI1 * linearImpulse * m_linearJacobian.angular1;

	b2->m_linearVelocity += (invMass2 * linearImpulse) * m_linearJacobian.linear2;
	b2->m_angularVelocity += invI2 * linearImpulse * m_linearJacobian.angular2;

	// Solve angular constraint.
	float32 angularCdot = b2->m_angularVelocity - b1->m_angularVelocity;
	float32 angularImpulse = -m_angularMass * angularCdot;
	m_angularImpulse += angularImpulse;

	b1->m_angularVelocity -= invI1 * angularImpulse;
	b2->m_angularVelocity += invI2 * angularImpulse;

	// Solve linear motor constraint.
	if (m_enableMotor && m_limitState != e_equalLimits)
	{
		float32 motorCdot = m_motorJacobian.Compute(b1->m_linearVelocity, b1->m_angularVelocity, b2->m_linearVelocity, b2->m_angularVelocity) - m_motorSpeed;
		float32 motorImpulse = -m_motorMass * motorCdot;
		float32 oldMotorImpulse = m_motorImpulse;
		m_motorImpulse = b2Clamp(m_motorImpulse + motorImpulse, -dt * m_maxMotorForce, dt * m_maxMotorForce);
		motorImpulse = m_motorImpulse - oldMotorImpulse;

		b1->m_linearVelocity += (invMass1 * motorImpulse) * m_motorJacobian.linear1;
		b1->m_angularVelocity += invI1 * motorImpulse * m_motorJacobian.angular1;

		b2->m_linearVelocity += (invMass2 * motorImpulse) * m_motorJacobian.linear2;
		b2->m_angularVelocity += invI2 * motorImpulse * m_motorJacobian.angular2;
	}

	// Solve linear limit constraint.
	if (m_enableLimit && m_limitState != e_inactiveLimit)
	{
		float32 limitCdot = m_motorJacobian.Compute(b1->m_linearVelocity, b1->m_angularVelocity, b2->m_linearVelocity, b2->m_angularVelocity);
		float32 limitImpulse = -m_motorMass * limitCdot;

		if (m_limitState == e_equalLimits)
		{
			m_limitImpulse += limitImpulse;
		}
		else if (m_limitState == e_atLowerLimit)
		{
			float32 oldLimitImpulse = m_limitImpulse;
			m_limitImpulse = b2Max(m_limitImpulse + limitImpulse, 0.0f);
			limitImpulse = m_limitImpulse - oldLimitImpulse;
		}
		else if (m_limitState == e_atUpperLimit)
		{
			float32 oldLimitImpulse = m_limitImpulse;
			m_limitImpulse = b2Min(m_limitImpulse + limitImpulse, 0.0f);
			limitImpulse = m_limitImpulse - oldLimitImpulse;
		}

		b1->m_linearVelocity += (invMass1 * limitImpulse) * m_motorJacobian.linear1;
		b1->m_angularVelocity += invI1 * limitImpulse * m_motorJacobian.angular1;

		b2->m_linearVelocity += (invMass2 * limitImpulse) * m_motorJacobian.linear2;
		b2->m_angularVelocity += invI2 * limitImpulse * m_motorJacobian.angular2;
	}
}

bool b2PrismaticJoint::SolvePositionConstraints()
{
	b2Body* b1 = m_body1;
	b2Body* b2 = m_body2;

	float32 invMass1 = b1->m_invMass, invMass2 = b2->m_invMass;
	float32 invI1 = b1->m_invI, invI2 = b2->m_invI;

	b2Vec2 r1 = b2Mul(b1->m_R, m_localAnchor1);
	b2Vec2 r2 = b2Mul(b2->m_R, m_localAnchor2);
	b2Vec2 p1 = b1->m_position + r1;
	b2Vec2 p2 = b2->m_position + r2;
	b2Vec2 d = p2 - p1;
	b2Vec2 ay1 = b2Mul(b1->m_R, m_localYAxis1);

	// Solve linear (point-to-line) constraint.
	float32 linearC = b2Dot(ay1, d);
	// Prevent overly large corrections.
	linearC = b2Clamp(linearC, -b2_maxLinearCorrection, b2_maxLinearCorrection);
	float32 linearImpulse = -m_linearMass * linearC;

	b1->m_position += (invMass1 * linearImpulse) * m_linearJacobian.linear1;
	b1->m_rotation += invI1 * linearImpulse * m_linearJacobian.angular1;
	//b1->m_R.Set(b1->m_rotation); // updated by angular constraint
	b2->m_position += (invMass2 * linearImpulse) * m_linearJacobian.linear2;
	b2->m_rotation += invI2 * linearImpulse * m_linearJacobian.angular2;
	//b2->m_R.Set(b2->m_rotation); // updated by angular constraint

	float32 positionError = b2Abs(linearC);

	// Solve angular constraint.
	float32 angularC = b2->m_rotation - b1->m_rotation - m_initialAngle;
	// Prevent overly large corrections.
	angularC = b2Clamp(angularC, -b2_maxAngularCorrection, b2_maxAngularCorrection);
	float32 angularImpulse = -m_angularMass * angularC;

	b1->m_rotation -= b1->m_invI * angularImpulse;
	b1->m_R.Set(b1->m_rotation);
	b2->m_rotation += b2->m_invI * angularImpulse;
	b2->m_R.Set(b2->m_rotation);

	float32 angularError = b2Abs(angularC);

	// Solve linear limit constraint.
	if (m_enableLimit && m_limitState != e_inactiveLimit)
	{
		b2Vec2 r1 = b2Mul(b1->m_R, m_localAnchor1);
		b2Vec2 r2 = b2Mul(b2->m_R, m_localAnchor2);
		b2Vec2 p1 = b1->m_position + r1;
		b2Vec2 p2 = b2->m_position + r2;
		b2Vec2 d = p2 - p1;
		b2Vec2 ax1 = b2Mul(b1->m_R, m_localXAxis1);

		float32 translation = b2Dot(ax1, d);
		float32 limitImpulse = 0.0f;

		if (m_limitState == e_equalLimits)
		{
			// Prevent large angular corrections
			float32 limitC = b2Clamp(translation, -b2_maxLinearCorrection, b2_maxLinearCorrection);
			limitImpulse = -m_motorMass * limitC;
			positionError = b2Max(positionError, b2Abs(angularC));
		}
		else if (m_limitState == e_atLowerLimit)
		{
			// Prevent large angular corrections
			float32 limitC = b2Clamp(translation - m_lowerTranslation, -b2_maxLinearCorrection, b2_maxLinearCorrection);
			limitImpulse = -m_motorMass * (limitC + b2_linearSlop);
			float32 oldLimitImpulse = m_limitPositionImpulse;
			m_limitPositionImpulse = b2Max(m_limitPositionImpulse + limitImpulse, 0.0f);
			limitImpulse = m_limitPositionImpulse - oldLimitImpulse;
			positionError = b2Max(positionError, -limitC);
		}
		else if (m_limitState == e_atUpperLimit)
		{
			// Prevent large angular corrections
			float32 limitC = b2Clamp(translation - m_upperTranslation, -b2_maxLinearCorrection, b2_maxLinearCorrection);
			limitImpulse = -m_motorMass * (limitC - b2_linearSlop);
			float32 oldLimitImpulse = m_limitPositionImpulse;
			m_limitPositionImpulse = b2Min(m_limitPositionImpulse + limitImpulse, 0.0f);
			limitImpulse = m_limitPositionImpulse - oldLimitImpulse;
			positionError = b2Max(positionError, limitC);
		}

		b1->m_position += (invMass1 * limitImpulse) * m_motorJacobian.linear1;
		b1->m_rotation += invI1 * limitImpulse * m_motorJacobian.angular1;
		b1->m_R.Set(b1->m_rotation);
		b2->m_position += (invMass2 * limitImpulse) * m_motorJacobian.linear2;
		b2->m_rotation += invI2 * limitImpulse * m_motorJacobian.angular2;
		b2->m_R.Set(b2->m_rotation);
	}

	return positionError <= b2_linearSlop && angularError <= b2_angularSlop;
}

b2Vec2 b2PrismaticJoint::GetAnchor1() const
{
	b2Body* b1 = m_body1;
	return b1->m_position + b2Mul(b1->m_R, m_localAnchor1);
}

b2Vec2 b2PrismaticJoint::GetAnchor2() const
{
	b2Body* b2 = m_body2;
	return b2->m_position + b2Mul(b2->m_R, m_localAnchor2);
}

float32 b2PrismaticJoint::GetJointTranslation() const
{
	b2Body* b1 = m_body1;
	b2Body* b2 = m_body2;

	b2Vec2 r1 = b2Mul(b1->m_R, m_localAnchor1);
	b2Vec2 r2 = b2Mul(b2->m_R, m_localAnchor2);
	b2Vec2 p1 = b1->m_position + r1;
	b2Vec2 p2 = b2->m_position + r2;
	b2Vec2 d = p2 - p1;
	b2Vec2 ax1 = b2Mul(b1->m_R, m_localXAxis1);

	float32 translation = b2Dot(ax1, d);
	return translation;
}

float32 b2PrismaticJoint::GetJointSpeed() const
{
	b2Body* b1 = m_body1;
	b2Body* b2 = m_body2;

	b2Vec2 r1 = b2Mul(b1->m_R, m_localAnchor1);
	b2Vec2 r2 = b2Mul(b2->m_R, m_localAnchor2);
	b2Vec2 p1 = b1->m_position + r1;
	b2Vec2 p2 = b2->m_position + r2;
	b2Vec2 d = p2 - p1;
	b2Vec2 ax1 = b2Mul(b1->m_R, m_localXAxis1);

	b2Vec2 v1 = b1->m_linearVelocity;
	b2Vec2 v2 = b2->m_linearVelocity;
	float32 w1 = b1->m_angularVelocity;
	float32 w2 = b2->m_angularVelocity;

	float32 speed = b2Dot(d, b2Cross(w1, ax1)) + b2Dot(ax1, v2 + b2Cross(w2, r2) - v1 - b2Cross(w1, r1));
	return speed;
}

float32 b2PrismaticJoint::GetMotorForce(float32 invTimeStep) const
{
	return m_motorImpulse * invTimeStep;
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

#ifndef B2_PRISMATIC_JOINT_H
#define B2_PRISMATIC_JOINT_H

#include "b2Joint.h"

struct b2PrismaticJointDef : public b2JointDef
{
	b2PrismaticJointDef()
	{
		type = e_prismaticJoint;
		anchorPoint.Set(0.0f, 0.0f);
		axis.Set(1.0f, 0.0f);
		lowerTranslation = 0.0f;
		upperTranslation = 0.0f;
		motorForce = 0.0f;
		motorSpeed = 0.0f;
		enableLimit = false;
		enableMotor = false;
	}

	b2Vec2 anchorPoint;
	b2Vec2 axis;
	float32 lowerTranslation;
	float32 upperTranslation;
	float32 motorForce;
	float32 motorSpeed;
	bool enableLimit;
	bool enableMotor;
};

struct b2PrismaticJoint : public b2Joint
{
	b2Vec2 GetAnchor1() const;
	b2Vec2 GetAnchor2() const;
	float32 GetJointTranslation() const;
	float32 GetJointSpeed() const;
	float32 GetMotorForce(float32 invTimeStep) const;

	//--------------- Internals Below -------------------

	b2PrismaticJoint(const b2PrismaticJointDef* def);

	void PreSolve();
	void SolveVelocityConstraints(float32 dt);
	bool SolvePositionConstraints();

	b2Vec2 m_localAnchor1;
	b2Vec2 m_localAnchor2;
	b2Vec2 m_localXAxis1;
	b2Vec2 m_localYAxis1;
	float32 m_initialAngle;

	b2Jacobian m_linearJacobian;
	float32 m_linearMass;				// effective mass for point-to-line constraint.
	float32 m_linearImpulse;
	
	float32 m_angularMass;			// effective mass for angular constraint.
	float32 m_angularImpulse;

	b2Jacobian m_motorJacobian;
	float32 m_motorMass;			// effective mass for motor/limit translational constraint.
	float32 m_motorImpulse;
	float32 m_limitImpulse;
	float32 m_limitPositionImpulse;

	float32 m_lowerTranslation;
	float32 m_upperTranslation;
	float32 m_maxMotorForce;
	float32 m_motorSpeed;
	
	bool m_enableLimit;
	bool m_enableMotor;
	b2LimitState m_limitState;
};

#endif
