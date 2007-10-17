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

//#include "b2RevoluteJoint.h"
//#include "../../Dynamics/b2Body.h"

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Motor constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

class b2RevoluteJointDef extends b2JointDef{
	public b2RevoluteJointDef(){
		type = e_revoluteJoint;
		anchorPoint.Set(0.0f, 0.0f);
		lowerAngle = 0.0f;
		upperAngle = 0.0f;
		motorTorque = 0.0f;
		motorSpeed = 0.0f;
		enableLimit = false;
		enableMotor = false;
	}
	public b2Vec2 anchorPoint;
	public float lowerAngle;
	public float upperAngle;
	public float motorTorque;
	public float motorSpeed;
	public boolean enableLimit;
	public boolean enableMotor;
}

class b2RevoluteJoint extends b2Joint{
	
	b2Vec2 GetAnchor1();
	b2Vec2 GetAnchor2();
	float GetJointAngle();
	float GetJointSpeed();
	float GetMotorTorque(float invTimeStep);
	
	//--------------- Internals Below -------------------
	
	public b2RevoluteJoint(b2RevoluteJointDef def){
			super(def);
			m_localAnchor1 = b2MulT(m_body1.m_R, def.anchorPoint - m_body1.m_position);
			m_localAnchor2 = b2MulT(m_body2.m_R, def.anchorPoint - m_body2.m_position);
			
			m_intialAngle = m_body2.m_rotation - m_body1.m_rotation;
			
			m_ptpImpulse.Set(0.0f, 0.0f);
			m_motorImpulse = 0.0f;
			m_limitImpulse = 0.0f;
			m_limitPositionImpulse = 0.0f;
			
			m_lowerAngle = def.lowerAngle;
			m_upperAngle = def.upperAngle;
			m_maxMotorTorque = def.motorTorque;
			m_motorSpeed = def.motorSpeed;
			m_enableLimit = def.enableLimit;
			m_enableMotor = def.enableMotor;
	}
	
	public void PreSolve(){
		b2Body b1 = m_body1;
		b2Body b2 = m_body2;
		
		// Compute the effective mass matrix.
		b2Vec2 r1 = b2Mul(b1.m_R, m_localAnchor1);
		b2Vec2 r2 = b2Mul(b2.m_R, m_localAnchor2);
		
		// K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
		//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
		//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
		float invMass1 = b1.m_invMass, invMass2 = b2.m_invMass;
		float invI1 = b1.m_invI, invI2 = b2.m_invI;
		
		b2Mat22 K1; //Java note: I think we'll need to "new" these into existence before using them
		K1.col1.x = invMass1 + invMass2;	K1.col2.x = 0.0f;
		K1.col1.y = 0.0f;					K1.col2.y = invMass1 + invMass2;
		
		b2Mat22 K2;
		K2.col1.x =  invI1 * r1.y * r1.y;	K2.col2.x = -invI1 * r1.x * r1.y;
		K2.col1.y = -invI1 * r1.x * r1.y;	K2.col2.y =  invI1 * r1.x * r1.x;
		
		b2Mat22 K3;
		K3.col1.x =  invI2 * r2.y * r2.y;	K3.col2.x = -invI2 * r2.x * r2.y;
		K3.col1.y = -invI2 * r2.x * r2.y;	K3.col2.y =  invI2 * r2.x * r2.x;
		
		b2Mat22 K = K1 + K2 + K3;
		m_ptpMass = K.Invert();
		
		m_motorMass = 1.0f / (invI1 + invI2);
		
		if (m_enableMotor == false){
			m_motorImpulse = 0.0f;
		}
		
		if (m_enableLimit){
			float jointAngle = b2.m_rotation - b1.m_rotation - m_intialAngle;
			if (b2Abs(m_upperAngle - m_lowerAngle) < 2.0f * b2_angularSlop){
				m_limitState = e_equalLimits;
			}
			else if (jointAngle <= m_lowerAngle){
				if (m_limitState != e_atLowerLimit)	{
					m_limitImpulse = 0.0f;
				}
				m_limitState = e_atLowerLimit;
			}
			else if (jointAngle >= m_upperAngle){
				if (m_limitState != e_atUpperLimit)	{
					m_limitImpulse = 0.0f;
				}
				m_limitState = e_atUpperLimit;
			}
			else{
				m_limitState = e_inactiveLimit;
				m_limitImpulse = 0.0f;
			}
		}
		else{
			m_limitImpulse = 0.0f;
		}
		
		// Warm starting.
		b1.m_linearVelocity -= invMass1 * m_ptpImpulse;
		b1.m_angularVelocity -= invI1 * (b2Cross(r1, m_ptpImpulse) + m_motorImpulse + m_limitImpulse);
		
		b2.m_linearVelocity += invMass2 * m_ptpImpulse;
		b2.m_angularVelocity += invI2 * (b2Cross(r2, m_ptpImpulse) + m_motorImpulse + m_limitImpulse);
		
		m_limitPositionImpulse = 0.0f;
	}
	
	
	void SolveVelocityConstraints(float dt);
	boolean SolvePositionConstraints();
	
	b2Vec2 m_localAnchor1;
	b2Vec2 m_localAnchor2;
	b2Vec2 m_ptpImpulse;
	float m_motorImpulse;
	float m_limitImpulse;
	float m_limitPositionImpulse;
	
	b2Mat22 m_ptpMass;		// effective mass for point-to-point constraint.
	float m_motorMass;	// effective mass for motor/limit angular constraint.
	float m_intialAngle;
	float m_lowerAngle;
	float m_upperAngle;
	float m_maxMotorTorque;
	float m_motorSpeed;
	
	boolean m_enableLimit;
	boolean m_enableMotor;
	b2LimitState m_limitState;
}


void b2RevoluteJoint::SolveVelocityConstraints(float dt)
{
	b2Body* b1 = m_body1;
	b2Body* b2 = m_body2;

	b2Vec2 r1 = b2Mul(b1.m_R, m_localAnchor1);
	b2Vec2 r2 = b2Mul(b2.m_R, m_localAnchor2);

	// Solve point-to-point constraint
	b2Vec2 ptpCdot = b2.m_linearVelocity + b2Cross(b2.m_angularVelocity, r2) - b1.m_linearVelocity - b2Cross(b1.m_angularVelocity, r1);
	b2Vec2 ptpImpulse = -b2Mul(m_ptpMass, ptpCdot);
	m_ptpImpulse += ptpImpulse;

	b1.m_linearVelocity -= b1.m_invMass * ptpImpulse;
	b1.m_angularVelocity -= b1.m_invI * b2Cross(r1, ptpImpulse);

	b2.m_linearVelocity += b2.m_invMass * ptpImpulse;
	b2.m_angularVelocity += b2.m_invI * b2Cross(r2, ptpImpulse);

	if (m_enableMotor && m_limitState != e_equalLimits)
	{
		float motorCdot = b2.m_angularVelocity - b1.m_angularVelocity - m_motorSpeed;
		float motorImpulse = -m_motorMass * motorCdot;
		float oldMotorImpulse = m_motorImpulse;
		m_motorImpulse = b2Clamp(m_motorImpulse + motorImpulse, -dt * m_maxMotorTorque, dt * m_maxMotorTorque);
		motorImpulse = m_motorImpulse - oldMotorImpulse;
		b1.m_angularVelocity -= b1.m_invI * motorImpulse;
		b2.m_angularVelocity += b2.m_invI * motorImpulse;
	}

	if (m_enableLimit && m_limitState != e_inactiveLimit)
	{
		float limitCdot = b2.m_angularVelocity - b1.m_angularVelocity;
		float limitImpulse = -m_motorMass * limitCdot;

		if (m_limitState == e_equalLimits)
		{
			m_limitImpulse += limitImpulse;
		}
		else if (m_limitState == e_atLowerLimit)
		{
			float oldLimitImpulse = m_limitImpulse;
			m_limitImpulse = b2Max(m_limitImpulse + limitImpulse, 0.0f);
			limitImpulse = m_limitImpulse - oldLimitImpulse;
		}
		else if (m_limitState == e_atUpperLimit)
		{
			float oldLimitImpulse = m_limitImpulse;
			m_limitImpulse = b2Min(m_limitImpulse + limitImpulse, 0.0f);
			limitImpulse = m_limitImpulse - oldLimitImpulse;
		}

		b1.m_angularVelocity -= b1.m_invI * limitImpulse;
		b2.m_angularVelocity += b2.m_invI * limitImpulse;
	}
}

boolean b2RevoluteJoint::SolvePositionConstraints()
{
	b2Body* b1 = m_body1;
	b2Body* b2 = m_body2;

	// Solve point-to-point position error.
	b2Vec2 r1 = b2Mul(b1.m_R, m_localAnchor1);
	b2Vec2 r2 = b2Mul(b2.m_R, m_localAnchor2);

	b2Vec2 p1 = b1.m_position + r1;
	b2Vec2 p2 = b2.m_position + r2;
	b2Vec2 ptpC = p2 - p1;

	// Prevent overly large corrections.
	b2Vec2 dpMax = b2Vec2::Make(b2_maxLinearCorrection, b2_maxLinearCorrection);
	ptpC = b2Clamp(ptpC, -dpMax, dpMax);

	b2Vec2 impulse = -b2Mul(m_ptpMass, ptpC);

	b1.m_position -= b1.m_invMass * impulse;
	b1.m_rotation -= b1.m_invI * b2Cross(r1, impulse);
	b1.m_R.Set(b1.m_rotation);

	b2.m_position += b2.m_invMass * impulse;
	b2.m_rotation += b2.m_invI * b2Cross(r2, impulse);
	b2.m_R.Set(b2.m_rotation);

	float positionError = ptpC.Length();

	// Handle limits.
	float angularError = 0.0f;

	if (m_enableLimit && m_limitState != e_inactiveLimit)
	{
		float angle = b2.m_rotation - b1.m_rotation - m_intialAngle;
		float limitImpulse = 0.0f;

		if (m_limitState == e_equalLimits)
		{
			// Prevent large angular corrections
			float limitC = b2Clamp(angle, -b2_maxAngularCorrection, b2_maxAngularCorrection);
			limitImpulse = -m_motorMass * limitC;
			angularError = b2Abs(limitC);
		}
		else if (m_limitState == e_atLowerLimit)
		{
			// Prevent large angular corrections
			float limitC = b2Clamp(angle - m_lowerAngle, -b2_maxAngularCorrection, b2_maxAngularCorrection);
			limitImpulse = -m_motorMass * (limitC + b2_angularSlop);
			float oldLimitImpulse = m_limitPositionImpulse;
			m_limitPositionImpulse = b2Max(m_limitPositionImpulse + limitImpulse, 0.0f);
			limitImpulse = m_limitPositionImpulse - oldLimitImpulse;
			angularError = b2Max(0.0f, -limitC);
		}
		else if (m_limitState == e_atUpperLimit)
		{
			// Prevent large angular corrections
			float limitC = b2Clamp(angle - m_upperAngle, -b2_maxAngularCorrection, b2_maxAngularCorrection);
			limitImpulse = -m_motorMass * (limitC - b2_angularSlop);
			float oldLimitImpulse = m_limitPositionImpulse;
			m_limitPositionImpulse = b2Min(m_limitPositionImpulse + limitImpulse, 0.0f);
			limitImpulse = m_limitPositionImpulse - oldLimitImpulse;
			angularError = b2Max(0.0f, limitC);
		}

		b1.m_rotation -= b1.m_invI * limitImpulse;
		b1.m_R.Set(b1.m_rotation);
		b2.m_rotation += b2.m_invI * limitImpulse;
		b2.m_R.Set(b2.m_rotation);
	}

	return positionError <= b2_linearSlop && angularError <= b2_angularSlop;
}

b2Vec2 b2RevoluteJoint::GetAnchor1() const
{
	b2Body* b1 = m_body1;
	return b1.m_position + b2Mul(b1.m_R, m_localAnchor1);
}

b2Vec2 b2RevoluteJoint::GetAnchor2() const
{
	b2Body* b2 = m_body2;
	return b2.m_position + b2Mul(b2.m_R, m_localAnchor2);
}

float b2RevoluteJoint::GetJointAngle() const
{
	b2Body* b1 = m_body1;
	b2Body* b2 = m_body2;
	return b2.m_rotation - b1.m_rotation;
}

float b2RevoluteJoint::GetJointSpeed() const
{
	b2Body* b1 = m_body1;
	b2Body* b2 = m_body2;
	return b2.m_angularVelocity - b1.m_angularVelocity;
}

float b2RevoluteJoint::GetMotorTorque(float invTimeStep) const
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

//#ifndef B2_REVOLUTE_JOINT_H
//#define B2_REVOLUTE_JOINT_H

//#include "b2Joint.h"



//#endif


