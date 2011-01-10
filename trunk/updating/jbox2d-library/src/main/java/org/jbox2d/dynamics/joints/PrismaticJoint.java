package org.jbox2d.dynamics.joints;

import org.jbox2d.common.Mat33;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.Vec3;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.TimeStep;
import org.jbox2d.structs.dynamics.joints.LimitState;

public class PrismaticJoint extends Joint {

	public final Vec2 m_localAnchor1;
	public final Vec2 m_localAnchor2;
	public final Vec2 m_localXAxis1;
	public final Vec2 m_localYAxis1;
	public float m_refAngle;

	public final Vec2 m_axis, m_perp;
	public float m_s1, m_s2;
	public float m_a1, m_a2;

	public final Mat33 m_K;
	public final Vec3 m_impulse;

	public float m_motorMass;			// effective mass for motor/limit translational constraint.
	public float m_motorImpulse;

	public float m_lowerTranslation;
	public float m_upperTranslation;
	public float m_maxMotorForce;
	public float m_motorSpeed;
	
	public boolean m_enableLimit;
	public boolean m_enableMotor;
	public LimitState m_limitState;
	
	public PrismaticJoint(PrismaticJointDef def){
		super(def);
		m_localAnchor1 = new Vec2(def.localAnchorA);
		m_localAnchor2 = new Vec2(def.localAnchorB);
		m_localXAxis1 = new Vec2(def.localAxis1);
		m_localYAxis1 = new Vec2();
		Vec2.crossToOut(1f, m_localXAxis1, m_localYAxis1);
		m_refAngle = def.referenceAngle;

		m_impulse.setZero();
		m_motorMass = 0.0f;
		m_motorImpulse = 0.0f;

		m_lowerTranslation = def.lowerTranslation;
		m_upperTranslation = def.upperTranslation;
		m_maxMotorForce = def.maxMotorForce;
		m_motorSpeed = def.motorSpeed;
		m_enableLimit = def.enableLimit;
		m_enableMotor = def.enableMotor;
		m_limitState = LimitState.INACTIVE;

		m_axis.setZero();
		m_perp.setZero();
	}
	
	private final Vec2 anchorA = new Vec2();
	@Override
	public Vec2 getAnchorA() {
		m_bodyA.getWorldPointToOut(m_localAnchor1, anchorA);
		return anchorA;
	}

	private final Vec2 anchorB = new Vec2();
	@Override
	public Vec2 getAnchorB() {
		m_bodyB.getWorldPointToOut(m_localAnchor2, anchorB);
		return anchorB;
	}

	@Override
	public Vec2 getReactionForce(float inv_dt) {
		return inv_dt * (m_impulse.x * m_perp + (m_motorImpulse + m_impulse.z) * m_axis);
	}

	@Override
	public float getReactionTorque(float inv_dt) {
		return inv_dt * m_impulse.y;
	}
	
	/// Get the current joint translation, usually in meters.
	public float getJointTranslation(){
		Body b1 = m_bodyA;
		Body b2 = m_bodyB;

		Vec2 p1 = b1.getWorldPoint(m_localAnchor1);
		Vec2 p2 = b2.getWorldPoint(m_localAnchor2);
		Vec2 d = p2 - p1;
		Vec2 axis = b1.getWorldVector(m_localXAxis1);

		float translation = b2Dot(d, axis);
		return translation;
	}

	/// Get the current joint translation speed, usually in meters per second.
	public float getJointSpeed(){
		Body b1 = m_bodyA;
		Body b2 = m_bodyB;

		Vec2 r1 = Mul(b1.getTransform().R, m_localAnchor1 - b1.getLocalCenter());
		Vec2 r2 = Mul(b2.getTransform().R, m_localAnchor2 - b2.getLocalCenter());
		Vec2 p1 = b1.m_sweep.c + r1;
		Vec2 p2 = b2.m_sweep.c + r2;
		Vec2 d = p2 - p1;
		Vec2 axis = b1.getWorldVector(m_localXAxis1);

		Vec2 v1 = b1.m_linearVelocity;
		Vec2 v2 = b2.m_linearVelocity;
		float w1 = b1.m_angularVelocity;
		float w2 = b2.m_angularVelocity;

		float speed = b2Dot(d, b2Cross(w1, axis)) + b2Dot(axis, v2 + b2Cross(w2, r2) - v1 - b2Cross(w1, r1));
		return speed;
	}

	/// Is the joint limit enabled?
	public boolean isLimitEnabled() const;

	/// Enable/disable the joint limit.
	void EnableLimit(boolean flag){
		m_bodyA.setAwake(true);
		m_bodyB.setAwake(true);
		m_enableLimit = flag;
	}

	/// Get the lower joint limit, usually in meters.
	public float getLowerLimit() const;

	/// Get the upper joint limit, usually in meters.
	public float getUpperLimit() const;

	/// Set the joint limits, usually in meters.
	public void setLimits(float lower, float upper){
		assert(lower <= upper);
		m_bodyA.setAwake(true);
		m_bodyB.setAwake(true);
		m_lowerTranslation = lower;
		m_upperTranslation = upper;
	}

	/// Is the joint motor enabled?
	public boolean isMotorEnabled() const;

	/// Enable/disable the joint motor.
	public void enableMotor(boolean flag){
		m_bodyA.setAwake(true);
		m_bodyB.setAwake(true);
		m_enableMotor = flag;
	}

	/// Set the motor speed, usually in meters per second.
	public void setMotorSpeed(float speed){
		m_bodyA.setAwake(true);
		m_bodyB.setAwake(true);
		m_motorSpeed = speed;
	}

	/// Get the motor speed, usually in meters per second.
	public float getMotorSpeed() const;

	/// Set the maximum motor force, usually in N.
	public void setMaxMotorForce(float force){
		m_bodyA.setAwake(true);
		m_bodyB.setAwake(true);
		m_maxMotorForce = force;
	}

	/// Get the current motor force, usually in N.
	public float getMotorForce() const;


	@Override
	public void initVelocityConstraints(TimeStep step) {
		Body b1 = m_bodyA;
		Body b2 = m_bodyB;

		m_localCenterA = b1.getLocalCenter();
		m_localCenterB = b2.getLocalCenter();

		b2Transform xf1 = b1.getTransform();
		b2Transform xf2 = b2.getTransform();

		// Compute the effective masses.
		Vec2 r1 = Mul(xf1.R, m_localAnchor1 - m_localCenterA);
		Vec2 r2 = Mul(xf2.R, m_localAnchor2 - m_localCenterB);
		Vec2 d = b2.m_sweep.c + r2 - b1.m_sweep.c - r1;

		m_invMassA = b1.m_invMass;
		m_invIA = b1.m_invI;
		m_invMassB = b2.m_invMass;
		m_invIB = b2.m_invI;

		// Compute motor Jacobian and effective mass.
		{
			m_axis = Mul(xf1.R, m_localXAxis1);
			m_a1 = b2Cross(d + r1, m_axis);
			m_a2 = b2Cross(r2, m_axis);

			m_motorMass = m_invMassA + m_invMassB + m_invIA * m_a1 * m_a1 + m_invIB * m_a2 * m_a2;
			if (m_motorMass > b2_epsilon)
			{
				m_motorMass = 1.0f / m_motorMass;
			}
		}

		// Prismatic constraint.
		{
			m_perp = Mul(xf1.R, m_localYAxis1);

			m_s1 = b2Cross(d + r1, m_perp);
			m_s2 = b2Cross(r2, m_perp);

			float m1 = m_invMassA, m2 = m_invMassB;
			float i1 = m_invIA, i2 = m_invIB;

			float k11 = m1 + m2 + i1 * m_s1 * m_s1 + i2 * m_s2 * m_s2;
			float k12 = i1 * m_s1 + i2 * m_s2;
			float k13 = i1 * m_s1 * m_a1 + i2 * m_s2 * m_a2;
			float k22 = i1 + i2;
			float k23 = i1 * m_a1 + i2 * m_a2;
			float k33 = m1 + m2 + i1 * m_a1 * m_a1 + i2 * m_a2 * m_a2;

			m_K.col1.set(k11, k12, k13);
			m_K.col2.set(k12, k22, k23);
			m_K.col3.set(k13, k23, k33);
		}

		// Compute motor and limit terms.
		if (m_enableLimit)
		{
			float jointTranslation = b2Dot(m_axis, d);
			if (abs(m_upperTranslation - m_lowerTranslation) < 2.0f * b2_linearSlop)
			{
				m_limitState = e_equalLimits;
			}
			else if (jointTranslation <= m_lowerTranslation)
			{
				if (m_limitState != e_atLowerLimit)
				{
					m_limitState = e_atLowerLimit;
					m_impulse.z = 0.0f;
				}
			}
			else if (jointTranslation >= m_upperTranslation)
			{
				if (m_limitState != e_atUpperLimit)
				{
					m_limitState = e_atUpperLimit;
					m_impulse.z = 0.0f;
				}
			}
			else
			{
				m_limitState = e_inactiveLimit;
				m_impulse.z = 0.0f;
			}
		}
		else
		{
			m_limitState = e_inactiveLimit;
			m_impulse.z = 0.0f;
		}

		if (m_enableMotor == false)
		{
			m_motorImpulse = 0.0f;
		}

		if (step.warmStarting)
		{
			// Account for variable time step.
			m_impulse *= step.dtRatio;
			m_motorImpulse *= step.dtRatio;

			Vec2 P = m_impulse.x * m_perp + (m_motorImpulse + m_impulse.z) * m_axis;
			float L1 = m_impulse.x * m_s1 + m_impulse.y + (m_motorImpulse + m_impulse.z) * m_a1;
			float L2 = m_impulse.x * m_s2 + m_impulse.y + (m_motorImpulse + m_impulse.z) * m_a2;

			b1.m_linearVelocity -= m_invMassA * P;
			b1.m_angularVelocity -= m_invIA * L1;

			b2.m_linearVelocity += m_invMassB * P;
			b2.m_angularVelocity += m_invIB * L2;
		}
		else
		{
			m_impulse.setZero();
			m_motorImpulse = 0.0f;
		}
	}

	@Override
	public boolean solvePositionConstraints(float baumgarte) {
		B2_NOT_USED(baumgarte);

		Body b1 = m_bodyA;
		Body b2 = m_bodyB;

		Vec2 c1 = b1.m_sweep.c;
		float a1 = b1.m_sweep.a;

		Vec2 c2 = b2.m_sweep.c;
		float a2 = b2.m_sweep.a;

		// Solve linear limit constraint.
		float linearError = 0.0f, angularError = 0.0f;
		boolean active = false;
		float C2 = 0.0f;

		Mat22 R1(a1), R2(a2);

		Vec2 r1 = Mul(R1, m_localAnchor1 - m_localCenterA);
		Vec2 r2 = Mul(R2, m_localAnchor2 - m_localCenterB);
		Vec2 d = c2 + r2 - c1 - r1;

		if (m_enableLimit)
		{
			m_axis = Mul(R1, m_localXAxis1);

			m_a1 = b2Cross(d + r1, m_axis);
			m_a2 = b2Cross(r2, m_axis);

			float translation = b2Dot(m_axis, d);
			if (abs(m_upperTranslation - m_lowerTranslation) < 2.0f * b2_linearSlop)
			{
				// Prevent large angular corrections
				C2 = b2Clamp(translation, -b2_maxLinearCorrection, b2_maxLinearCorrection);
				linearError = abs(translation);
				active = true;
			}
			else if (translation <= m_lowerTranslation)
			{
				// Prevent large linear corrections and allow some slop.
				C2 = b2Clamp(translation - m_lowerTranslation + b2_linearSlop, -b2_maxLinearCorrection, 0.0f);
				linearError = m_lowerTranslation - translation;
				active = true;
			}
			else if (translation >= m_upperTranslation)
			{
				// Prevent large linear corrections and allow some slop.
				C2 = b2Clamp(translation - m_upperTranslation - b2_linearSlop, 0.0f, b2_maxLinearCorrection);
				linearError = translation - m_upperTranslation;
				active = true;
			}
		}

		m_perp = Mul(R1, m_localYAxis1);

		m_s1 = b2Cross(d + r1, m_perp);
		m_s2 = b2Cross(r2, m_perp);

		Vec3 impulse;
		Vec2 C1;
		C1.x = b2Dot(m_perp, d);
		C1.y = a2 - a1 - m_refAngle;

		linearError = Max(linearError, abs(C1.x));
		angularError = abs(C1.y);

		if (active)
		{
			float m1 = m_invMassA, m2 = m_invMassB;
			float i1 = m_invIA, i2 = m_invIB;

			float k11 = m1 + m2 + i1 * m_s1 * m_s1 + i2 * m_s2 * m_s2;
			float k12 = i1 * m_s1 + i2 * m_s2;
			float k13 = i1 * m_s1 * m_a1 + i2 * m_s2 * m_a2;
			float k22 = i1 + i2;
			float k23 = i1 * m_a1 + i2 * m_a2;
			float k33 = m1 + m2 + i1 * m_a1 * m_a1 + i2 * m_a2 * m_a2;

			m_K.col1.set(k11, k12, k13);
			m_K.col2.set(k12, k22, k23);
			m_K.col3.set(k13, k23, k33);

			Vec3 C;
			C.x = C1.x;
			C.y = C1.y;
			C.z = C2;

			impulse = m_K.Solve33(-C);
		}
		else
		{
			float m1 = m_invMassA, m2 = m_invMassB;
			float i1 = m_invIA, i2 = m_invIB;

			float k11 = m1 + m2 + i1 * m_s1 * m_s1 + i2 * m_s2 * m_s2;
			float k12 = i1 * m_s1 + i2 * m_s2;
			float k22 = i1 + i2;

			m_K.col1.set(k11, k12, 0.0f);
			m_K.col2.set(k12, k22, 0.0f);

			Vec2 impulse1 = m_K.Solve22(-C1);
			impulse.x = impulse1.x;
			impulse.y = impulse1.y;
			impulse.z = 0.0f;
		}

		Vec2 P = impulse.x * m_perp + impulse.z * m_axis;
		float L1 = impulse.x * m_s1 + impulse.y + impulse.z * m_a1;
		float L2 = impulse.x * m_s2 + impulse.y + impulse.z * m_a2;

		c1 -= m_invMassA * P;
		a1 -= m_invIA * L1;
		c2 += m_invMassB * P;
		a2 += m_invIB * L2;

		// TODO_ERIN remove need for this.
		b1.m_sweep.c = c1;
		b1.m_sweep.a = a1;
		b2.m_sweep.c = c2;
		b2.m_sweep.a = a2;
		b1.SynchronizeTransform();
		b2.SynchronizeTransform();
		
		return linearError <= b2_linearSlop && angularError <= b2_angularSlop;
	}

	@Override
	public void solveVelocityConstraints(TimeStep step) {
		Body b1 = m_bodyA;
		Body b2 = m_bodyB;

		Vec2 v1 = b1.m_linearVelocity;
		float w1 = b1.m_angularVelocity;
		Vec2 v2 = b2.m_linearVelocity;
		float w2 = b2.m_angularVelocity;

		// Solve linear motor constraint.
		if (m_enableMotor && m_limitState != e_equalLimits)
		{
			float Cdot = b2Dot(m_axis, v2 - v1) + m_a2 * w2 - m_a1 * w1;
			float impulse = m_motorMass * (m_motorSpeed - Cdot);
			float oldImpulse = m_motorImpulse;
			float maxImpulse = step.dt * m_maxMotorForce;
			m_motorImpulse = b2Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
			impulse = m_motorImpulse - oldImpulse;

			Vec2 P = impulse * m_axis;
			float L1 = impulse * m_a1;
			float L2 = impulse * m_a2;

			v1 -= m_invMassA * P;
			w1 -= m_invIA * L1;

			v2 += m_invMassB * P;
			w2 += m_invIB * L2;
		}

		Vec2 Cdot1;
		Cdot1.x = b2Dot(m_perp, v2 - v1) + m_s2 * w2 - m_s1 * w1;
		Cdot1.y = w2 - w1;

		if (m_enableLimit && m_limitState != e_inactiveLimit)
		{
			// Solve prismatic and limit constraint in block form.
			float Cdot2;
			Cdot2 = b2Dot(m_axis, v2 - v1) + m_a2 * w2 - m_a1 * w1;
			Vec3 Cdot(Cdot1.x, Cdot1.y, Cdot2);

			Vec3 f1 = m_impulse;
			Vec3 df =  m_K.Solve33(-Cdot);
			m_impulse += df;

			if (m_limitState == e_atLowerLimit)
			{
				m_impulse.z = Max(m_impulse.z, 0.0f);
			}
			else if (m_limitState == e_atUpperLimit)
			{
				m_impulse.z = Min(m_impulse.z, 0.0f);
			}

			// f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
			Vec2 b = -Cdot1 - (m_impulse.z - f1.z) * Vec2(m_K.col3.x, m_K.col3.y);
			Vec2 f2r = m_K.Solve22(b) + Vec2(f1.x, f1.y);
			m_impulse.x = f2r.x;
			m_impulse.y = f2r.y;

			df = m_impulse - f1;

			Vec2 P = df.x * m_perp + df.z * m_axis;
			float L1 = df.x * m_s1 + df.y + df.z * m_a1;
			float L2 = df.x * m_s2 + df.y + df.z * m_a2;

			v1 -= m_invMassA * P;
			w1 -= m_invIA * L1;

			v2 += m_invMassB * P;
			w2 += m_invIB * L2;
		}
		else
		{
			// Limit is inactive, just solve the prismatic constraint in block form.
			Vec2 df = m_K.Solve22(-Cdot1);
			m_impulse.x += df.x;
			m_impulse.y += df.y;

			Vec2 P = df.x * m_perp;
			float L1 = df.x * m_s1 + df.y;
			float L2 = df.x * m_s2 + df.y;

			v1 -= m_invMassA * P;
			w1 -= m_invIA * L1;

			v2 += m_invMassB * P;
			w2 += m_invIB * L2;
		}

		b1.m_linearVelocity = v1;
		b1.m_angularVelocity = w1;
		b2.m_linearVelocity = v2;
		b2.m_angularVelocity = w2;
	}

}
