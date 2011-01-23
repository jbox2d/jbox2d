/*******************************************************************************
 * Copyright (c) 2011, Daniel Murphy
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL DANIEL MURPHY BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
package org.jbox2d.dynamics.joints;

import org.jbox2d.common.Mat22;
import org.jbox2d.common.Mat33;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.Vec3;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.TimeStep;
import org.jbox2d.pooling.PoolingStack.PoolContainer;
import org.jbox2d.pooling.WorldPool;

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
	
	public float m_motorMass; // effective mass for motor/limit translational constraint.
	public float m_motorImpulse;
	
	public float m_lowerTranslation;
	public float m_upperTranslation;
	public float m_maxMotorForce;
	public float m_motorSpeed;
	
	public boolean m_enableLimit;
	public boolean m_enableMotor;
	public LimitState m_limitState;
	
	public PrismaticJoint(WorldPool argWorld, PrismaticJointDef def) {
		super(argWorld, def);
		m_localAnchor1 = new Vec2(def.localAnchorA);
		m_localAnchor2 = new Vec2(def.localAnchorB);
		m_localXAxis1 = new Vec2(def.localAxis1);
		m_localYAxis1 = new Vec2();
		Vec2.crossToOut(1f, m_localXAxis1, m_localYAxis1);
		m_refAngle = def.referenceAngle;
		
		m_impulse = new Vec3();
		m_motorMass = 0.0f;
		m_motorImpulse = 0.0f;
		
		m_lowerTranslation = def.lowerTranslation;
		m_upperTranslation = def.upperTranslation;
		m_maxMotorForce = def.maxMotorForce;
		m_motorSpeed = def.motorSpeed;
		m_enableLimit = def.enableLimit;
		m_enableMotor = def.enableMotor;
		m_limitState = LimitState.INACTIVE;
		
		m_K = new Mat33();
		m_axis = new Vec2();
		m_perp = new Vec2();
	}
	
	@Override
	public void getAnchorA(Vec2 argOut) {
		m_bodyA.getWorldPointToOut(m_localAnchor1, argOut);
	}
	
	@Override
	public void getAnchorB(Vec2 argOut) {
		m_bodyB.getWorldPointToOut(m_localAnchor2, argOut);
	}
	
	@Override
	public void getReactionForce(float inv_dt, Vec2 argOut) {
		Vec2 temp = pool.popVec2();
		temp.set(m_axis).mulLocal(m_motorImpulse + m_impulse.z);
		argOut.set(m_perp).mulLocal(m_impulse.x).addLocal(temp).mulLocal(inv_dt);
		pool.pushVec2(1);
	}
	
	@Override
	public float getReactionTorque(float inv_dt) {
		return inv_dt * m_impulse.y;
	}
	
	// / Get the current joint translation, usually in meters.
	public float getJointTranslation() {
		Body b1 = m_bodyA;
		Body b2 = m_bodyB;
		
		Vec2 p1 = pool.popVec2();
		Vec2 p2 = pool.popVec2();
		Vec2 axis = pool.popVec2();
		
		b1.getWorldPointToOut(m_localAnchor1, p1);
		b2.getWorldPointToOut(m_localAnchor2, p2);
		p2.subLocal(p1);
		b1.getWorldVectorToOut(m_localXAxis1, axis);
		
		float translation = Vec2.dot(p2, axis);
		
		pool.pushVec2(3);
		return translation;
	}
	
	// / Get the current joint translation speed, usually in meters per second.
	public float getJointSpeed() {
		Body b1 = m_bodyA;
		Body b2 = m_bodyB;
		
		PoolContainer<Vec2> pc = pool.popVec2(9);
		Vec2 temp = pc.p0;
		Vec2 r1 = pc.p1;
		Vec2 r2 = pc.p2;
		Vec2 p1 = pc.p3;
		Vec2 p2 = pc.p4;
		Vec2 d = pc.p5;
		Vec2 axis = pc.p6;
		Vec2 temp2 = pc.p7;
		Vec2 temp3 = pc.p8;
		
		temp.set(m_localAnchor1).subLocal(b1.getLocalCenter());
		Mat22.mulToOut(b1.getTransform().R, temp, r1);
		
		temp.set(m_localAnchor2).subLocal(b2.getLocalCenter());
		Mat22.mulToOut(b2.getTransform().R, temp, r2);
		
		p1.set(b1.m_sweep.c).addLocal(r1);
		p2.set(b2.m_sweep.c).addLocal(r2);
		
		d.set(p2).subLocal(p1);
		b1.getWorldVectorToOut(m_localXAxis1, axis);
		
		Vec2 v1 = b1.m_linearVelocity;
		Vec2 v2 = b2.m_linearVelocity;
		float w1 = b1.m_angularVelocity;
		float w2 = b2.m_angularVelocity;
		
		
		Vec2.crossToOut(w1, axis, temp);
		Vec2.crossToOut(w2, r2, temp2);
		Vec2.crossToOut(w1, r1, temp3);
		
		temp2.addLocal(v2).subLocal(v1).subLocal(temp3);
		float speed = Vec2.dot(d, temp) + Vec2.dot(axis, temp2);
		
		pool.pushVec2(9);
		
		return speed;
	}
	
	// / Is the joint limit enabled?
	public boolean isLimitEnabled() {
		return m_enableLimit;
	}
	
	// / Enable/disable the joint limit.
	public void enableLimit(boolean flag) {
		m_bodyA.setAwake(true);
		m_bodyB.setAwake(true);
		m_enableLimit = flag;
	}
	
	// / Get the lower joint limit, usually in meters.
	public float getLowerLimit() {
		return m_lowerTranslation;
	}
	
	// / Get the upper joint limit, usually in meters.
	public float getUpperLimit() {
		return m_upperTranslation;
	}
	
	// / Set the joint limits, usually in meters.
	public void setLimits(float lower, float upper) {
		assert (lower <= upper);
		m_bodyA.setAwake(true);
		m_bodyB.setAwake(true);
		m_lowerTranslation = lower;
		m_upperTranslation = upper;
	}
	
	// / Is the joint motor enabled?
	public boolean isMotorEnabled() {
		return m_enableMotor;
	}
	
	// / Enable/disable the joint motor.
	public void enableMotor(boolean flag) {
		m_bodyA.setAwake(true);
		m_bodyB.setAwake(true);
		m_enableMotor = flag;
	}
	
	// / Set the motor speed, usually in meters per second.
	public void setMotorSpeed(float speed) {
		m_bodyA.setAwake(true);
		m_bodyB.setAwake(true);
		m_motorSpeed = speed;
	}
	
	// / Get the motor speed, usually in meters per second.
	public float getMotorSpeed() {
		return m_motorSpeed;
	}
	
	// / Set the maximum motor force, usually in N.
	public void setMaxMotorForce(float force) {
		m_bodyA.setAwake(true);
		m_bodyB.setAwake(true);
		m_maxMotorForce = force;
	}
	
	// / Get the current motor force, usually in N.
	public float getMotorForce() {
		return m_motorImpulse;
	}
	
	@Override
	public void initVelocityConstraints(TimeStep step) {
		Body b1 = m_bodyA;
		Body b2 = m_bodyB;
		
		m_localCenterA.set(b1.getLocalCenter());
		m_localCenterB.set(b2.getLocalCenter());
		
		Transform xf1 = b1.getTransform();
		Transform xf2 = b2.getTransform();
		
		// Compute the effective masses.
		
		final Vec2 temp = pool.popVec2();
		final Vec2 r1 = pool.popVec2();
		final Vec2 r2 = pool.popVec2();
		final Vec2 d = pool.popVec2();
		
		r1.set(m_localAnchor1).subLocal(m_localCenterA);
		r2.set(m_localAnchor2).subLocal(m_localCenterB);
		Mat22.mulToOut(xf1.R, r1, r1);
		Mat22.mulToOut(xf2.R, r2, r2);
		
		d.set(b2.m_sweep.c).addLocal(r2).subLocal(b1.m_sweep.c).subLocal(r1);
		
		m_invMassA = b1.m_invMass;
		m_invIA = b1.m_invI;
		m_invMassB = b2.m_invMass;
		m_invIB = b2.m_invI;
		
		// Compute motor Jacobian and effective mass.
		{
			Mat22.mulToOut(xf1.R, m_localXAxis1, m_axis);
			temp.set(d).addLocal(r1);
			m_a1 = Vec2.cross(temp, m_axis);
			m_a2 = Vec2.cross(r2, m_axis);
			
			m_motorMass = m_invMassA + m_invMassB + m_invIA * m_a1 * m_a1 + m_invIB * m_a2 * m_a2;
			if (m_motorMass > Settings.EPSILON) {
				m_motorMass = 1.0f / m_motorMass;
			}
		}
		
		// Prismatic constraint.
		{
			Mat22.mulToOut(xf1.R, m_localYAxis1, m_perp);
			
			temp.set(d).addLocal(r1);
			m_s1 = Vec2.cross(temp, m_perp);
			m_s2 = Vec2.cross(r2, m_perp);
			
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
		if (m_enableLimit) {
			
			float jointTranslation = Vec2.dot(m_axis, d);
			if (MathUtils.abs(m_upperTranslation - m_lowerTranslation) < 2.0f * Settings.linearSlop) {
				m_limitState = LimitState.EQUAL;
			}
			else if (jointTranslation <= m_lowerTranslation) {
				if (m_limitState != LimitState.AT_LOWER) {
					m_limitState = LimitState.AT_LOWER;
					m_impulse.z = 0.0f;
				}
			}
			else if (jointTranslation >= m_upperTranslation) {
				if (m_limitState != LimitState.AT_UPPER) {
					m_limitState = LimitState.AT_UPPER;
					m_impulse.z = 0.0f;
				}
			}
			else {
				m_limitState = LimitState.INACTIVE;
				m_impulse.z = 0.0f;
			}
		}
		else {
			m_limitState = LimitState.INACTIVE;
			m_impulse.z = 0.0f;
		}
		
		if (m_enableMotor == false) {
			m_motorImpulse = 0.0f;
		}
		
		if (step.warmStarting) {
			// Account for variable time step.
			m_impulse.mulLocal(step.dtRatio);
			m_motorImpulse *= step.dtRatio;
			
			final Vec2 P = pool.popVec2();
			temp.set(m_axis).mulLocal(m_motorImpulse + m_impulse.z);
			P.set(m_perp).mulLocal(m_impulse.x).addLocal(temp);
			
			float L1 = m_impulse.x * m_s1 + m_impulse.y + (m_motorImpulse + m_impulse.z) * m_a1;
			float L2 = m_impulse.x * m_s2 + m_impulse.y + (m_motorImpulse + m_impulse.z) * m_a2;
			
			temp.set(P).mulLocal(m_invMassA);
			b1.m_linearVelocity.subLocal(temp);
			b1.m_angularVelocity -= m_invIA * L1;
			
			temp.set(P).mulLocal(m_invMassB);
			b2.m_linearVelocity.addLocal(temp);
			b2.m_angularVelocity += m_invIB * L2;
			
			pool.pushVec2(1);
		}
		else {
			m_impulse.setZero();
			m_motorImpulse = 0.0f;
		}
		pool.pushVec2(4);
	}
	
	@Override
	public boolean solvePositionConstraints(float baumgarte) {
		
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
		
		final Mat22 R1 = pool.popMat22();
		final Mat22 R2 = pool.popMat22();
		R1.set(a1);
		R2.set(a2);
		
		final Vec2 temp = pool.popVec2();
		final Vec2 r1 = pool.popVec2();
		final Vec2 r2 = pool.popVec2();
		final Vec2 d = pool.popVec2();
		
		r1.set(m_localAnchor1).subLocal(m_localCenterA);
		r2.set(m_localAnchor2).subLocal(m_localCenterB);
		Mat22.mulToOut(R1, r1, r1);
		Mat22.mulToOut(R2, r2, r2);
		d.set(c2).addLocal(r2).subLocal(c1).subLocal(r1);
		
		if (m_enableLimit) {
			Mat22.mulToOut(R1, m_localXAxis1, m_axis);
			
			temp.set(d).addLocal(r1);
			m_a1 = Vec2.cross(temp, m_axis);
			m_a2 = Vec2.cross(r2, m_axis);
			
			float translation = Vec2.dot(m_axis, d);
			if (MathUtils.abs(m_upperTranslation - m_lowerTranslation) < 2.0f * Settings.linearSlop) {
				// Prevent large angular corrections
				C2 = MathUtils.clamp(translation, -Settings.maxLinearCorrection, Settings.maxLinearCorrection);
				linearError = MathUtils.abs(translation);
				active = true;
			}
			else if (translation <= m_lowerTranslation) {
				// Prevent large linear corrections and allow some slop.
				C2 = MathUtils.clamp(translation - m_lowerTranslation + Settings.linearSlop,
						-Settings.maxLinearCorrection, 0.0f);
				linearError = m_lowerTranslation - translation;
				active = true;
			}
			else if (translation >= m_upperTranslation) {
				// Prevent large linear corrections and allow some slop.
				C2 = MathUtils.clamp(translation - m_upperTranslation - Settings.linearSlop, 0.0f,
						Settings.maxLinearCorrection);
				linearError = translation - m_upperTranslation;
				active = true;
			}
		}
		
		Mat22.mulToOut(R1, m_localYAxis1, m_perp);
		
		temp.set(d).addLocal(r1);
		m_s1 = Vec2.cross(temp, m_perp);
		m_s2 = Vec2.cross(r2, m_perp);
		
		final Vec3 impulse = pool.popVec3();
		final Vec2 C1 = pool.popVec2();
		
		C1.x = Vec2.dot(m_perp, d);
		C1.y = a2 - a1 - m_refAngle;
		
		linearError = MathUtils.max(linearError, MathUtils.abs(C1.x));
		angularError = MathUtils.abs(C1.y);
		
		if (active) {
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
			
			final Vec3 C = pool.popVec3();
			C.x = C1.x;
			C.y = C1.y;
			C.z = C2;
			
			m_K.solve33ToOut(C.negateLocal(), impulse);
			pool.pushVec3(1);
		}
		else {
			float m1 = m_invMassA, m2 = m_invMassB;
			float i1 = m_invIA, i2 = m_invIB;
			
			float k11 = m1 + m2 + i1 * m_s1 * m_s1 + i2 * m_s2 * m_s2;
			float k12 = i1 * m_s1 + i2 * m_s2;
			float k22 = i1 + i2;
			
			m_K.col1.set(k11, k12, 0.0f);
			m_K.col2.set(k12, k22, 0.0f);
			
			m_K.solve22ToOut(C1.negateLocal(), temp);
			C1.negateLocal();
			
			impulse.x = temp.x;
			impulse.y = temp.y;
			impulse.z = 0.0f;
		}
		
		final Vec2 P = pool.popVec2();
		temp.set(m_perp).mulLocal(impulse.x);
		P.set(m_axis).mulLocal(impulse.z).addLocal(temp);
		float L1 = impulse.x * m_s1 + impulse.y + impulse.z * m_a1;
		float L2 = impulse.x * m_s2 + impulse.y + impulse.z * m_a2;
		
		temp.set(P).mulLocal(m_invMassA);
		c1.subLocal(temp);
		a1 -= m_invIA * L1;
		
		temp.set(P).mulLocal(m_invMassB);
		c2.addLocal(temp);
		a2 += m_invIB * L2;
		
		// TODO_ERIN remove need for this.
		b1.m_sweep.c.set(c1);
		b1.m_sweep.a = a1;

		b2.m_sweep.c.set(c2);
		b2.m_sweep.a = a2;
		b1.synchronizeTransform();
		b2.synchronizeTransform();
		
		pool.pushVec2(6);
		pool.pushVec3(1);
		pool.pushMat22(2);
		
		return linearError <= Settings.linearSlop && angularError <= Settings.angularSlop;
	}
	
	@Override
	public void solveVelocityConstraints(TimeStep step) {
		Body b1 = m_bodyA;
		Body b2 = m_bodyB;
		
		Vec2 v1 = b1.m_linearVelocity;
		float w1 = b1.m_angularVelocity;
		Vec2 v2 = b2.m_linearVelocity;
		float w2 = b2.m_angularVelocity;
		
		final Vec2 temp = pool.popVec2();
		
		// Solve linear motor constraint.
		if (m_enableMotor && m_limitState != LimitState.EQUAL) {
			temp.set(v2).subLocal(v1);
			float Cdot = Vec2.dot(m_axis, temp) + m_a2 * w2 - m_a1 * w1;
			float impulse = m_motorMass * (m_motorSpeed - Cdot);
			float oldImpulse = m_motorImpulse;
			float maxImpulse = step.dt * m_maxMotorForce;
			m_motorImpulse = MathUtils.clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
			impulse = m_motorImpulse - oldImpulse;
			
			final Vec2 P = pool.popVec2();
			P.set(m_axis).mulLocal(impulse);
			float L1 = impulse * m_a1;
			float L2 = impulse * m_a2;
			
			temp.set(P).mulLocal(m_invMassA);
			v1.subLocal(temp);
			w1 -= m_invIA * L1;
			
			temp.set(P).mulLocal(m_invMassB);
			v2.addLocal(temp);
			w2 += m_invIB * L2;
			
			pool.pushVec2(1);
		}
		
		final Vec2 Cdot1 = pool.popVec2();
		temp.set(v2).subLocal(v1);
		Cdot1.x = Vec2.dot(m_perp, temp) + m_s2 * w2 - m_s1 * w1;
		Cdot1.y = w2 - w1;
		//System.out.println(Cdot1);
		
		if (m_enableLimit && m_limitState != LimitState.INACTIVE) {
			// Solve prismatic and limit constraint in block form.
			float Cdot2;
			temp.set(v2).subLocal(v1);
			Cdot2 = Vec2.dot(m_axis, temp) + m_a2 * w2 - m_a1 * w1;
			
			final Vec3 Cdot = pool.popVec3();
			Cdot.set(Cdot1.x, Cdot1.y, Cdot2);
			Cdot.negateLocal();
			
			final Vec3 f1 = pool.popVec3();
			f1.set(m_impulse);
			
			final Vec3 df = pool.popVec3();
			m_K.solve33ToOut(Cdot, df);
			m_impulse.addLocal(df);
			
			if (m_limitState == LimitState.AT_LOWER) {
				m_impulse.z = MathUtils.max(m_impulse.z, 0.0f);
			}
			else if (m_limitState == LimitState.AT_UPPER) {
				m_impulse.z = MathUtils.min(m_impulse.z, 0.0f);
			}
			
			// f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) +
			// f1(1:2)
			final Vec2 b = pool.popVec2();
			final Vec2 f2r = pool.popVec2();

			temp.set(m_K.col3.x, m_K.col3.y).mulLocal(m_impulse.z - f1.z);
			b.set(Cdot1).negateLocal().subLocal(temp);
			
			temp.set(f1.x, f1.y);
			m_K.solve22ToOut(b, f2r);
			f2r.addLocal(temp);
			m_impulse.x = f2r.x;
			m_impulse.y = f2r.y;
			
			df.set(m_impulse).subLocal(f1);
			
			final Vec2 P = pool.popVec2();
			temp.set(m_axis).mulLocal(df.z);
			P.set(m_perp).mulLocal(df.x).addLocal(temp);
			
			float L1 = df.x * m_s1 + df.y + df.z * m_a1;
			float L2 = df.x * m_s2 + df.y + df.z * m_a2;
			
			temp.set(P).mulLocal(m_invMassA);
			v1.subLocal(temp);
			w1 -= m_invIA * L1;
			
			temp.set(P).mulLocal(m_invMassB);
			v2.addLocal(temp);
			w2 += m_invIB * L2;
			
			pool.pushVec2(3);
			pool.pushVec3(3);
		}
		else {
			// Limit is inactive, just solve the prismatic constraint in block form.
			final Vec2 df = pool.popVec2();
			m_K.solve22ToOut(Cdot1.negateLocal(), df);
			Cdot1.negateLocal();
			
			m_impulse.x += df.x;
			m_impulse.y += df.y;
			
			final Vec2 P = pool.popVec2();
			P.set(m_perp).mulLocal(df.x);
			float L1 = df.x * m_s1 + df.y;
			float L2 = df.x * m_s2 + df.y;
			
			temp.set(P).mulLocal(m_invMassA);
			v1.subLocal(temp);
			w1 -= m_invIA * L1;
			
			temp.set(P).mulLocal(m_invMassB);
			v2.addLocal(temp);
			w2 += m_invIB * L2;
			
			pool.pushVec2(2);
		}
		
		b1.m_linearVelocity.set(v1);
		b1.m_angularVelocity = w1;
		b2.m_linearVelocity.set(v2);
		b2.m_angularVelocity = w2;
		
		pool.pushVec2(2);
	}
}
