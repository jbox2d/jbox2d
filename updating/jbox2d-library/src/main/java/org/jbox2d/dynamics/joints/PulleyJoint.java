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
/**
 * Created at 12:12:02 PM Jan 23, 2011
 */
package org.jbox2d.dynamics.joints;

import org.jbox2d.common.Mat22;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.TimeStep;
import org.jbox2d.pooling.WorldPool;

/**
 * @author Daniel Murphy
 */
public class PulleyJoint extends Joint {
	
	public static final float MIN_PULLEY_LENGTH = 2.0f;
	
	private final Vec2 m_groundAnchor1 = new Vec2();
	private final Vec2 m_groundAnchor2 = new Vec2();
	private final Vec2 m_localAnchor1 = new Vec2();
	private final Vec2 m_localAnchor2 = new Vec2();
	
	private final Vec2 m_u1 = new Vec2();
	private final Vec2 m_u2 = new Vec2();
	
	private float m_constant;
	private float m_ratio;
	
	private float m_maxLength1;
	private float m_maxLength2;
	
	// Effective masses
	private float m_pulleyMass;
	private float m_limitMass1;
	private float m_limitMass2;
	
	// Impulses for accumulation/warm starting.
	private float m_impulse;
	private float m_limitImpulse1;
	private float m_limitImpulse2;
	
	private LimitState m_state;
	private LimitState m_limitState1;
	private LimitState m_limitState2;
	
	/**
	 * @param argWorldPool
	 * @param def
	 */
	public PulleyJoint(WorldPool argWorldPool, PulleyJointDef def) {
		super(argWorldPool, def);
		m_groundAnchor1.set(def.groundAnchorA);
		m_groundAnchor2.set(def.groundAnchorB);
		m_localAnchor1.set(def.localAnchorA);
		m_localAnchor2.set(def.localAnchorB);
		
		assert (def.ratio != 0.0f);
		m_ratio = def.ratio;
		
		m_constant = def.lengthA + m_ratio * def.lengthB;
		
		m_maxLength1 = MathUtils.min(def.maxLengthA, m_constant - m_ratio * MIN_PULLEY_LENGTH);
		m_maxLength2 = MathUtils.min(def.maxLengthB, (m_constant - MIN_PULLEY_LENGTH) / m_ratio);
		
		m_impulse = 0.0f;
		m_limitImpulse1 = 0.0f;
		m_limitImpulse2 = 0.0f;
	}
	
	/**
	 * @see org.jbox2d.dynamics.joints.Joint#getAnchorA(org.jbox2d.common.Vec2)
	 */
	@Override
	public void getAnchorA(Vec2 argOut) {
		m_bodyA.getWorldPointToOut(m_localAnchor1, argOut);
	}
	
	/**
	 * @see org.jbox2d.dynamics.joints.Joint#getAnchorB(org.jbox2d.common.Vec2)
	 */
	@Override
	public void getAnchorB(Vec2 argOut) {
		m_bodyB.getWorldPointToOut(m_localAnchor2, argOut);
	}
	
	/**
	 * @see org.jbox2d.dynamics.joints.Joint#getReactionForce(float,
	 *      org.jbox2d.common.Vec2)
	 */
	@Override
	public void getReactionForce(float inv_dt, Vec2 argOut) {
		argOut.set(m_u2).mulLocal(m_impulse).mulLocal(inv_dt);
	}
	
	/**
	 * @see org.jbox2d.dynamics.joints.Joint#getReactionTorque(float)
	 */
	@Override
	public float getReactionTorque(float inv_dt) {
		return 0f;
	}
	
	public Vec2 getGroundAnchorA() {
		return m_groundAnchor1;
	}
	
	public Vec2 getGroundAnchorB() {
		return m_groundAnchor2;
	}
	
	public float getLength1() {
		final Vec2 p = pool.popVec2();
		m_bodyA.getWorldPointToOut(m_localAnchor1, p);
		p.subLocal(m_groundAnchor1);
		
		float len = p.length();
		pool.pushVec2(1);
		return len;
	}
	
	public float getLength2() {
		final Vec2 p = pool.popVec2();
		m_bodyB.getWorldPointToOut(m_localAnchor2, p);
		p.subLocal(m_groundAnchor2);
		
		float len = p.length();
		pool.pushVec2(1);
		return len;
	}
	
	public float getRatio() {
		return m_ratio;
	}
	
	/**
	 * @see org.jbox2d.dynamics.joints.Joint#initVelocityConstraints(org.jbox2d.dynamics.TimeStep)
	 */
	@Override
	public void initVelocityConstraints(TimeStep step) {
		Body b1 = m_bodyA;
		Body b2 = m_bodyB;
		
		final Vec2 r1 = pool.popVec2();
		final Vec2 r2 = pool.popVec2();
		final Vec2 p1 = pool.popVec2();
		final Vec2 p2 = pool.popVec2();
		final Vec2 s1 = pool.popVec2();
		final Vec2 s2 = pool.popVec2();
		
		r1.set(m_localAnchor1).subLocal(b1.getLocalCenter());
		r2.set(m_localAnchor2).subLocal(b2.getLocalCenter());
		
		Mat22.mulToOut(b1.getTransform().R, r1, r1);
		Mat22.mulToOut(b2.getTransform().R, r2, r2);
		
		p1.set(b1.m_sweep.c).addLocal(r1);
		p2.set(b2.m_sweep.c).addLocal(r2);
		
		s1.set(m_groundAnchor1);
		s2.set(m_groundAnchor2);
		
		// Get the pulley axes.
		m_u1.set(p1).subLocal(s1);
		m_u2.set(p2).subLocal(s2);
		
		float length1 = m_u1.length();
		float length2 = m_u2.length();
		
		if (length1 > Settings.linearSlop) {
			m_u1.mulLocal(1.0f / length1);
		}
		else {
			m_u1.setZero();
		}
		
		if (length2 > Settings.linearSlop) {
			m_u2.mulLocal(1.0f / length2);
		}
		else {
			m_u2.setZero();
		}
		
		float C = m_constant - length1 - m_ratio * length2;
		if (C > 0.0f) {
			m_state = LimitState.INACTIVE;
			m_impulse = 0.0f;
		}
		else {
			m_state = LimitState.AT_UPPER;
		}
		
		if (length1 < m_maxLength1) {
			m_limitState1 = LimitState.INACTIVE;
			m_limitImpulse1 = 0.0f;
		}
		else {
			m_limitState1 = LimitState.AT_UPPER;
		}
		
		if (length2 < m_maxLength2) {
			m_limitState2 = LimitState.INACTIVE;
			m_limitImpulse2 = 0.0f;
		}
		else {
			m_limitState2 = LimitState.AT_UPPER;
		}
		
		// Compute effective mass.
		float cr1u1 = Vec2.cross(r1, m_u1);
		float cr2u2 = Vec2.cross(r2, m_u2);
		
		m_limitMass1 = b1.m_invMass + b1.m_invI * cr1u1 * cr1u1;
		m_limitMass2 = b2.m_invMass + b2.m_invI * cr2u2 * cr2u2;
		m_pulleyMass = m_limitMass1 + m_ratio * m_ratio * m_limitMass2;
		assert (m_limitMass1 > Settings.EPSILON);
		assert (m_limitMass2 > Settings.EPSILON);
		assert (m_pulleyMass > Settings.EPSILON);
		m_limitMass1 = 1.0f / m_limitMass1;
		m_limitMass2 = 1.0f / m_limitMass2;
		m_pulleyMass = 1.0f / m_pulleyMass;
		
		if (step.warmStarting) {
			// Scale impulses to support variable time steps.
			m_impulse *= step.dtRatio;
			m_limitImpulse1 *= step.dtRatio;
			m_limitImpulse2 *= step.dtRatio;
			
			final Vec2 P1 = pool.popVec2();
			final Vec2 P2 = pool.popVec2();
			final Vec2 temp = pool.popVec2();
			
			// Warm starting.
			P1.set(m_u1).mulLocal(-(m_impulse + m_limitImpulse1));
			P2.set(m_u2).mulLocal(-m_ratio * m_impulse - m_limitImpulse2);
			
			temp.set(P1).mulLocal(b1.m_invMass);
			b1.m_linearVelocity.addLocal(temp);
			b1.m_angularVelocity += b1.m_invI * Vec2.cross(r1, P1);
			
			temp.set(P2).mulLocal(b2.m_invMass);
			b2.m_linearVelocity.addLocal(temp);
			b2.m_angularVelocity += b2.m_invI * Vec2.cross(r2, P2);
			
			pool.pushVec2(3);
		}
		else {
			m_impulse = 0.0f;
			m_limitImpulse1 = 0.0f;
			m_limitImpulse2 = 0.0f;
		}
		
		pool.pushVec2(6);
	}
	
	/**
	 * @see org.jbox2d.dynamics.joints.Joint#solveVelocityConstraints(org.jbox2d.dynamics.TimeStep)
	 */
	@Override
	public void solveVelocityConstraints(TimeStep step) {
		Body b1 = m_bodyA;
		Body b2 = m_bodyB;
		
		final Vec2 r1 = pool.popVec2();
		final Vec2 r2 = pool.popVec2();
		
		r1.set(m_localAnchor1).subLocal(b1.getLocalCenter());
		r2.set(m_localAnchor2).subLocal(b2.getLocalCenter());
		
		Mat22.mulToOut(b1.getTransform().R, r1, r1);
		Mat22.mulToOut(b2.getTransform().R, r2, r2);
		
		if (m_state == LimitState.AT_UPPER) {
			final Vec2 v1 = pool.popVec2();
			final Vec2 v2 = pool.popVec2();
			
			Vec2.crossToOut(b1.m_angularVelocity, r1, v1);
			Vec2.crossToOut(b2.m_angularVelocity, r2, v2);
			
			v1.addLocal(b1.m_linearVelocity);
			v2.addLocal(b2.m_linearVelocity);
			
			float Cdot = -Vec2.dot(m_u1, v1) - m_ratio * Vec2.dot(m_u2, v2);
			float impulse = m_pulleyMass * (-Cdot);
			float oldImpulse = m_impulse;
			m_impulse = MathUtils.max(0.0f, m_impulse + impulse);
			impulse = m_impulse - oldImpulse;
			
			final Vec2 P1 = pool.popVec2();
			final Vec2 P2 = pool.popVec2();
			final Vec2 temp = pool.popVec2();
			
			P1.set(m_u1).mulLocal(-impulse);
			P2.set(m_u2).mulLocal(-m_ratio * impulse);
			
			temp.set(P1).mulLocal(b1.m_invMass);
			b1.m_linearVelocity.addLocal(temp);
			b1.m_angularVelocity += b1.m_invI * Vec2.cross(r1, P1);
			
			temp.set(P2).mulLocal(b2.m_invMass);
			b2.m_linearVelocity.addLocal(temp);
			b2.m_angularVelocity += b2.m_invI * Vec2.cross(r2, P2);
			
			pool.pushVec2(5);
		}
		
		if (m_limitState1 == LimitState.AT_UPPER) {
			final Vec2 v1 = pool.popVec2();
			
			Vec2.crossToOut(b1.m_angularVelocity, r1, v1);
			v1.addLocal(b1.m_linearVelocity);
			
			float Cdot = -Vec2.dot(m_u1, v1);
			float impulse = -m_limitMass1 * Cdot;
			float oldImpulse = m_limitImpulse1;
			m_limitImpulse1 = MathUtils.max(0.0f, m_limitImpulse1 + impulse);
			impulse = m_limitImpulse1 - oldImpulse;
			
			final Vec2 P1 = pool.popVec2();
			final Vec2 temp = pool.popVec2();
			
			P1.set(m_u1).mulLocal(-impulse);
			
			temp.set(P1).mulLocal(b1.m_invMass);
			b1.m_linearVelocity.addLocal(temp);
			b1.m_angularVelocity += b1.m_invI * Vec2.cross(r1, P1);
			
			pool.pushVec2(3);
		}
		
		if (m_limitState2 == LimitState.AT_UPPER) {
			
			final Vec2 v2 = pool.popVec2();
			Vec2.crossToOut(b2.m_angularVelocity, r2, v2);
			v2.addLocal(b2.m_linearVelocity);
			
			float Cdot = -Vec2.dot(m_u2, v2);
			float impulse = -m_limitMass2 * Cdot;
			float oldImpulse = m_limitImpulse2;
			m_limitImpulse2 = MathUtils.max(0.0f, m_limitImpulse2 + impulse);
			impulse = m_limitImpulse2 - oldImpulse;
			
			final Vec2 P2 = pool.popVec2();
			final Vec2 temp = pool.popVec2();
			
			P2.set(m_u2).mulLocal(-impulse);
			
			temp.set(P2).mulLocal(b2.m_invMass);
			b2.m_linearVelocity.addLocal(temp);
			b2.m_angularVelocity += b2.m_invI * Vec2.cross(r2, P2);
			
			pool.pushVec2(3);
		}
		
		pool.pushVec2(2);
	}
	
	/**
	 * @see org.jbox2d.dynamics.joints.Joint#solvePositionConstraints(float)
	 */
	@Override
	public boolean solvePositionConstraints(float baumgarte) {
		Body b1 = m_bodyA;
		Body b2 = m_bodyB;
		
		final Vec2 s1 = pool.popVec2();
		final Vec2 s2 = pool.popVec2();
		
		s1.set(m_groundAnchor1);
		s2.set(m_groundAnchor2);
		
		float linearError = 0.0f;
		
		if (m_state == LimitState.AT_UPPER) {
			final Vec2 r1 = pool.popVec2();
			final Vec2 r2 = pool.popVec2();
			final Vec2 p1 = pool.popVec2();
			final Vec2 p2 = pool.popVec2();
			
			r1.set(m_localAnchor1).subLocal(b1.getLocalCenter());
			r2.set(m_localAnchor2).subLocal(b2.getLocalCenter());
			
			Mat22.mulToOut(b1.getTransform().R, r1, r1);
			Mat22.mulToOut(b2.getTransform().R, r2, r2);
			
			p1.set(b1.m_sweep.c).addLocal(r1);
			p2.set(b2.m_sweep.c).addLocal(r2);
			
			// Get the pulley axes.
			m_u1.set(p1).subLocal(s1);
			m_u2.set(p2).subLocal(s2);
			
			float length1 = m_u1.length();
			float length2 = m_u2.length();
			
			if (length1 > Settings.linearSlop) {
				m_u1.mulLocal(1.0f / length1);
			}
			else {
				m_u1.setZero();
			}
			
			if (length2 > Settings.linearSlop) {
				m_u2.mulLocal(1.0f / length2);
			}
			else {
				m_u2.setZero();
			}
			
			float C = m_constant - length1 - m_ratio * length2;
			linearError = MathUtils.max(linearError, -C);
			
			C = MathUtils.clamp(C + Settings.linearSlop, -Settings.maxLinearCorrection, 0.0f);
			float impulse = -m_pulleyMass * C;
			
			final Vec2 P1 = pool.popVec2();
			final Vec2 P2 = pool.popVec2();
			final Vec2 temp = pool.popVec2();
			
			P1.set(m_u1).mulLocal(-impulse);
			P2.set(m_u2).mulLocal(-m_ratio * impulse);
			
			temp.set(P1).mulLocal(b1.m_invMass);
			b1.m_sweep.c.addLocal(temp);
			b1.m_sweep.a += b1.m_invI * Vec2.cross(r1, P1);
			
			temp.set(P2).mulLocal(b2.m_invMass);
			b2.m_sweep.c.addLocal(temp);
			b2.m_sweep.a += b2.m_invI * Vec2.cross(r2, P2);
			
			b1.synchronizeTransform();
			b2.synchronizeTransform();
			
			pool.pushVec2(7);
		}
		
		if (m_limitState1 == LimitState.AT_UPPER) {
			final Vec2 r1 = pool.popVec2();
			final Vec2 p1 = pool.popVec2();
			
			r1.set(m_localAnchor1).subLocal(b1.getLocalCenter());
			
			Mat22.mulToOut(b1.getTransform().R, r1, r1);
			
			p1.set(b1.m_sweep.c).addLocal(r1);
			
			m_u1.set(p1).subLocal(s1);
			
			float length1 = m_u1.length();
			
			if (length1 > Settings.linearSlop) {
				m_u1.mulLocal(1.0f / length1);
			}
			else {
				m_u1.setZero();
			}
			
			float C = m_maxLength1 - length1;
			linearError = MathUtils.max(linearError, -C);
			C = MathUtils.clamp(C + Settings.linearSlop, -Settings.maxLinearCorrection, 0.0f);
			float impulse = -m_limitMass1 * C;
			
			final Vec2 P1 = pool.popVec2();
			final Vec2 temp = pool.popVec2();
			
			P1.set(m_u1).mulLocal(-impulse);
			
			temp.set(P1).mulLocal(b1.m_invMass);
			b1.m_sweep.c.addLocal(temp);
			b1.m_sweep.a += b1.m_invI * Vec2.cross(r1, P1);
			
			b1.synchronizeTransform();
			
			pool.pushVec2(4);
		}
		
		if (m_limitState2 == LimitState.AT_UPPER) {
			final Vec2 r2 = pool.popVec2();
			final Vec2 p2 = pool.popVec2();
			
			r2.set(m_localAnchor2).subLocal(b2.getLocalCenter());
			
			Mat22.mulToOut(b2.getTransform().R, r2, r2);
			
			p2.set(b2.m_sweep.c).addLocal(r2);
			
			// Get the pulley axes.
			m_u2.set(p2).subLocal(s2);
			
			float length2 = m_u2.length();
			
			if (length2 > Settings.linearSlop) {
				m_u2.mulLocal(1.0f / length2);
			}
			else {
				m_u2.setZero();
			}
			
			float C = m_maxLength2 - length2;
			linearError = MathUtils.max(linearError, -C);
			C = MathUtils.clamp(C + Settings.linearSlop, -Settings.maxLinearCorrection, 0.0f);
			float impulse = -m_limitMass2 * C;
			
			final Vec2 P2 = pool.popVec2();
			final Vec2 temp = pool.popVec2();
			
			P2.set(m_u2).mulLocal(-impulse);
			
			temp.set(P2).mulLocal(b2.m_invMass);
			b2.m_sweep.c.addLocal(temp);
			b2.m_sweep.a += b2.m_invI * Vec2.cross(r2, P2);
			
			b2.synchronizeTransform();
			
			pool.pushVec2(4);
		}
		pool.pushVec2(2);
		
		return linearError < Settings.linearSlop;
	}
}
