/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/
 * Box2D homepage: http://www.box2d.org
 * 
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package org.jbox2d.dynamics.joints;

import org.jbox2d.common.Mat22;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.TimeStep;
import org.jbox2d.pooling.TLVec2;


//Updated to rev. 56->130 of b2PulleyJoint.cpp/.h

public class PulleyJoint extends Joint {
	// The pulley joint is connected to two bodies and two fixed ground points.
	// The pulley supports a ratio such that:
	// length1 + ratio * length2 = constant
	// Yes, the force transmitted is scaled by the ratio.
	// The pulley also enforces a maximum length limit on both sides. This is
	// useful to prevent one side of the pulley hitting the top.

	// Pulley:
	// length1 = norm(p1 - s1)
	// length2 = norm(p2 - s2)
	// C0 = (length1 + ratio * length2)_initial
	// C = C0 - (length1 + ratio * length2) = 0
	// u1 = (p1 - s1) / norm(p1 - s1)
	// u2 = (p2 - s2) / norm(p2 - s2)
	// Cdot = -dot(u1, v1 + cross(w1, r1)) - ratio * dot(u2, v2 + cross(w2, r2))
	// J = -[u1 cross(r1, u1) ratio * u2 ratio * cross(r2, u2)]
	// K = J * invM * JT
	// = invMass1 + invI1 * cross(r1, u1)^2 + ratio^2 * (invMass2 + invI2 *
	// cross(r2, u2)^2)
	//
	// Limit:
	// C = maxLength - length
	// u = (p - s) / norm(p - s)
	// Cdot = -dot(u, v + cross(w, r))
	// K = invMass + invI * cross(r, u)^2
	// 0 <= impulse

	// We need a minimum pulley length to help prevent one side going to zero.
	public static final float MIN_PULLEY_LENGTH = 2.0f;//Settings.lengthUnitsPerMeter;

	public Body m_ground;
	public final Vec2 m_groundAnchor1;
	public final Vec2 m_groundAnchor2;
	public final Vec2 m_localAnchor1;
	public final Vec2 m_localAnchor2;

	public final Vec2 m_u1;
	public final Vec2 m_u2;

	public float m_constant;
	public float m_ratio;

	public float m_maxLength1;
	public float m_maxLength2;

	// Effective masses
	public float m_pulleyMass;
	public float m_limitMass1;
	public float m_limitMass2;

	// Impulses for accumulation/warm starting.
	public float m_force;
	public float m_limitForce1;
	public float m_limitForce2;

	// Position impulses for accumulation.
	public float m_positionImpulse;
	public float m_limitPositionImpulse1;
	public float m_limitPositionImpulse2;

	public LimitState m_state;
	public LimitState m_limitState1;
	public LimitState m_limitState2;

	public PulleyJoint(final PulleyJointDef def) {
		super(def);
		m_ground = m_body1.m_world.getGroundBody();
		m_groundAnchor1 = def.groundAnchor1.sub(m_ground.m_xf.position);
		m_groundAnchor2 = def.groundAnchor2.sub(m_ground.m_xf.position);
		m_localAnchor1 = def.localAnchor1.clone();
		m_localAnchor2 = def.localAnchor2.clone();
		m_u1 = new Vec2();
		m_u2 = new Vec2();

		assert(def.ratio != 0.0f);
		m_ratio = def.ratio;

		m_constant = def.length1 + m_ratio * def.length2;

		m_maxLength1 = MathUtils.min(def.maxLength1, m_constant - m_ratio * MIN_PULLEY_LENGTH);
		m_maxLength2 = MathUtils.min(def.maxLength2, (m_constant - MIN_PULLEY_LENGTH) / m_ratio);

		m_force = 0.0f;
		m_limitForce1 = 0.0f;
		m_limitForce2 = 0.0f;
	}

	// djm pooled
	private static final TLVec2 tlr1 = new TLVec2();
	private static final TLVec2 tlr2 = new TLVec2();
	private static final TLVec2 tlp1 = new TLVec2();
	private static final TLVec2 tlp2 = new TLVec2();
	private static final TLVec2 tls1 = new TLVec2();
	private static final TLVec2 tls2 = new TLVec2();
	private static final TLVec2 tlP1 = new TLVec2();
	private static final TLVec2 tlP2 = new TLVec2();
	@Override
	public void initVelocityConstraints(final TimeStep step) {
		final Body b1 = m_body1;
		final Body b2 = m_body2;
		
		final Vec2 r1 = tlr1.get();
		final Vec2 r2 = tlr2.get();
		final Vec2 p1 = tlp1.get();
		final Vec2 p2 = tlp2.get();
		final Vec2 s1 = tls1.get();
		final Vec2 s2 = tls2.get();
		final Vec2 P1 = tlP1.get();
		final Vec2 P2 = tlP2.get();

		r1.set(b1.getMemberLocalCenter());
		r2.set(b2.getMemberLocalCenter());
		r1.subLocal(m_localAnchor1).negateLocal();
		r2.subLocal(m_localAnchor2).negateLocal();
		Mat22.mulToOut(b1.m_xf.R, r1, r1);
		Mat22.mulToOut(b2.m_xf.R, r2, r2);


		p1.set(b1.m_sweep.c);
		p1.addLocal(r1);
		p2.set(b2.m_sweep.c);
		p2.addLocal(r2);

		s1.set(m_ground.m_xf.position);
		s1.addLocal(m_groundAnchor1);
		s2.set(m_ground.m_xf.position);
		s2.addLocal(m_groundAnchor2);

		// Get the pulley axes.
		m_u1.set(p1);
		m_u1.subLocal(s1);
		m_u2.set(p2);
		m_u2.subLocal(s2);

		final float length1 = m_u1.length();
		final float length2 = m_u2.length();

		if (length1 > Settings.linearSlop) {
			m_u1.mulLocal(1.0f / length1);
		} else {
			m_u1.setZero();
		}

		if (length2 > Settings.linearSlop) {
			m_u2.mulLocal(1.0f / length2);
		} else {
			m_u2.setZero();
		}

		final float C = m_constant - length1 - m_ratio * length2;
		if (C > 0.0f) {
			m_state = LimitState.INACTIVE_LIMIT;
			m_force = 0.0f;
		} else {
			m_state = LimitState.AT_UPPER_LIMIT;
			m_positionImpulse = 0.0f;
		}

		if (length1 < m_maxLength1) {
			m_limitState1 = LimitState.INACTIVE_LIMIT;
			m_limitForce1 = 0.0f;
		} else {
			m_limitState1 = LimitState.AT_UPPER_LIMIT;
			m_limitPositionImpulse1 = 0.0f;
		}

		if (length2 < m_maxLength2) {
			m_limitState2 = LimitState.INACTIVE_LIMIT;
			m_limitForce2 = 0.0f;
		} else {
			m_limitState2 = LimitState.AT_UPPER_LIMIT;
			m_limitPositionImpulse2 = 0.0f;
		}

		// Compute effective mass.
		final float cr1u1 = Vec2.cross(r1, m_u1);
		final float cr2u2 = Vec2.cross(r2, m_u2);

		m_limitMass1 = b1.m_invMass + b1.m_invI * cr1u1 * cr1u1;
		m_limitMass2 = b2.m_invMass + b2.m_invI * cr2u2 * cr2u2;
		m_pulleyMass = m_limitMass1 + m_ratio * m_ratio * m_limitMass2;
		assert(m_limitMass1 > Settings.EPSILON);
		assert(m_limitMass2 > Settings.EPSILON);
		assert(m_pulleyMass > Settings.EPSILON);
		m_limitMass1 = 1.0f / m_limitMass1;
		m_limitMass2 = 1.0f / m_limitMass2;
		m_pulleyMass = 1.0f / m_pulleyMass;

		if (step.warmStarting) {
			// Warm starting.
			P1.set(m_u1);
			P1.mulLocal(step.dt * (-m_force - m_limitForce1));
			P2.set(m_u2);
			P2.mulLocal(step.dt * (-m_ratio * m_force - m_limitForce2));
			b1.m_angularVelocity += b1.m_invI * Vec2.cross(r1, P1);
			b2.m_angularVelocity += b2.m_invI * Vec2.cross(r2, P2);
			b1.m_linearVelocity.addLocal(P1.mulLocal(b1.m_invMass));
			b2.m_linearVelocity.addLocal(P2.mulLocal(b2.m_invMass));
		} else {
			m_force = 0.0f;
			m_limitForce1 = 0.0f;
			m_limitForce2 = 0.0f;
		}
	}

	// djm pooled, some from above
	private static final TLVec2 tlv1 = new TLVec2();
	private static final TLVec2 tlv2 = new TLVec2();
	
	@Override
	public void solveVelocityConstraints(final TimeStep step) {
		final Body b1 = m_body1;
		final Body b2 = m_body2;

		final Vec2 v1 = tlv1.get();
		final Vec2 v2 = tlv2.get();
		final Vec2 P1 = tlP1.get();
		final Vec2 P2 = tlP2.get();
		final Vec2 r1 = tlr1.get();
		final Vec2 r2 = tlr2.get();

		r1.set(b1.getMemberLocalCenter());
		r2.set(b2.getMemberLocalCenter());
		r1.subLocal(m_localAnchor1).negateLocal();
		r2.subLocal(m_localAnchor2).negateLocal();
		Mat22.mulToOut(b1.m_xf.R, r1, r1);
		Mat22.mulToOut(b2.m_xf.R, r2, r2);

		if (m_state == LimitState.AT_UPPER_LIMIT) {
			//Vec2 v1 = b1.m_linearVelocity.add(Vec2.cross(b1.m_angularVelocity, r1));
			//Vec2 v2 = b2.m_linearVelocity.add(Vec2.cross(b2.m_angularVelocity, r2));
			Vec2.crossToOut(b1.m_angularVelocity, r1, v1);
			Vec2.crossToOut(b2.m_angularVelocity, r2, v2);
			v1.add(b1.m_linearVelocity);
			v2.add(b2.m_linearVelocity);

			final float Cdot = -Vec2.dot(m_u1, v1) - m_ratio * Vec2.dot(m_u2, v2);
			float force = -step.inv_dt * m_pulleyMass * Cdot;
			final float oldForce = m_force;
			m_force = MathUtils.max(0.0f, m_force + force);
			force = m_force - oldForce;

			P1.set(m_u1);
			P1.mulLocal(-step.dt * force);
			P2.set(m_u2);
			P2.mulLocal(-step.dt * m_ratio * force);
			b1.m_linearVelocity.x += b1.m_invMass * P1.x;
			b1.m_linearVelocity.y += b1.m_invMass * P1.y;
			b1.m_angularVelocity += b1.m_invI * Vec2.cross(r1, P1);
			b2.m_linearVelocity.x += b2.m_invMass * P2.x;
			b2.m_linearVelocity.y += b2.m_invMass * P2.y;
			b2.m_angularVelocity += b2.m_invI * Vec2.cross(r2, P2);
		}

		if (m_limitState1 == LimitState.AT_UPPER_LIMIT) {
			Vec2.crossToOut(b1.m_angularVelocity, r1, v1);
			v1.addLocal(b1.m_linearVelocity);
			//Vec2 v1 = b1.m_linearVelocity.add(Vec2.cross(b1.m_angularVelocity, r1));

			final float Cdot = -Vec2.dot(m_u1, v1);
			float force = -step.inv_dt * m_limitMass1 * Cdot;
			final float oldForce = m_limitForce1;
			m_limitForce1 = MathUtils.max(0.0f, m_limitForce1 + force);
			force = m_limitForce1 - oldForce;

			//Vec2 P1 = m_u1.mul(-step.dt * force);
			P1.set(m_u1);
			P1.mulLocal(-step.dt * force);
			b1.m_linearVelocity.x += b1.m_invMass * P1.x;
			b1.m_linearVelocity.y += b1.m_invMass * P1.y;
			b1.m_angularVelocity += b1.m_invI * Vec2.cross(r1, P1);
		}

		if (m_limitState2 == LimitState.AT_UPPER_LIMIT) {
			Vec2.crossToOut(b2.m_angularVelocity, r2, v2);
			v2.addLocal(b2.m_linearVelocity);
			//Vec2 v2 = b2.m_linearVelocity.add(Vec2.cross(b2.m_angularVelocity, r2));

			final float Cdot = -Vec2.dot(m_u2, v2);
			float force = -step.inv_dt * m_limitMass2 * Cdot;
			final float oldForce = m_limitForce2;
			m_limitForce2 = MathUtils.max(0.0f, m_limitForce2 + force);
			force = m_limitForce2 - oldForce;

			//Vec2 P2 = m_u2.mul(-step.dt * force);
			P2.set(m_u2);
			P2.mulLocal(-step.dt * force);
			b2.m_linearVelocity.x += b2.m_invMass * P2.x;
			b2.m_linearVelocity.y += b2.m_invMass * P2.y;
			b2.m_angularVelocity += b2.m_invI * Vec2.cross(r2, P2);
		}
	}

	//djm pooled, some from above

	@Override
	public boolean solvePositionConstraints() {
		final Body b1 = m_body1;
		final Body b2 = m_body2;

		final Vec2 r1 = tlr1.get();
		final Vec2 r2 = tlr2.get();
		final Vec2 p1 = tlp1.get();
		final Vec2 p2 = tlp2.get();
		final Vec2 s1 = tls1.get();
		final Vec2 s2 = tls2.get();
		final Vec2 P1 = tlP1.get();
		final Vec2 P2 = tlP2.get();
		
		s1.set(m_ground.m_xf.position);
		s1.addLocal(m_groundAnchor1);
		s2.set(m_ground.m_xf.position);
		s2.addLocal(m_groundAnchor2);

		float linearError = 0.0f;

		if (m_state == LimitState.AT_UPPER_LIMIT) {
			r1.set(b1.getMemberLocalCenter());
			r2.set(b2.getMemberLocalCenter());
			r1.subLocal(m_localAnchor1).negateLocal();
			r2.subLocal(m_localAnchor2).negateLocal();
			Mat22.mulToOut(b1.m_xf.R, r1, r1);
			Mat22.mulToOut(b2.m_xf.R, r2, r2);

			p1.set(b1.m_sweep.c);
			p1.addLocal(r1);
			p2.set(b2.m_sweep.c);
			p2.addLocal(r2);

			// Get the pulley axes.
			m_u1.set(p1.x - s1.x,p1.y - s1.y);
			m_u2.set(p2.x - s2.x,p2.y - s2.y);

			final float length1 = m_u1.length();
			final float length2 = m_u2.length();

			if (length1 > Settings.linearSlop) {
				m_u1.mulLocal(1.0f / length1);
			} else {
				m_u1.setZero();
			}

			if (length2 > Settings.linearSlop) {
				m_u2.mulLocal(1.0f / length2);
			} else {
				m_u2.setZero();
			}

			float C = m_constant - length1 - m_ratio * length2;
			linearError = MathUtils.max(linearError, -C);

			C = MathUtils.clamp(C + Settings.linearSlop, -Settings.maxLinearCorrection, 0.0f);
			float impulse = -m_pulleyMass * C;
			final float oldImpulse = m_positionImpulse;
			m_positionImpulse = MathUtils.max(0.0f, m_positionImpulse + impulse);
			impulse = m_positionImpulse - oldImpulse;

			//Vec2 P1 = m_u1.mul(-impulse);
			//Vec2 P2 = m_u2.mul(-m_ratio * impulse);
			P1.set(m_u1);
			P1.mulLocal(-impulse);
			P2.set(m_u2);
			P2.mulLocal(-m_ratio * impulse);

			b1.m_sweep.c.x += b1.m_invMass * P1.x;
			b1.m_sweep.c.y += b1.m_invMass * P1.y;
			b1.m_sweep.a += b1.m_invI * Vec2.cross(r1, P1);
			b2.m_sweep.c.x += b2.m_invMass * P2.x;
			b2.m_sweep.c.y += b2.m_invMass * P2.y;
			b2.m_sweep.a += b2.m_invI * Vec2.cross(r2, P2);

			b1.synchronizeTransform();
			b2.synchronizeTransform();
		}

		if (m_limitState1 == LimitState.AT_UPPER_LIMIT) {
			//Vec2 r1 = Mat22.mul(b1.m_xf.R, m_localAnchor1.sub(b1.getMemberLocalCenter()));
			r1.set(b1.getMemberLocalCenter());
			r1.subLocal(m_localAnchor1).negateLocal();
			Mat22.mulToOut(b1.m_xf.R, r1, r1);

			//Vec2 p1 = b1.m_sweep.c.add(r1);
			p1.set(b1.m_sweep.c);
			p1.addLocal(r1);

			m_u1.set(p1.x - s1.x, p1.y - s1.y);
			final float length1 = m_u1.length();

			if (length1 > Settings.linearSlop) {
				m_u1.mulLocal(1.0f / length1);
			} else {
				m_u1.setZero();
			}

			float C = m_maxLength1 - length1;
			linearError = MathUtils.max(linearError, -C);
			C = MathUtils.clamp(C + Settings.linearSlop, -Settings.maxLinearCorrection, 0.0f);
			float impulse = -m_limitMass1 * C;
			final float oldLimitPositionImpulse = m_limitPositionImpulse1;
			m_limitPositionImpulse1 = MathUtils.max(0.0f, m_limitPositionImpulse1 + impulse);
			impulse = m_limitPositionImpulse1 - oldLimitPositionImpulse;

			//Vec2 P1 = m_u1.mul(-impulse);
			P1.set(m_u1);
			P1.mulLocal(-impulse);

			b1.m_sweep.c.x += b1.m_invMass * P1.x;
			b1.m_sweep.c.y += b1.m_invMass * P1.y;
			b1.m_sweep.a += b1.m_invI * Vec2.cross(r1, P1);

			b1.synchronizeTransform();
		}

		if (m_limitState2 == LimitState.AT_UPPER_LIMIT) {
			//Vec2 r2 = Mat22.mul(b2.m_xf.R, m_localAnchor2.sub(b2.getMemberLocalCenter()));
			//Vec2 p2 = b2.m_sweep.c.add(r2);

			r2.set(b2.getMemberLocalCenter());
			r2.subLocal(m_localAnchor2).negateLocal();
			Mat22.mulToOut(b2.m_xf.R, r2, r2);

			p2.set(b2.m_sweep.c);
			p2.addLocal(r2);

			m_u2.set(p2.x - s2.x, p2.y - s2.y);
			final float length2 = m_u2.length();

			if (length2 > Settings.linearSlop) {
				m_u2.mulLocal(1.0f / length2);
			} else {
				m_u2.setZero();
			}

			float C = m_maxLength2 - length2;
			linearError = MathUtils.max(linearError, -C);
			C = MathUtils.clamp(C + Settings.linearSlop, -Settings.maxLinearCorrection, 0.0f);
			float impulse = -m_limitMass2 * C;
			final float oldLimitPositionImpulse = m_limitPositionImpulse2;
			m_limitPositionImpulse2 = MathUtils.max(0.0f, m_limitPositionImpulse2 + impulse);
			impulse = m_limitPositionImpulse2 - oldLimitPositionImpulse;

			//Vec2 P2 = m_u2.mul(-impulse);
			P2.set(m_u2);
			P2.mulLocal(-impulse);
			b2.m_sweep.c.x += b2.m_invMass * P2.x;
			b2.m_sweep.c.y += b2.m_invMass * P2.y;
			b2.m_sweep.a += b2.m_invI * Vec2.cross(r2, P2);

			b2.synchronizeTransform();
		}

		return linearError < Settings.linearSlop;
	}

	@Override
	public Vec2 getAnchor1() {
		return m_body1.getWorldLocation(m_localAnchor1);
	}

	@Override
	public Vec2 getAnchor2() {
		return m_body2.getWorldLocation(m_localAnchor2);
	}

	@Override
	public Vec2 getReactionForce() {
		final Vec2 F = m_u2.mul(m_force);
		return F;
	}

	@Override
	public float getReactionTorque() {
		return 0.0f;
	}

	public Vec2 getGroundAnchor1() {
		return m_ground.m_xf.position.add(m_groundAnchor1);
	}

	public Vec2 getGroundAnchor2() {
		return m_ground.m_xf.position.add(m_groundAnchor2);
	}

	public float getLength1() {
		final Vec2 p = m_body1.getWorldLocation(m_localAnchor1);
		final Vec2 s = m_ground.m_xf.position.add(m_groundAnchor1);
		final Vec2 d = p.subLocal(s);
		return d.length();
	}

	public float getLength2() {
		final Vec2 p = m_body2.getWorldLocation(m_localAnchor2);
		final Vec2 s = m_ground.m_xf.position.add(m_groundAnchor2);
		final Vec2 d = p.subLocal(s);
		return d.length();
	}

	public float getRatio() {
		return m_ratio;
	}

}