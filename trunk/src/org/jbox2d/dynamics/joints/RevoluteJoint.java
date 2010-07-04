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
import org.jbox2d.pooling.TLMat22;
import org.jbox2d.pooling.TLVec2;


//Updated to rev. 56->108 of b2RevoluteJoint.cpp/.h

//Point-to-point constraint
//C = p2 - p1
//Cdot = v2 - v1
//   = v2 + cross(w2, r2) - v1 - cross(w1, r1)
//J = [-I -r1_skew I r2_skew ]
//Identity used:
//w k % (rx i + ry j) = w * (-ry i + rx j)

//Motor constraint
//Cdot = w2 - w1
//J = [0 0 -1 0 0 1]
//K = invI1 + invI2

public class RevoluteJoint extends Joint {
	public final Vec2 m_localAnchor1;	// relative
	public final Vec2 m_localAnchor2;
	public final Vec2 m_pivotForce;
	public float m_motorForce;
	public float m_limitForce;
	public float m_limitPositionImpulse;

	public final Mat22 m_pivotMass;		// effective mass for point-to-point constraint.
	public float m_motorMass;	// effective mass for motor/limit angular constraint.

	public boolean m_enableMotor;
	public float m_maxMotorTorque;
	public float m_motorSpeed;

	public boolean m_enableLimit;
	public float m_referenceAngle;
	public float m_lowerAngle;
	public float m_upperAngle;
	public LimitState m_limitState;

	public RevoluteJoint(final RevoluteJointDef def) {
		super(def);
		m_localAnchor1 = def.localAnchor1.clone();
		m_localAnchor2 = def.localAnchor2.clone();
		m_referenceAngle = def.referenceAngle;

		m_pivotForce = new Vec2(0.0f, 0.0f);
		m_motorForce = 0.0f;
		m_limitForce = 0.0f;
		m_limitPositionImpulse = 0.0f;
		m_pivotMass = new Mat22();

		m_lowerAngle = def.lowerAngle;
		m_upperAngle = def.upperAngle;
		m_maxMotorTorque = def.maxMotorTorque;
		m_motorSpeed = def.motorSpeed;
		m_enableLimit = def.enableLimit;
		m_enableMotor = def.enableMotor;
	}

	// djm pooled
	private static final TLVec2 tlr1 = new TLVec2();
	private static final TLVec2 tlr2 = new TLVec2();
	private static final TLMat22 tlK1 = new TLMat22();
	private static final TLMat22 tlK2 = new TLMat22();
	private static final TLMat22 tlK3 = new TLMat22();
	@Override
	public void initVelocityConstraints(final TimeStep step) {
		final Body b1 = m_body1;
		final Body b2 = m_body2;
		
		final Vec2 r1 = tlr1.get();
		final Vec2 r2 = tlr2.get();
		final Mat22 K1 = tlK1.get();
		final Mat22 K2 = tlK2.get();
		final Mat22 K3 = tlK3.get();

		// Compute the effective mass matrix.
		//Vec2 r1 = Mat22.mul(b1.m_xf.R, m_localAnchor1.sub(b1.getMemberLocalCenter()));
		//Vec2 r2 = Mat22.mul(b2.m_xf.R, m_localAnchor2.sub(b2.getMemberLocalCenter()));
		r1.set(b1.getMemberLocalCenter());
		r2.set(b2.getMemberLocalCenter());
		r1.subLocal(m_localAnchor1).negateLocal();
		r2.subLocal(m_localAnchor2).negateLocal();
		Mat22.mulToOut(b1.m_xf.R, r1, r1);
		Mat22.mulToOut(b2.m_xf.R, r2, r2);

		// K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
		//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
		//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
		final float invMass1 = b1.m_invMass, invMass2 = b2.m_invMass;
		final float invI1 = b1.m_invI, invI2 = b2.m_invI;

		K1.col1.x = invMass1 + invMass2;	K1.col2.x = 0.0f;
		K1.col1.y = 0.0f;					K1.col2.y = invMass1 + invMass2;

		K2.col1.x =  invI1 * r1.y * r1.y;	K2.col2.x = -invI1 * r1.x * r1.y;
		K2.col1.y = -invI1 * r1.x * r1.y;	K2.col2.y =  invI1 * r1.x * r1.x;

		K3.col1.x =  invI2 * r2.y * r2.y;	K3.col2.x = -invI2 * r2.x * r2.y;
		K3.col1.y = -invI2 * r2.x * r2.y;	K3.col2.y =  invI2 * r2.x * r2.x;

		//Mat22 K = K1.addLocal(K2).addLocal(K3);
		K1.addLocal(K2).addLocal(K3);

		//m_pivotMass = K1.invert();
		K1.invertToOut(m_pivotMass);

		m_motorMass = 1.0f / (invI1 + invI2);

		if (m_enableMotor == false) {
			m_motorForce = 0.0f;
		}

		if (m_enableLimit) {
			final float jointAngle = b2.m_sweep.a - b1.m_sweep.a - m_referenceAngle;
			if (MathUtils.abs(m_upperAngle - m_lowerAngle) < 2.0f * Settings.angularSlop) {
				m_limitState = LimitState.EQUAL_LIMITS;
			} else if (jointAngle <= m_lowerAngle) {
				if (m_limitState != LimitState.AT_LOWER_LIMIT) {
					m_limitForce = 0.0f;
				}
				m_limitState = LimitState.AT_LOWER_LIMIT;
			} else if (jointAngle >= m_upperAngle) {
				if (m_limitState != LimitState.AT_UPPER_LIMIT) {
					m_limitForce = 0.0f;
				}
				m_limitState = LimitState.AT_UPPER_LIMIT;
			}else {
				m_limitState = LimitState.INACTIVE_LIMIT;
				m_limitForce = 0.0f;
			}
		} else {
			m_limitForce = 0.0f;
		}

		if (step.warmStarting) {
			b1.m_linearVelocity.x -= step.dt * invMass1 * m_pivotForce.x;
			b1.m_linearVelocity.y -= step.dt * invMass1 * m_pivotForce.y;
			b1.m_angularVelocity -= step.dt * invI1 * (Vec2.cross(r1, m_pivotForce) + m_motorForce + m_limitForce);

			b2.m_linearVelocity.x += step.dt * invMass2 * m_pivotForce.x;
			b2.m_linearVelocity.y += step.dt * invMass2 * m_pivotForce.y;
			b2.m_angularVelocity += step.dt * invI2 * (Vec2.cross(r2, m_pivotForce) + m_motorForce + m_limitForce);
		} else {
			m_pivotForce.setZero();
			m_motorForce = 0.0f;
			m_limitForce = 0.0f;
		}

		m_limitPositionImpulse = 0.0f;
	}

	final Vec2 m_lastWarmStartingPivotForce = new Vec2(0.0f,0.0f);
	//private float m_lastWarmStartingMotorForce = 0.0f; djm not used
	//private float m_lastWarmStartingLimitForce = 0.0f;
	//private boolean m_warmStartingOld = true;

	// djm pooled, some from above
	private static final TLVec2 tltemp = new TLVec2();
	private static final TLVec2 tlpivotCdot = new TLVec2();
	private static final TLVec2 tlpivotForce = new TLVec2();
	@Override
	public void solveVelocityConstraints(final TimeStep step) {
		final Body b1 = m_body1;
		final Body b2 = m_body2;
		
		final Vec2 temp = tltemp.get();
		final Vec2 pivotCdot = tlpivotCdot.get();
		final Vec2 pivotForce = tlpivotForce.get();
		final Vec2 r1 = tlr1.get();
		final Vec2 r2 = tlr2.get();

		//Vec2 r1 = Mat22.mul(b1.m_xf.R, m_localAnchor1.sub(b1.getMemberLocalCenter()));
		//Vec2 r2 = Mat22.mul(b2.m_xf.R, m_localAnchor2.sub(b2.getMemberLocalCenter()));
		r1.set(b1.getMemberLocalCenter());
		r2.set(b2.getMemberLocalCenter());
		r1.subLocal(m_localAnchor1).negateLocal();
		r2.subLocal(m_localAnchor2).negateLocal();
		Mat22.mulToOut(b1.m_xf.R, r1, r1);
		Mat22.mulToOut(b2.m_xf.R, r2, r2);

		// Solve point-to-point constraint
		//Vec2 pivotCdot = b2.m_linearVelocity.add( Vec2.cross(b2.m_angularVelocity, r2).subLocal(b1.m_linearVelocity).subLocal(Vec2.cross(b1.m_angularVelocity, r1)));
		//Vec2 pivotForce = Mat22.mul(m_pivotMass, pivotCdot).mulLocal(-step.inv_dt);
		Vec2.crossToOut(b1.m_angularVelocity, r1, temp);
		Vec2.crossToOut(b2.m_angularVelocity, r2, pivotCdot);
		pivotCdot.subLocal(b1.m_linearVelocity).subLocal(temp).addLocal(b2.m_linearVelocity);

		Mat22.mulToOut(m_pivotMass, pivotCdot, pivotForce);
		pivotForce.mulLocal(-step.inv_dt);

		//if (!step.warmStarting) m_pivotForce.set(pivotForce);
		//else m_pivotForce.addLocal(pivotForce);
		//if (step.warmStarting && (!m_warmStartingOld)) m_pivotForce = m_lastWarmStartingPivotForce;
		if (step.warmStarting) {
			m_pivotForce.addLocal(pivotForce);
			m_lastWarmStartingPivotForce.set(m_pivotForce);
		} else {
			m_pivotForce.set(m_lastWarmStartingPivotForce);
		}

		//Vec2 P = pivotForce.mul(step.dt);
		final Vec2 P = pivotForce.mulLocal(step.dt);

		b1.m_linearVelocity.x -= b1.m_invMass * P.x;
		b1.m_linearVelocity.y -= b1.m_invMass * P.y;
		b1.m_angularVelocity -= b1.m_invI * Vec2.cross(r1, P);

		b2.m_linearVelocity.x += b2.m_invMass * P.x;
		b2.m_linearVelocity.y += b2.m_invMass * P.y;
		b2.m_angularVelocity += b2.m_invI * Vec2.cross(r2, P);

		if (m_enableMotor && m_limitState != LimitState.EQUAL_LIMITS) {
			final float motorCdot = b2.m_angularVelocity - b1.m_angularVelocity - m_motorSpeed;
			float motorForce = -step.inv_dt * m_motorMass * motorCdot;
			final float oldMotorForce = m_motorForce;
			m_motorForce = MathUtils.clamp(m_motorForce + motorForce, -m_maxMotorTorque, m_maxMotorTorque);
			motorForce = m_motorForce - oldMotorForce;

			if (!step.warmStarting) {
				m_motorForce = oldMotorForce;
			}

			final float P2 = step.dt * motorForce;
			b1.m_angularVelocity -= b1.m_invI * P2;
			b2.m_angularVelocity += b2.m_invI * P2;
		}

		if (m_enableLimit && m_limitState != LimitState.INACTIVE_LIMIT) {
			final float limitCdot = b2.m_angularVelocity - b1.m_angularVelocity;
			float limitForce = -step.inv_dt * m_motorMass * limitCdot;

			if (m_limitState == LimitState.EQUAL_LIMITS) {
				m_limitForce += limitForce;
			} else if (m_limitState == LimitState.AT_LOWER_LIMIT) {
				final float oldLimitForce = m_limitForce;
				m_limitForce = MathUtils.max(m_limitForce + limitForce, 0.0f);
				limitForce = m_limitForce - oldLimitForce;
			} else if (m_limitState == LimitState.AT_UPPER_LIMIT) {
				final float oldLimitForce = m_limitForce;
				m_limitForce = MathUtils.min(m_limitForce + limitForce, 0.0f);
				limitForce = m_limitForce - oldLimitForce;
			}

			final float P2 = step.dt * limitForce;
			b1.m_angularVelocity -= b1.m_invI * P2;
			b2.m_angularVelocity += b2.m_invI * P2;
		}
	}

	// djm pooled, some from above
	private static final TLVec2 tlp1 = new TLVec2();
	private static final TLVec2 tlp2 = new TLVec2();
	private static final TLVec2 tlptpC = new TLVec2();
	private static final TLVec2 tlimpulse = new TLVec2();
	@Override
	public boolean solvePositionConstraints() {
		final Body b1 = m_body1;
		final Body b2 = m_body2;

		final Vec2 p1 = tlp1.get();
		final Vec2 p2 = tlp2.get();
		final Vec2 ptpC = tlptpC.get();
		final Vec2 impulse = tlimpulse.get();
		final Vec2 r1 = tlr1.get();
		final Vec2 r2 = tlr2.get();
		final Mat22 K1 = tlK1.get();
		final Mat22 K2 = tlK2.get();
		final Mat22 K3 = tlK3.get();
		
		float positionError = 0f;

		// Solve point-to-point position error.
		//Vec2 r1 = Mat22.mul(b1.m_xf.R, m_localAnchor1.sub(b1.getMemberLocalCenter()));
		//Vec2 r2 = Mat22.mul(b2.m_xf.R, m_localAnchor2.sub(b2.getMemberLocalCenter()));
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
		ptpC.set(p2);
		ptpC.subLocal(p1);

		positionError = ptpC.length();

		// Prevent overly large corrections.
		//public b2Vec2 dpMax(b2_maxLinearCorrection, b2_maxLinearCorrection);
		//ptpC = b2Clamp(ptpC, -dpMax, dpMax);

		final float invMass1 = b1.m_invMass, invMass2 = b2.m_invMass;
		final float invI1 = b1.m_invI, invI2 = b2.m_invI;

		K1.col1.x = invMass1 + invMass2;    K1.col2.x = 0.0f;
		K1.col1.y = 0.0f;                   K1.col2.y = invMass1 + invMass2;

		K2.col1.x =  invI1 * r1.y * r1.y;   K2.col2.x = -invI1 * r1.x * r1.y;
		K2.col1.y = -invI1 * r1.x * r1.y;   K2.col2.y =  invI1 * r1.x * r1.x;

		K3.col1.x =  invI2 * r2.y * r2.y;   K3.col2.x = -invI2 * r2.x * r2.y;
		K3.col1.y = -invI2 * r2.x * r2.y;   K3.col2.y =  invI2 * r2.x * r2.x;

		final Mat22 K = K1.addLocal(K2).addLocal(K3);
		K.solveToOut(ptpC.negateLocal(), impulse);

		b1.m_sweep.c.x -= b1.m_invMass * impulse.x;
		b1.m_sweep.c.y -= b1.m_invMass * impulse.y;
		b1.m_sweep.a -= b1.m_invI * Vec2.cross(r1, impulse);

		b2.m_sweep.c.x += b2.m_invMass * impulse.x;
		b2.m_sweep.c.y += b2.m_invMass * impulse.y;
		b2.m_sweep.a += b2.m_invI * Vec2.cross(r2, impulse);

		b1.synchronizeTransform();
		b2.synchronizeTransform();

		// Handle limits.
		float angularError = 0.0f;


		if (m_enableLimit && m_limitState != LimitState.INACTIVE_LIMIT) {
			final float angle = b2.m_sweep.a - b1.m_sweep.a - m_referenceAngle;
			float limitImpulse = 0.0f;

			if (m_limitState == LimitState.EQUAL_LIMITS) {
				// Prevent large angular corrections
				final float limitC = MathUtils.clamp(angle, -Settings.maxAngularCorrection, Settings.maxAngularCorrection);
				limitImpulse = -m_motorMass * limitC;
				angularError = MathUtils.abs(limitC);
			} else if (m_limitState == LimitState.AT_LOWER_LIMIT) {
				float limitC = angle - m_lowerAngle;
				angularError = MathUtils.max(0.0f, -limitC);

				// Prevent large angular corrections and allow some slop.
				limitC = MathUtils.clamp(limitC + Settings.angularSlop, -Settings.maxAngularCorrection, 0.0f);
				limitImpulse = -m_motorMass * limitC;
				final float oldLimitImpulse = m_limitPositionImpulse;
				m_limitPositionImpulse = MathUtils.max(m_limitPositionImpulse + limitImpulse, 0.0f);
				limitImpulse = m_limitPositionImpulse - oldLimitImpulse;
			} else if (m_limitState == LimitState.AT_UPPER_LIMIT) {
				float limitC = angle - m_upperAngle;
				angularError = MathUtils.max(0.0f, limitC);

				// Prevent large angular corrections and allow some slop.
				limitC = MathUtils.clamp(limitC - Settings.angularSlop, 0.0f, Settings.maxAngularCorrection);
				limitImpulse = -m_motorMass * limitC;
				final float oldLimitImpulse = m_limitPositionImpulse;
				m_limitPositionImpulse = MathUtils.min(m_limitPositionImpulse + limitImpulse, 0.0f);
				limitImpulse = m_limitPositionImpulse - oldLimitImpulse;
			}

			b1.m_sweep.a -= b1.m_invI * limitImpulse;
			b2.m_sweep.a += b2.m_invI * limitImpulse;

			b1.synchronizeTransform();
			b2.synchronizeTransform();
		}

		return positionError <= Settings.linearSlop && angularError <= Settings.angularSlop;
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
		return m_pivotForce;
	}

	@Override
	public float getReactionTorque() {
		return m_limitForce;
	}

	public float getJointAngle() {
		final Body b1 = m_body1;
		final Body b2 = m_body2;
		return b2.m_sweep.a - b1.m_sweep.a - m_referenceAngle;
	}

	public float getJointSpeed() {
		final Body b1 = m_body1;
		final Body b2 = m_body2;
		return b2.m_angularVelocity - b1.m_angularVelocity;
	}

	public boolean isMotorEnabled() {
		return m_enableMotor;
	}

	public void enableMotor(final boolean flag) {
		m_enableMotor = flag;
	}

	public float getMotorTorque() {
		return m_motorForce;
	}

	public void setMotorSpeed(final float speed) {
		m_motorSpeed = speed;
	}

	public void setMaxMotorTorque(final float torque) {
		m_maxMotorTorque = torque;
	}

	public boolean isLimitEnabled() {
		return m_enableLimit;
	}

	public void enableLimit(final boolean flag) {
		m_enableLimit = flag;
	}

	public float getLowerLimit() {
		return m_lowerAngle;
	}

	public float getUpperLimit() {
		return m_upperAngle;
	}

	public void setLimits(final float lower, final float upper) {
		assert(lower <= upper);
		m_lowerAngle = lower;
		m_upperAngle = upper;
	}
}
