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


//Updated to rev 56->130 of b2PrismaticJoint.cpp/.h

//Linear constraint (point-to-line)
//d = p2 - p1 = x2 + r2 - x1 - r1
//C = dot(ay1, d)
//Cdot = dot(d, cross(w1, ay1)) + dot(ay1, v2 + cross(w2, r2) - v1 - cross(w1, r1))
//   = -dot(ay1, v1) - dot(cross(d + r1, ay1), w1) + dot(ay1, v2) + dot(cross(r2, ay1), v2)
//J = [-ay1 -cross(d+r1,ay1) ay1 cross(r2,ay1)]
//
//Angular constraint
//C = a2 - a1 + a_initial
//Cdot = w2 - w1
//J = [0 0 -1 0 0 1]

/**
 * A prismatic joint. This joint provides one degree of freedom: translation
 * along an axis fixed in body1. Relative rotation is prevented. You can
 * use a joint limit to restrict the range of motion and a joint motor to
 * drive the motion or to model joint friction.
 */
public class PrismaticJoint extends Joint {

	public final Vec2 m_localAnchor1;
	public final Vec2 m_localAnchor2;
	public final Vec2 m_localXAxis1;
	public final Vec2 m_localYAxis1;
	public float m_refAngle;

	public Jacobian m_linearJacobian;
	public float m_linearMass;				// effective mass for point-to-line constraint.
	public float m_force;

	public float m_angularMass;			// effective mass for angular constraint.
	public float m_torque;

	public Jacobian m_motorJacobian;
	public float m_motorMass;			// effective mass for motor/limit translational constraint.
	public float m_motorForce;
	public float m_limitForce;
	public float m_limitPositionImpulse;

	public float m_lowerTranslation;
	public float m_upperTranslation;
	public float m_maxMotorForce;
	public float m_motorSpeed;

	public boolean m_enableLimit;
	public boolean m_enableMotor;
	public LimitState m_limitState;

	public PrismaticJoint(final PrismaticJointDef def) {
		super(def);
		m_localAnchor1 = def.localAnchor1.clone();
		m_localAnchor2 = def.localAnchor2.clone();
		m_localXAxis1 = def.localAxis1.clone();
		m_localYAxis1 = Vec2.cross(1.0f, m_localXAxis1);
		m_refAngle = def.referenceAngle;

		m_linearJacobian = new Jacobian();
		//m_linearJacobian.setZero(); djm not needed
		m_linearMass = 0.0f;
		m_force = 0.0f;

		m_angularMass = 0.0f;
		m_torque = 0.0f;

		m_motorJacobian = new Jacobian();
		//m_motorJacobian.setZero(); djm not needed
		m_motorMass = 0.0f;
		m_motorForce = 0.0f;
		m_limitForce = 0.0f;
		m_limitPositionImpulse = 0.0f;

		m_lowerTranslation = def.lowerTranslation;
		m_upperTranslation = def.upperTranslation;
		m_maxMotorForce = def.maxMotorForce;
		m_motorSpeed = def.motorSpeed;
		m_enableLimit = def.enableLimit;
		m_enableMotor = def.enableMotor;
	}

	// djm pooled
	private static final TLVec2 tlr1 = new TLVec2();
	private static final TLVec2 tlr2 = new TLVec2();
	private static final TLVec2 tlax1 = new TLVec2();
	private static final TLVec2 tlay1 = new TLVec2();
	private static final TLVec2 tle = new TLVec2();
	private static final TLVec2 tlax1Neg = new TLVec2();
	private static final TLVec2 tlay1Neg = new TLVec2();
	private static final TLVec2 tld = new TLVec2();
	
	@Override
	public void initVelocityConstraints(final TimeStep step) {
		final Body b1 = m_body1;
		final Body b2 = m_body2;

		final Vec2 r1 = tlr1.get();
		final Vec2 r2 = tlr2.get();
		final Vec2 ax1 = tlax1.get();
		final Vec2 ay1 = tlay1.get();
		final Vec2 e = tle.get();
		final Vec2 ax1Neg = tlax1Neg.get();
		final Vec2 ay1Neg = tlay1Neg.get();
		final Vec2 d = tld.get();
		
		// Compute the effective masses.
		Mat22.mulToOut(b1.m_xf.R, m_localAnchor1.sub(b1.getMemberLocalCenter()), r1);
		Mat22.mulToOut(b2.m_xf.R, m_localAnchor2.sub(b2.getMemberLocalCenter()), r2);

		final float invMass1 = b1.m_invMass, invMass2 = b2.m_invMass;
		final float invI1 = b1.m_invI, invI2 = b2.m_invI;

		// Compute point to line constraint effective mass.
		// J = [-ay1 -cross(d+r1,ay1) ay1 cross(r2,ay1)]
		Mat22.mulToOut(b1.m_xf.R, m_localYAxis1, ay1);
		e.set(b2.m_sweep.c);
		e.addLocal(r2).subLocal(b1.m_sweep.c);	// e = d + r1

		Vec2.negateToOut( ay1, ay1Neg);

		m_linearJacobian.set(ay1Neg, -Vec2.cross(e, ay1), ay1, Vec2.cross(r2, ay1));

		m_linearMass = invMass1 + invI1 * m_linearJacobian.angular1
		* m_linearJacobian.angular1 + invMass2 + invI2
		* m_linearJacobian.angular2 * m_linearJacobian.angular2;

		assert m_linearMass > Settings.EPSILON;

		m_linearMass = 1.0f / m_linearMass;

		// Compute angular constraint effective mass.
		m_angularMass = invI1 + invI2;
		if (m_angularMass > Settings.EPSILON) {
			m_angularMass = 1.0f / m_angularMass;
		}

		// Compute motor and limit terms.
		if (m_enableLimit || m_enableMotor) {
			// The motor and limit share a Jacobian and effective mass.
			Mat22.mulToOut(b1.m_xf.R, m_localXAxis1, ax1);
			Vec2.negateToOut( ax1, ax1Neg);

			m_motorJacobian.set(ax1Neg, -Vec2.cross(e, ax1), ax1, Vec2.cross(r2, ax1));

			m_motorMass = invMass1 + invI1 * m_motorJacobian.angular1
			* m_motorJacobian.angular1 + invMass2 + invI2
			* m_motorJacobian.angular2 * m_motorJacobian.angular2;
			assert m_motorMass > Settings.EPSILON;
			m_motorMass = 1.0f / m_motorMass;

			if (m_enableLimit) {
				d.set( e);
				d.subLocal( r1);
				//Vec2 d = e.sub(r1); // p2 - p1
				final float jointTranslation = Vec2.dot(ax1, d);

				if (MathUtils.abs(m_upperTranslation - m_lowerTranslation) < 2.0f * Settings.linearSlop) {
					m_limitState = LimitState.EQUAL_LIMITS;
				}
				else if (jointTranslation <= m_lowerTranslation) {
					if (m_limitState != LimitState.AT_LOWER_LIMIT) {
						m_limitForce = 0.0f;
					}
					m_limitState = LimitState.AT_LOWER_LIMIT;
				}
				else if (jointTranslation >= m_upperTranslation) {
					if (m_limitState != LimitState.AT_UPPER_LIMIT) {
						m_limitForce = 0.0f;
					}
					m_limitState = LimitState.AT_UPPER_LIMIT;
				}
				else {
					m_limitState = LimitState.INACTIVE_LIMIT;
					m_limitForce = 0.0f;
				}
			}
		}

		if (m_enableMotor == false) {
			m_motorForce = 0.0f;
		}

		if (m_enableLimit == false) {
			m_limitForce = 0.0f;
		}

		if (step.warmStarting){
			//Vec2 P1 = new Vec2( step.dt * (m_force * m_linearJacobian.linear1.x + (m_motorForce + m_limitForce) * m_motorJacobian.linear1.x),
			//					step.dt * (m_force * m_linearJacobian.linear1.y + (m_motorForce + m_limitForce) * m_motorJacobian.linear1.y) );
			//Vec2 P2 = new Vec2( step.dt * (m_force * m_linearJacobian.linear2.x + (m_motorForce + m_limitForce) * m_motorJacobian.linear2.x),
			//					step.dt * (m_force * m_linearJacobian.linear2.y + (m_motorForce + m_limitForce) * m_motorJacobian.linear2.y) );
			final float L1 = step.dt * (m_force * m_linearJacobian.angular1 - m_torque + (m_motorForce + m_limitForce) * m_motorJacobian.angular1);
			final float L2 = step.dt * (m_force * m_linearJacobian.angular2 + m_torque + (m_motorForce + m_limitForce) * m_motorJacobian.angular2);

			//b1.m_linearVelocity.x += invMass1 * P1.x;
			//b1.m_linearVelocity.y += invMass1 * P1.y; djm no vec creation
			b1.m_linearVelocity.x += invMass1 * step.dt * (m_force * m_linearJacobian.linear1.x + (m_motorForce + m_limitForce) * m_motorJacobian.linear1.x);
			b1.m_linearVelocity.y += invMass1 * step.dt * (m_force * m_linearJacobian.linear1.y + (m_motorForce + m_limitForce) * m_motorJacobian.linear1.y);

			b1.m_angularVelocity += invI1 * L1;


			b2.m_linearVelocity.x += invMass2 * step.dt * (m_force * m_linearJacobian.linear2.x + (m_motorForce + m_limitForce) * m_motorJacobian.linear2.x);
			b2.m_linearVelocity.y += invMass2 * step.dt * (m_force * m_linearJacobian.linear2.y + (m_motorForce + m_limitForce) * m_motorJacobian.linear2.y);

			b2.m_angularVelocity += invI2 * L2;
		} else {
			m_force = 0.0f;
			m_torque = 0.0f;
			m_limitForce = 0.0f;
			m_motorForce = 0.0f;
		}

		m_limitPositionImpulse = 0.0f;

	}

	private float m_lastWarmStartingForce, m_lastWarmStartingTorque;

	@Override
	public void solveVelocityConstraints(final TimeStep step) {
		final Body b1 = m_body1;
		final Body b2 = m_body2;

		final float invMass1 = b1.m_invMass, invMass2 = b2.m_invMass;
		final float invI1 = b1.m_invI, invI2 = b2.m_invI;

		// Solve linear constraint.
		final float linearCdot = m_linearJacobian.compute(b1.m_linearVelocity, b1.m_angularVelocity,
		                                                  b2.m_linearVelocity, b2.m_angularVelocity);
		final float force = -step.inv_dt * m_linearMass * linearCdot;

		//m_force += force;
		if (step.warmStarting) {
			m_force += (force);
			m_lastWarmStartingForce = m_force;
		} else {
			m_force = m_lastWarmStartingForce;
		}

		final float P = step.dt * force;
		b1.m_linearVelocity.x += (invMass1 * P) * m_linearJacobian.linear1.x;
		b1.m_linearVelocity.y += (invMass1 * P) * m_linearJacobian.linear1.y;
		b1.m_angularVelocity += invI1 * P * m_linearJacobian.angular1;

		b2.m_linearVelocity.x += (invMass2 * P) * m_linearJacobian.linear2.x;
		b2.m_linearVelocity.y += (invMass2 * P) * m_linearJacobian.linear2.y;
		b2.m_angularVelocity += invI2 * P * m_linearJacobian.angular2;

		// Solve angular constraint.
		final float angularCdot = b2.m_angularVelocity - b1.m_angularVelocity;
		final float torque = -step.inv_dt * m_angularMass * angularCdot;
		m_torque += torque;
		if (step.warmStarting) {
			m_torque += torque;
			m_lastWarmStartingTorque = m_torque;
		} else {
			m_torque = m_lastWarmStartingTorque;
		}

		final float L = step.dt * torque;
		b1.m_angularVelocity -= invI1 * L;
		b2.m_angularVelocity += invI2 * L;

		// Solve linear motor constraint.
		if (m_enableMotor && m_limitState != LimitState.EQUAL_LIMITS) {
			final float motorCdot = m_motorJacobian.compute(b1.m_linearVelocity, b1.m_angularVelocity, b2.m_linearVelocity, b2.m_angularVelocity) - m_motorSpeed;
			float motorForce = -step.inv_dt * m_motorMass * motorCdot;
			final float oldMotorForce = m_motorForce;
			m_motorForce = MathUtils.clamp(m_motorForce + motorForce, -m_maxMotorForce, m_maxMotorForce);
			motorForce = m_motorForce - oldMotorForce;

			final float P2 = step.dt * motorForce;
			b1.m_linearVelocity.x += (invMass1 * P2) * m_motorJacobian.linear1.x;
			b1.m_linearVelocity.y += (invMass1 * P2) * m_motorJacobian.linear1.y;
			b1.m_angularVelocity += invI1 * P2 * m_motorJacobian.angular1;

			b2.m_linearVelocity.x += (invMass2 * P2) * m_motorJacobian.linear2.x;
			b2.m_linearVelocity.y += (invMass2 * P2) * m_motorJacobian.linear2.y;
			b2.m_angularVelocity += invI2 * P2 * m_motorJacobian.angular2;
		}

		// Solve linear limit constraint.
		if (m_enableLimit && m_limitState != LimitState.INACTIVE_LIMIT) {
			final float limitCdot = m_motorJacobian.compute(b1.m_linearVelocity, b1.m_angularVelocity, b2.m_linearVelocity, b2.m_angularVelocity);
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

			b1.m_linearVelocity.x += (invMass1 * P2) * m_motorJacobian.linear1.x;
			b1.m_linearVelocity.y += (invMass1 * P2) * m_motorJacobian.linear1.y;
			b1.m_angularVelocity += invI1 * P2 * m_motorJacobian.angular1;

			b2.m_linearVelocity.x += (invMass2 * P2) * m_motorJacobian.linear2.x;
			b2.m_linearVelocity.y += (invMass2 * P2) * m_motorJacobian.linear2.y;
			b2.m_angularVelocity += invI2 * P2 * m_motorJacobian.angular2;
		}
	}

	// djm pooled, using pool above too
	private static final TLVec2 tltemp = new TLVec2();
	private static final TLVec2 tlp1 = new TLVec2();
	private static final TLVec2 tlp2 = new TLVec2();
	private static final TLVec2 tlr1z = new TLVec2();
	private static final TLVec2 tlr2z = new TLVec2();
	private static final TLVec2 tlp1z = new TLVec2();
	private static final TLVec2 tlp2z = new TLVec2();
	private static final TLVec2 tldz = new TLVec2();
	@Override
	public boolean solvePositionConstraints() {
		final Body b1 = m_body1;
		final Body b2 = m_body2;

		final Vec2 temp = tltemp.get();
		final Vec2 p1 = tlp1.get();
		final Vec2 p2 = tlp2.get();
		final Vec2 r1z = tlr1z.get();
		final Vec2 r2z = tlr2z.get();
		final Vec2 p1z = tlp1z.get();
		final Vec2 p2z = tlp2z.get();
		final Vec2 dz = tldz.get();
		final Vec2 r1 = tlr1.get();
		final Vec2 r2 = tlr2.get();
		final Vec2 d = tld.get();
		final Vec2 ax1 = tlax1.get();
		final Vec2 ay1 = tlay1.get();
		
		final float invMass1 = b1.m_invMass, invMass2 = b2.m_invMass;
		final float invI1 = b1.m_invI, invI2 = b2.m_invI;
		temp.set(m_localAnchor1);
		temp.subLocal(b1.getMemberLocalCenter());
		Mat22.mulToOut(b1.m_xf.R, temp, r1);
		temp.set(m_localAnchor2);
		temp.subLocal(b2.getMemberLocalCenter());
		Mat22.mulToOut(b2.m_xf.R, temp, r2);
		p1.set(b1.m_sweep.c);
		p1.addLocal(r1);
		p2.set(b2.m_sweep.c);
		p2.addLocal(r2);
		d.set(p2);
		d.subLocal(p1);
		Mat22.mulToOut(b1.m_xf.R, m_localYAxis1, ay1);

		// Solve linear (point-to-line) constraint.
		float linearC = Vec2.dot(ay1, d);
		// Prevent overly large corrections.
		linearC = MathUtils.clamp(linearC, -Settings.maxLinearCorrection, Settings.maxLinearCorrection);
		final float linearImpulse = -m_linearMass * linearC;

		b1.m_sweep.c.x += (invMass1 * linearImpulse) * m_linearJacobian.linear1.x;
		b1.m_sweep.c.y += (invMass1 * linearImpulse) * m_linearJacobian.linear1.y;
		b1.m_sweep.a += invI1 * linearImpulse * m_linearJacobian.angular1;
		//b1.SynchronizeTransform(); // updated by angular constraint
		b2.m_sweep.c.x += (invMass2 * linearImpulse) * m_linearJacobian.linear2.x;
		b2.m_sweep.c.y += (invMass2 * linearImpulse) * m_linearJacobian.linear2.y;
		b2.m_sweep.a += invI2 * linearImpulse * m_linearJacobian.angular2;
		//b2.SynchronizeTransform(); // updated by angular constraint

		float positionError = MathUtils.abs(linearC);

		// Solve angular constraint.
		float angularC = b2.m_sweep.a - b1.m_sweep.a - m_refAngle;
		// Prevent overly large corrections.
		angularC = MathUtils.clamp(angularC, -Settings.maxAngularCorrection, Settings.maxAngularCorrection);
		final float angularImpulse = -m_angularMass * angularC;

		b1.m_sweep.a -= b1.m_invI * angularImpulse;
		b2.m_sweep.a += b2.m_invI * angularImpulse;

		b1.synchronizeTransform();
		b2.synchronizeTransform();

		final float angularError = MathUtils.abs(angularC);

		// Solve linear limit constraint.
		if (m_enableLimit && m_limitState != LimitState.INACTIVE_LIMIT)
		{
			/*Vec2 r1z = Mat22.mul(b1.m_xf.R, m_localAnchor1.sub(b1.getMemberLocalCenter()));
    		Vec2 r2z = Mat22.mul(b2.m_xf.R, m_localAnchor2.sub(b2.getMemberLocalCenter()));
    		Vec2 p1z = b1.m_sweep.c.add(r1z);
    		Vec2 p2z = b2.m_sweep.c.add(r2z);
    		Vec2 dz = p2z.sub(p1z);
    		Vec2 ax1 = Mat22.mul(b1.m_xf.R, m_localXAxis1);*/
			temp.set(m_localAnchor1);
			temp.subLocal(b1.getMemberLocalCenter());
			Mat22.mulToOut(b1.m_xf.R, temp, r1z);
			temp.set(m_localAnchor2);
			temp.subLocal(b2.getMemberLocalCenter());
			Mat22.mulToOut(b2.m_xf.R, temp, r2z);

			p1z.set(b1.m_sweep.c);
			p1z.addLocal(r1z);
			p2z.set(b2.m_sweep.c);
			p2z.addLocal(r2z);

			dz.set(p2z);
			dz.subLocal(p1z);
			Mat22.mulToOut(b1.m_xf.R, m_localXAxis1, ax1);

			final float translation = Vec2.dot(ax1, dz);
			float limitImpulse = 0.0f;

			if (m_limitState == LimitState.EQUAL_LIMITS) {
				// Prevent large angular corrections
				final float limitC = MathUtils.clamp(translation, -Settings.maxLinearCorrection, Settings.maxLinearCorrection);
				limitImpulse = -m_motorMass * limitC;
				positionError = MathUtils.max(positionError, MathUtils.abs(angularC));
			} else if (m_limitState == LimitState.AT_LOWER_LIMIT) {
				float limitC = translation - m_lowerTranslation;
				positionError = MathUtils.max(positionError, -limitC);

				// Prevent large linear corrections and allow some slop.
				limitC = MathUtils.clamp(limitC + Settings.linearSlop, -Settings.maxLinearCorrection, 0.0f);
				limitImpulse = -m_motorMass * limitC;
				final float oldLimitImpulse = m_limitPositionImpulse;
				m_limitPositionImpulse = MathUtils.max(m_limitPositionImpulse + limitImpulse, 0.0f);
				limitImpulse = m_limitPositionImpulse - oldLimitImpulse;
			} else if (m_limitState == LimitState.AT_UPPER_LIMIT) {
				float limitC = translation - m_upperTranslation;
				positionError = MathUtils.max(positionError, limitC);

				// Prevent large linear corrections and allow some slop.
				limitC = MathUtils.clamp(limitC - Settings.linearSlop, 0.0f, Settings.maxLinearCorrection);
				limitImpulse = -m_motorMass * limitC;
				final float oldLimitImpulse = m_limitPositionImpulse;
				m_limitPositionImpulse = MathUtils.min(m_limitPositionImpulse + limitImpulse, 0.0f);
				limitImpulse = m_limitPositionImpulse - oldLimitImpulse;
			}

			b1.m_sweep.c.x += (invMass1 * limitImpulse) * m_motorJacobian.linear1.x;
			b1.m_sweep.c.y += (invMass1 * limitImpulse) * m_motorJacobian.linear1.y;
			b1.m_sweep.a += invI1 * limitImpulse * m_motorJacobian.angular1;
			b2.m_sweep.c.x += (invMass2 * limitImpulse) * m_motorJacobian.linear2.x;
			b2.m_sweep.c.y += (invMass2 * limitImpulse) * m_motorJacobian.linear2.y;
			b2.m_sweep.a += invI2 * limitImpulse * m_motorJacobian.angular2;

			b1.synchronizeTransform();
			b2.synchronizeTransform();
		}

		return positionError <= Settings.linearSlop && angularError <= Settings.angularSlop;
	}


	@Override
	public Vec2 getAnchor1() {
		return m_body1.getWorldLocation(m_localAnchor1);
	}

	public void getAnchor1ToOut(final Vec2 out){
		m_body1.getWorldLocationToOut( m_localAnchor1, out);
	}

	@Override
	public Vec2 getAnchor2() {
		return m_body2.getWorldLocation(m_localAnchor2);
	}

	public void getAnchor2ToOut(final Vec2 out){
		m_body2.getWorldLocationToOut( m_localAnchor2, out);
	}

	/// Get the current joint translation, usually in meters.
	// djm pooled, and from above
	public static final TLVec2 tlaxis = new TLVec2();
	public float getJointTranslation() {
		final Body b1 = m_body1;
		final Body b2 = m_body2;

		final Vec2 axis = tlaxis.get();
		final Vec2 p1 = tlp1.get();
		final Vec2 p2 = tlp2.get();
		final Vec2 d = tld.get();
		
		b1.getWorldLocationToOut(m_localAnchor1, p1);
		b2.getWorldLocationToOut(m_localAnchor2, p2);
		d.set(p2);
		d.subLocal(p1);
		b1.getWorldDirectionToOut(m_localXAxis1, axis);

		final float translation = Vec2.dot(d, axis);
		return translation;
	}

	/// Get the current joint translation speed, usually in meters per second.
	// djm pooled, use pool from above
	private static final TLVec2 tlw1xAxis = new TLVec2();
	private static final TLVec2 tlv22 = new TLVec2();
	private static final TLVec2 tlw2xR2 = new TLVec2();
	private static final TLVec2 tlw1xR1 = new TLVec2();
	
	public float getJointSpeed() {
		final Body b1 = m_body1;
		final Body b2 = m_body2;

		final Vec2 r1 = tlr1.get();
		final Vec2 r2 = tlr2.get();
		final Vec2 p1 = tlp1.get();
		final Vec2 p2 = tlp2.get();
		final Vec2 d = tld.get();
		final Vec2 axis = tlaxis.get();
		final Vec2 w1xAxis = tlw1xAxis.get();
		final Vec2 v22 = tlv22.get();
		final Vec2 w2xR2 = tlw2xR2.get();
		final Vec2 w1xR1 = tlw1xR1.get();
		
		Mat22.mulToOut(b1.m_xf.R, m_localAnchor1.sub(b1.getMemberLocalCenter()), r1);
		Mat22.mulToOut(b2.m_xf.R, m_localAnchor2.sub(b2.getMemberLocalCenter()), r2);
		p1.set(b1.m_sweep.c);
		p1.addLocal(r1);
		p2.set(b2.m_sweep.c);
		p2.addLocal(r2);

		d.set(p2);
		d.subLocal(p1);
		b1.getWorldDirectionToOut(m_localXAxis1, axis);

		final Vec2 v1 = b1.m_linearVelocity;
		final Vec2 v2 = b2.m_linearVelocity;
		final float w1 = b1.m_angularVelocity;
		final float w2 = b2.m_angularVelocity;

		Vec2.crossToOut( w1, axis, w1xAxis);
		Vec2.crossToOut( w2, r2, w2xR2);
		Vec2.crossToOut( w1, r1, w1xR1);
		v22.set( v2);
		final float speed = Vec2.dot(d, w1xAxis) + Vec2.dot(axis, v22.addLocal(w2xR2).subLocal(v1).subLocal(w1xR1));
		//float speed = Vec2.dot(d, Vec2.cross(w1, axis)) + Vec2.dot(axis, v2.add(Vec2.cross(w2, r2)).subLocal(v1).subLocal(Vec2.cross(w1, r1)));
		return speed;
	}


	@Override
	public float getReactionTorque() {
		return m_torque;
	}

	// djm a little expensive
	@Override
	public Vec2 getReactionForce() {
		final Vec2 ax1 = Mat22.mul(m_body1.m_xf.R, m_localXAxis1);
		final Vec2 ay1 = Mat22.mul(m_body1.m_xf.R, m_localYAxis1);

		return new Vec2(m_limitForce * ax1.x + m_force * ay1.x,
		                m_limitForce * ax1.y + m_force * ay1.y);
	}

	// djm pooled
	private static final TLVec2 tlreactionAx1 = new TLVec2
	();
	public void getReactionForceToOut(final Vec2 out){
		final Vec2 reactionAx1 = tlreactionAx1.get();
		Mat22.mulToOut(m_body1.m_xf.R, m_localXAxis1, reactionAx1);
		Mat22.mulToOut(m_body1.m_xf.R, m_localYAxis1, out);

		//float tempy = m_limitForce * reactionAx1.x + m_force * out.x;
		out.x = m_limitForce * reactionAx1.x + m_force * out.x;
		out.y = m_limitForce * reactionAx1.y + m_force * out.y;
	}

	/** Is the joint limit enabled? */
	public boolean isLimitEnabled() {
		return m_enableLimit;
	}

	/** Enable/disable the joint limit. */
	public void enableLimit(final boolean flag) {
		m_enableLimit = flag;
	}

	/** Get the lower joint limit, usually in meters. */
	public float getLowerLimit() {
		return m_lowerTranslation;
	}

	/** Get the upper joint limit, usually in meters. */
	public float getUpperLimit() {
		return m_upperTranslation;
	}

	/** Set the joint limits, usually in meters. */
	public void setLimits(final float lower, final float upper) {
		assert(lower <= upper);
		m_lowerTranslation = lower;
		m_upperTranslation = upper;
	}

	/** Is the joint motor enabled? */
	public boolean isMotorEnabled() {
		return m_enableMotor;
	}

	/** Enable/disable the joint motor. */
	public void enableMotor(final boolean flag) {
		m_enableMotor = flag;
	}

	/** Set the motor speed, usually in meters per second. */
	public void setMotorSpeed(final float speed) {
		m_motorSpeed = speed;
	}

	/** Get the motor speed, usually in meters per second. */
	public float getMotorSpeed() {
		return m_motorSpeed;
	}

	/** Set the maximum motor torque, usually in N. */
	public void setMaxMotorForce(final float force) {
		m_maxMotorForce = force;
	}

	/** Get the current motor torque, usually in N. */
	public float getMotorForce() {
		return m_motorForce;
	}
}