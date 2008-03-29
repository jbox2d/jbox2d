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
import org.jbox2d.dynamics.World;


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

	public Vec2 m_localAnchor1;
	public Vec2 m_localAnchor2;
	public Vec2 m_localXAxis1;
	public Vec2 m_localYAxis1;
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

    public PrismaticJoint(PrismaticJointDef def) {
        super(def);
        m_localAnchor1 = def.localAnchor1.clone();
    	m_localAnchor2 = def.localAnchor2.clone();
    	m_localXAxis1 = def.localAxis1.clone();
    	m_localYAxis1 = Vec2.cross(1.0f, m_localXAxis1);
    	m_refAngle = def.referenceAngle;

    	m_linearJacobian = new Jacobian();
    	m_linearJacobian.setZero();
    	m_linearMass = 0.0f;
    	m_force = 0.0f;

    	m_angularMass = 0.0f;
    	m_torque = 0.0f;

    	m_motorJacobian = new Jacobian();
    	m_motorJacobian.setZero();
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

    public void initVelocityConstraints(TimeStep step) {
        Body b1 = m_body1;
        Body b2 = m_body2;

        // Compute the effective masses.
        Vec2 r1 = Mat22.mul(b1.m_xf.R, m_localAnchor1.sub(b1.getLocalCenter()));
    	Vec2 r2 = Mat22.mul(b2.m_xf.R, m_localAnchor2.sub(b2.getLocalCenter()));

        float invMass1 = b1.m_invMass, invMass2 = b2.m_invMass;
        float invI1 = b1.m_invI, invI2 = b2.m_invI;

        // Compute point to line constraint effective mass.
        // J = [-ay1 -cross(d+r1,ay1) ay1 cross(r2,ay1)]
        Vec2 ay1 = Mat22.mul(b1.m_xf.R, m_localYAxis1);
    	Vec2 e = b2.m_sweep.c.add(r2).subLocal(b1.m_sweep.c);	// e = d + r1

        m_linearJacobian.set(ay1.negate(), -Vec2.cross(e, ay1), ay1, Vec2
                .cross(r2, ay1));
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
        	Vec2 ax1 = Mat22.mul(b1.m_xf.R, m_localXAxis1);
    		
            m_motorJacobian.set(ax1.negate(), -Vec2.cross(e, ax1), ax1, Vec2
                    .cross(r2, ax1));
            m_motorMass = invMass1 + invI1 * m_motorJacobian.angular1
                    * m_motorJacobian.angular1 + invMass2 + invI2
                    * m_motorJacobian.angular2 * m_motorJacobian.angular2;
            assert m_motorMass > Settings.EPSILON;
            m_motorMass = 1.0f / m_motorMass;

            if (m_enableLimit) {
                Vec2 d = e.sub(r1); // p2 - p1
                float jointTranslation = Vec2.dot(ax1, d);

                if (Math.abs(m_upperTranslation - m_lowerTranslation) < 2.0f * Settings.linearSlop) {
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

        if (World.ENABLE_WARM_STARTING){
    		Vec2 P1 = new Vec2( step.dt * (m_force * m_linearJacobian.linear1.x + (m_motorForce + m_limitForce) * m_motorJacobian.linear1.x),
    							step.dt * (m_force * m_linearJacobian.linear1.y + (m_motorForce + m_limitForce) * m_motorJacobian.linear1.y) );
    		Vec2 P2 = new Vec2( step.dt * (m_force * m_linearJacobian.linear2.x + (m_motorForce + m_limitForce) * m_motorJacobian.linear2.x),
    							step.dt * (m_force * m_linearJacobian.linear2.y + (m_motorForce + m_limitForce) * m_motorJacobian.linear2.y) );
    		float L1 = step.dt * (m_force * m_linearJacobian.angular1 - m_torque + (m_motorForce + m_limitForce) * m_motorJacobian.angular1);
    		float L2 = step.dt * (m_force * m_linearJacobian.angular2 + m_torque + (m_motorForce + m_limitForce) * m_motorJacobian.angular2);

    		b1.m_linearVelocity.x += invMass1 * P1.x;
    		b1.m_linearVelocity.y += invMass1 * P1.y;
    		b1.m_angularVelocity += invI1 * L1;

    		b2.m_linearVelocity.x += invMass2 * P2.x;
    		b2.m_linearVelocity.y += invMass2 * P2.y;
    		b2.m_angularVelocity += invI2 * L2;  
        } else {
        	m_force = 0.0f;
    		m_torque = 0.0f;
    		m_limitForce = 0.0f;
    		m_motorForce = 0.0f;
        }
        
        m_limitPositionImpulse = 0.0f;
        
    }

    public void solveVelocityConstraints(TimeStep step) {
        Body b1 = m_body1;
        Body b2 = m_body2;

        float invMass1 = b1.m_invMass, invMass2 = b2.m_invMass;
        float invI1 = b1.m_invI, invI2 = b2.m_invI;

        // Solve linear constraint.
        float linearCdot = m_linearJacobian
                .compute(b1.m_linearVelocity, b1.m_angularVelocity,
                        b2.m_linearVelocity, b2.m_angularVelocity);
        float force = -step.inv_dt * m_linearMass * linearCdot;
        m_force += force;

        float P = step.dt * force;
    	b1.m_linearVelocity.x += (invMass1 * P) * m_linearJacobian.linear1.x;
    	b1.m_linearVelocity.y += (invMass1 * P) * m_linearJacobian.linear1.y;
    	b1.m_angularVelocity += invI1 * P * m_linearJacobian.angular1;

    	b2.m_linearVelocity.x += (invMass2 * P) * m_linearJacobian.linear2.x;
    	b2.m_linearVelocity.y += (invMass2 * P) * m_linearJacobian.linear2.y;
    	b2.m_angularVelocity += invI2 * P * m_linearJacobian.angular2;

    	// Solve angular constraint.
    	float angularCdot = b2.m_angularVelocity - b1.m_angularVelocity;
    	float torque = -step.inv_dt * m_angularMass * angularCdot;
    	m_torque += torque;

    	float L = step.dt * torque;
    	b1.m_angularVelocity -= invI1 * L;
    	b2.m_angularVelocity += invI2 * L;

    	// Solve linear motor constraint.
    	if (m_enableMotor && m_limitState != LimitState.EQUAL_LIMITS) {
    		float motorCdot = m_motorJacobian.compute(b1.m_linearVelocity, b1.m_angularVelocity, b2.m_linearVelocity, b2.m_angularVelocity) - m_motorSpeed;
    		float motorForce = -step.inv_dt * m_motorMass * motorCdot;
    		float oldMotorForce = m_motorForce;
    		m_motorForce = MathUtils.clamp(m_motorForce + motorForce, -m_maxMotorForce, m_maxMotorForce);
    		motorForce = m_motorForce - oldMotorForce;

    		float P2 = step.dt * motorForce;
    		b1.m_linearVelocity.x += (invMass1 * P2) * m_motorJacobian.linear1.x;
    		b1.m_linearVelocity.y += (invMass1 * P2) * m_motorJacobian.linear1.y;
    		b1.m_angularVelocity += invI1 * P2 * m_motorJacobian.angular1;

    		b2.m_linearVelocity.x += (invMass2 * P2) * m_motorJacobian.linear2.x;
    		b2.m_linearVelocity.y += (invMass2 * P2) * m_motorJacobian.linear2.y;
    		b2.m_angularVelocity += invI2 * P2 * m_motorJacobian.angular2;
    	}

    	// Solve linear limit constraint.
    	if (m_enableLimit && m_limitState != LimitState.INACTIVE_LIMIT) {
    		float limitCdot = m_motorJacobian.compute(b1.m_linearVelocity, b1.m_angularVelocity, b2.m_linearVelocity, b2.m_angularVelocity);
    		float limitForce = -step.inv_dt * m_motorMass * limitCdot;

    		if (m_limitState == LimitState.EQUAL_LIMITS) {
    			m_limitForce += limitForce;
    		} else if (m_limitState == LimitState.AT_LOWER_LIMIT) {
    			float oldLimitForce = m_limitForce;
    			m_limitForce = Math.max(m_limitForce + limitForce, 0.0f);
    			limitForce = m_limitForce - oldLimitForce;
    		} else if (m_limitState == LimitState.AT_UPPER_LIMIT) {
    			float oldLimitForce = m_limitForce;
    			m_limitForce = Math.min(m_limitForce + limitForce, 0.0f);
    			limitForce = m_limitForce - oldLimitForce;
    		}

    		float P2 = step.dt * limitForce;

    		b1.m_linearVelocity.x += (invMass1 * P2) * m_motorJacobian.linear1.x;
    		b1.m_linearVelocity.y += (invMass1 * P2) * m_motorJacobian.linear1.y;
    		b1.m_angularVelocity += invI1 * P2 * m_motorJacobian.angular1;

    		b2.m_linearVelocity.x += (invMass2 * P2) * m_motorJacobian.linear2.x;
    		b2.m_linearVelocity.y += (invMass2 * P2) * m_motorJacobian.linear2.y;
    		b2.m_angularVelocity += invI2 * P2 * m_motorJacobian.angular2;
    	}
    }

    public boolean solvePositionConstraints() {
    	Body b1 = m_body1;
    	Body b2 = m_body2;

    	float invMass1 = b1.m_invMass, invMass2 = b2.m_invMass;
    	float invI1 = b1.m_invI, invI2 = b2.m_invI;

    	Vec2 r1 = Mat22.mul(b1.m_xf.R, m_localAnchor1.sub(b1.getLocalCenter()));
    	Vec2 r2 = Mat22.mul(b2.m_xf.R, m_localAnchor2.sub(b2.getLocalCenter()));
    	Vec2 p1 = b1.m_sweep.c.add(r1);
    	Vec2 p2 = b2.m_sweep.c.add(r2);
    	Vec2 d = p2.sub(p1);
    	Vec2 ay1 = Mat22.mul(b1.m_xf.R, m_localYAxis1);

    	// Solve linear (point-to-line) constraint.
    	float linearC = Vec2.dot(ay1, d);
    	// Prevent overly large corrections.
    	linearC = MathUtils.clamp(linearC, -Settings.maxLinearCorrection, Settings.maxLinearCorrection);
    	float linearImpulse = -m_linearMass * linearC;

    	b1.m_sweep.c.x += (invMass1 * linearImpulse) * m_linearJacobian.linear1.x;
    	b1.m_sweep.c.y += (invMass1 * linearImpulse) * m_linearJacobian.linear1.y;
    	b1.m_sweep.a += invI1 * linearImpulse * m_linearJacobian.angular1;
    	//b1.SynchronizeTransform(); // updated by angular constraint
    	b2.m_sweep.c.x += (invMass2 * linearImpulse) * m_linearJacobian.linear2.x;
    	b2.m_sweep.c.y += (invMass2 * linearImpulse) * m_linearJacobian.linear2.y;
    	b2.m_sweep.a += invI2 * linearImpulse * m_linearJacobian.angular2;
    	//b2.SynchronizeTransform(); // updated by angular constraint

    	float positionError = Math.abs(linearC);

    	// Solve angular constraint.
    	float angularC = b2.m_sweep.a - b1.m_sweep.a - m_refAngle;
    	// Prevent overly large corrections.
    	angularC = MathUtils.clamp(angularC, -Settings.maxAngularCorrection, Settings.maxAngularCorrection);
    	float angularImpulse = -m_angularMass * angularC;

    	b1.m_sweep.a -= b1.m_invI * angularImpulse;
    	b2.m_sweep.a += b2.m_invI * angularImpulse;

    	b1.synchronizeTransform();
    	b2.synchronizeTransform();

    	float angularError = Math.abs(angularC);

    	// Solve linear limit constraint.
    	if (m_enableLimit && m_limitState != LimitState.INACTIVE_LIMIT)
    	{
    		Vec2 r1z = Mat22.mul(b1.m_xf.R, m_localAnchor1.sub(b1.getLocalCenter()));
    		Vec2 r2z = Mat22.mul(b2.m_xf.R, m_localAnchor2.sub(b2.getLocalCenter()));
    		Vec2 p1z = b1.m_sweep.c.add(r1z);
    		Vec2 p2z = b2.m_sweep.c.add(r2z);
    		Vec2 dz = p2z.sub(p1z);
    		Vec2 ax1 = Mat22.mul(b1.m_xf.R, m_localXAxis1);

    		float translation = Vec2.dot(ax1, dz);
    		float limitImpulse = 0.0f;

    		if (m_limitState == LimitState.EQUAL_LIMITS) {
    			// Prevent large angular corrections
    			float limitC = MathUtils.clamp(translation, -Settings.maxLinearCorrection, Settings.maxLinearCorrection);
    			limitImpulse = -m_motorMass * limitC;
    			positionError = Math.max(positionError, Math.abs(angularC));
    		} else if (m_limitState == LimitState.AT_LOWER_LIMIT) {
    			float limitC = translation - m_lowerTranslation;
    			positionError = Math.max(positionError, -limitC);

    			// Prevent large linear corrections and allow some slop.
    			limitC = MathUtils.clamp(limitC + Settings.linearSlop, -Settings.maxLinearCorrection, 0.0f);
    			limitImpulse = -m_motorMass * limitC;
    			float oldLimitImpulse = m_limitPositionImpulse;
    			m_limitPositionImpulse = Math.max(m_limitPositionImpulse + limitImpulse, 0.0f);
    			limitImpulse = m_limitPositionImpulse - oldLimitImpulse;
    		} else if (m_limitState == LimitState.AT_UPPER_LIMIT) {
    			float limitC = translation - m_upperTranslation;
    			positionError = Math.max(positionError, limitC);

    			// Prevent large linear corrections and allow some slop.
    			limitC = MathUtils.clamp(limitC - Settings.linearSlop, 0.0f, Settings.maxLinearCorrection);
    			limitImpulse = -m_motorMass * limitC;
    			float oldLimitImpulse = m_limitPositionImpulse;
    			m_limitPositionImpulse = Math.min(m_limitPositionImpulse + limitImpulse, 0.0f);
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
    	return m_body1.getWorldPoint(m_localAnchor1);
    }

    @Override
    public Vec2 getAnchor2() {
    	return m_body2.getWorldPoint(m_localAnchor2);
    }

    /// Get the current joint translation, usually in meters.
    public float getJointTranslation() {
    	Body b1 = m_body1;
    	Body b2 = m_body2;

    	Vec2 p1 = b1.getWorldPoint(m_localAnchor1);
    	Vec2 p2 = b2.getWorldPoint(m_localAnchor2);
    	Vec2 d = p2.sub(p1);
    	Vec2 axis = b1.getWorldVector(m_localXAxis1);

    	float translation = Vec2.dot(d, axis);
    	return translation;
    }

    /// Get the current joint translation speed, usually in meters per second.
	public float getJointSpeed() {
    	Body b1 = m_body1;
    	Body b2 = m_body2;

    	Vec2 r1 = Mat22.mul(b1.m_xf.R, m_localAnchor1.sub(b1.getLocalCenter()));
    	Vec2 r2 = Mat22.mul(b2.m_xf.R, m_localAnchor2.sub(b2.getLocalCenter()));
    	Vec2 p1 = b1.m_sweep.c.add(r1);
    	Vec2 p2 = b2.m_sweep.c.add(r2);
    	Vec2 d = p2.sub(p1);
    	Vec2 axis = b1.getWorldVector(m_localXAxis1);

    	Vec2 v1 = b1.m_linearVelocity;
    	Vec2 v2 = b2.m_linearVelocity;
    	float w1 = b1.m_angularVelocity;
    	float w2 = b2.m_angularVelocity;

    	float speed = Vec2.dot(d, Vec2.cross(w1, axis)) + Vec2.dot(axis, v2.add(Vec2.cross(w2, r2)).subLocal(v1).subLocal(Vec2.cross(w1, r1)));
    	return speed;
    }

	
    public float getReactionTorque() {
    	return m_torque;
    }
    
    public Vec2 getReactionForce() {
    	Vec2 ax1 = Mat22.mul(m_body1.m_xf.R, m_localXAxis1);
    	Vec2 ay1 = Mat22.mul(m_body1.m_xf.R, m_localYAxis1);

    	return new Vec2(m_limitForce * ax1.x + m_force * ay1.x, 
    					m_limitForce * ax1.y + m_force * ay1.y);
    }
    
    /** Is the joint limit enabled? */
    public boolean isLimitEnabled() {
    	return m_enableLimit;
    }

    /** Enable/disable the joint limit. */
    public void enableLimit(boolean flag) {
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
    public void setLimits(float lower, float upper) {
    	assert(lower <= upper);
    	m_lowerTranslation = lower;
    	m_upperTranslation = upper;
    }

    /** Is the joint motor enabled? */
    public boolean isMotorEnabled() {
    	return m_enableMotor;
    }

    /** Enable/disable the joint motor. */
    public void enableMotor(boolean flag) {
    	m_enableMotor = flag;
    }

	/** Set the motor speed, usually in meters per second. */
    public void setMotorSpeed(float speed) {
    	m_motorSpeed = speed;
    }

	/** Get the motor speed, usually in meters per second. */
    public float getMotorSpeed() {
    	return m_motorSpeed;
    }
    
    /** Set the maximum motor torque, usually in N. */
    public void setMaxMotorForce(float force) {
    	m_maxMotorForce = force;
    }
    
    /** Get the current motor torque, usually in N. */
    public float getMotorForce() {
    	return m_motorForce;
    }
}