/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/ 
 * Box2D homepage: http://www.gphysics.com
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
package dynamics.joints;

import common.MathUtils;
import common.Settings;
import common.Vec2;
import common.Mat22;
import dynamics.Body;
import dynamics.TimeStep;
import dynamics.World;

//Updated to rev. 56 of b2RevoluteJoint.cpp/.h

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

    Vec2 m_localAnchor1;

    Vec2 m_localAnchor2;

    Vec2 m_ptpImpulse;

    float m_motorImpulse;

    float m_limitImpulse;

    float m_limitPositionImpulse;

    Mat22 m_ptpMass; // effective mass for point-to-point constraint.

    float m_motorMass; // effective mass for motor/limit angular constraint.

    float m_intialAngle;

    float m_lowerAngle;

    float m_upperAngle;

    public float m_maxMotorTorque;

    public float m_motorSpeed;

    public boolean m_enableLimit;

    public boolean m_enableMotor;

    LimitState m_limitState;

    public RevoluteJoint(RevoluteJointDef description) {
        super(description);
        m_localAnchor1 = m_body1.m_R.mulT(description.anchorPoint
                .sub(m_body1.m_position));
        m_localAnchor2 = m_body2.m_R.mulT(description.anchorPoint
                .sub(m_body2.m_position));

        m_intialAngle = m_body2.m_rotation - m_body1.m_rotation;

        m_ptpImpulse = new Vec2(0.0f, 0.0f);
        m_motorImpulse = 0.0f;
        m_limitImpulse = 0.0f;
        m_limitPositionImpulse = 0.0f;

        m_lowerAngle = description.lowerAngle;
        m_upperAngle = description.upperAngle;
        m_maxMotorTorque = description.motorTorque;
        m_motorSpeed = description.motorSpeed;
        m_enableLimit = description.enableLimit;
        m_enableMotor = description.enableMotor;
    }

    @Override
    public void prepareVelocitySolver() {
        Body b1 = m_body1;
        Body b2 = m_body2;

        // Compute the effective mass matrix.
        Vec2 r1 = b1.m_R.mul(m_localAnchor1);
        Vec2 r2 = b2.m_R.mul(m_localAnchor2);

        // K = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2)
        // * invI2 * skew(r2)]
        // = [1/m1+1/m2 0 ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 *
        // [r1.y*r1.y -r1.x*r1.y]
        // [ 0 1/m1+1/m2] [-r1.x*r1.y r1.x*r1.x] [-r1.x*r1.y r1.x*r1.x]
        float invMass1 = b1.m_invMass, invMass2 = b2.m_invMass;
        float invI1 = b1.m_invI, invI2 = b2.m_invI;

        Mat22 K1 = new Mat22();
        K1.col1.x = invMass1 + invMass2;
        K1.col2.x = 0.0f;
        K1.col1.y = 0.0f;
        K1.col2.y = invMass1 + invMass2;

        Mat22 K2 = new Mat22();
        K2.col1.x = invI1 * r1.y * r1.y;
        K2.col2.x = -invI1 * r1.x * r1.y;
        K2.col1.y = -invI1 * r1.x * r1.y;
        K2.col2.y = invI1 * r1.x * r1.x;

        Mat22 K3 = new Mat22();
        K3.col1.x = invI2 * r2.y * r2.y;
        K3.col2.x = -invI2 * r2.x * r2.y;
        K3.col1.y = -invI2 * r2.x * r2.y;
        K3.col2.y = invI2 * r2.x * r2.x;

        Mat22 K = K1.add(K2).add(K3);
        m_ptpMass = K.invert();

        m_motorMass = 1.0f / (invI1 + invI2);

        if (m_enableMotor == false) {
            m_motorImpulse = 0.0f;
        }

        if (m_enableLimit) {
            float jointAngle = b2.m_rotation - b1.m_rotation - m_intialAngle;
            if (Math.abs(m_upperAngle - m_lowerAngle) < 2.0f * Settings.angularSlop) {
                m_limitState = LimitState.EQUAL_LIMITS;
            }
            else if (jointAngle <= m_lowerAngle) {
                if (m_limitState != LimitState.AT_LOWER_LIMIT) {
                    m_limitImpulse = 0.0f;
                }
                m_limitState = LimitState.AT_LOWER_LIMIT;
            }
            else if (jointAngle >= m_upperAngle) {
                if (m_limitState != LimitState.AT_UPPER_LIMIT) {
                    m_limitImpulse = 0.0f;
                }
                m_limitState = LimitState.AT_UPPER_LIMIT;
            }
            else {
                m_limitState = LimitState.INACTIVE_LIMIT;
                m_limitImpulse = 0.0f;
            }
        }
        else {
            m_limitImpulse = 0.0f;
        }

        if (World.ENABLE_WARM_STARTING) {
            b1.m_linearVelocity.subLocal(m_ptpImpulse.mul(invMass1));
            b1.m_angularVelocity -= invI1
                    * (Vec2.cross(r1, m_ptpImpulse) + m_motorImpulse + m_limitImpulse);

            b2.m_linearVelocity.addLocal(m_ptpImpulse.mul(invMass2));
            b2.m_angularVelocity += invI2
                    * (Vec2.cross(r2, m_ptpImpulse) + m_motorImpulse + m_limitImpulse);
        } else {
            m_ptpImpulse.setZero();
            m_motorImpulse = 0.0f;
            m_limitImpulse = 0.0f;
        }
        m_limitPositionImpulse = 0.0f;
    }

    @Override
    public void solveVelocityConstraints(TimeStep step) {
        Body b1 = m_body1;
        Body b2 = m_body2;

        Vec2 r1 = b1.m_R.mul(m_localAnchor1);
        Vec2 r2 = b2.m_R.mul(m_localAnchor2);

        // Solve point-to-point constraint
        // Vec2 ptpCdot = b2.m_linearVelocity.add(
        // Vec2.cross(b2.m_angularVelocity, r2)).sub(b1.m_linearVelocity)
        // .sub(Vec2.cross(b1.m_angularVelocity, r1));

        Vec2 ptpCdot = b2.m_linearVelocity.clone().addLocal(
                Vec2.cross(b2.m_angularVelocity, r2)).subLocal(
                b1.m_linearVelocity).sub(Vec2.cross(b1.m_angularVelocity, r1));

        Vec2 ptpImpulse = m_ptpMass.mul(ptpCdot).negateLocal();
        m_ptpImpulse.addLocal(ptpImpulse);

        b1.m_linearVelocity.subLocal(ptpImpulse.mul(b1.m_invMass));
        b1.m_angularVelocity -= b1.m_invI * Vec2.cross(r1, ptpImpulse);

        b2.m_linearVelocity.addLocal(ptpImpulse.mul(b2.m_invMass));
        b2.m_angularVelocity += b2.m_invI * Vec2.cross(r2, ptpImpulse);

        if (m_enableMotor && m_limitState != LimitState.EQUAL_LIMITS) {
            float motorCdot = b2.m_angularVelocity - b1.m_angularVelocity
                    - m_motorSpeed;
            float motorImpulse = -m_motorMass * motorCdot;
            float oldMotorImpulse = m_motorImpulse;
            m_motorImpulse = MathUtils.clamp(m_motorImpulse + motorImpulse,
                    -step.dt * m_maxMotorTorque, step.dt * m_maxMotorTorque);
            motorImpulse = m_motorImpulse - oldMotorImpulse;
            b1.m_angularVelocity -= b1.m_invI * motorImpulse;
            b2.m_angularVelocity += b2.m_invI * motorImpulse;
        }

        if (m_enableLimit && m_limitState != LimitState.INACTIVE_LIMIT) {
            float limitCdot = b2.m_angularVelocity - b1.m_angularVelocity;
            float limitImpulse = -m_motorMass * limitCdot;

            if (m_limitState == LimitState.EQUAL_LIMITS) {
                m_limitImpulse += limitImpulse;
            }
            else if (m_limitState == LimitState.AT_LOWER_LIMIT) {
                float oldLimitImpulse = m_limitImpulse;
                m_limitImpulse = Math.max(m_limitImpulse + limitImpulse, 0.0f);
                limitImpulse = m_limitImpulse - oldLimitImpulse;
            }
            else if (m_limitState == LimitState.AT_UPPER_LIMIT) {
                float oldLimitImpulse = m_limitImpulse;
                m_limitImpulse = Math.min(m_limitImpulse + limitImpulse, 0.0f);
                limitImpulse = m_limitImpulse - oldLimitImpulse;
            }

            b1.m_angularVelocity -= b1.m_invI * limitImpulse;
            b2.m_angularVelocity += b2.m_invI * limitImpulse;
        }
    }

    @Override
    public boolean solvePositionConstraints() {
        Body b1 = m_body1;
        Body b2 = m_body2;
        
        float positionError = 0f;

        // Solve point-to-point position error.
        Vec2 r1 = b1.m_R.mul(m_localAnchor1);
        Vec2 r2 = b2.m_R.mul(m_localAnchor2);

        Vec2 p1 = b1.m_position.add(r1);
        Vec2 p2 = b2.m_position.add(r2);
        Vec2 ptpC = p2.sub(p1);
        
        positionError = ptpC.length();

        // Prevent overly large corrections.
//        Vec2 dpMax = new Vec2(Settings.maxLinearCorrection,
//                Settings.maxLinearCorrection);
//        ptpC = MathUtils.clamp(ptpC, dpMax.negate(), dpMax);

        //Vec2 impulse = m_ptpMass.mul(ptpC).negateLocal();

        float invMass1 = b1.m_invMass, invMass2 = b2.m_invMass;
        float invI1 = b1.m_invI, invI2 = b2.m_invI;

        Mat22 K1 = new Mat22();
        K1.col1.x = invMass1 + invMass2;    K1.col2.x = 0.0f;
        K1.col1.y = 0.0f;                   K1.col2.y = invMass1 + invMass2;
        
        Mat22 K2 = new Mat22();
        K2.col1.x =  invI1 * r1.y * r1.y;   K2.col2.x = -invI1 * r1.x * r1.y;
        K2.col1.y = -invI1 * r1.x * r1.y;   K2.col2.y =  invI1 * r1.x * r1.x;

        Mat22 K3 = new Mat22();
        K3.col1.x =  invI2 * r2.y * r2.y;   K3.col2.x = -invI2 * r2.x * r2.y;
        K3.col1.y = -invI2 * r2.x * r2.y;   K3.col2.y =  invI2 * r2.x * r2.x;

        Mat22 K = K1.add(K2).add(K3);
        Vec2 impulse = K.solve(ptpC.negate());

        b1.m_position.subLocal(impulse.mul(b1.m_invMass));
        b1.m_rotation -= b1.m_invI * Vec2.cross(r1, impulse);
        b1.m_R.setAngle(b1.m_rotation);

        b2.m_position.addLocal(impulse.mul(b2.m_invMass));
        b2.m_rotation += b2.m_invI * Vec2.cross(r2, impulse);
        b2.m_R.setAngle(b2.m_rotation);

        //float positionError = ptpC.length();

        // Handle limits.
        float angularError = 0.0f;

        if (m_enableLimit && m_limitState != LimitState.INACTIVE_LIMIT) {
            float angle = b2.m_rotation - b1.m_rotation - m_intialAngle;
            float limitImpulse = 0.0f;

            if (m_limitState == LimitState.EQUAL_LIMITS) {
                // Prevent large angular corrections
                float limitC = MathUtils.clamp(angle,
                        -Settings.maxAngularCorrection,
                        Settings.maxAngularCorrection);
                limitImpulse = -m_motorMass * limitC;
                angularError = Math.abs(limitC);
            }
            else if (m_limitState == LimitState.AT_LOWER_LIMIT) {
                // Prevent large angular corrections
                float limitC = angle - m_lowerAngle;
                angularError = Math.max(0.0f, -limitC);

                // Prevent large angular corrections and allow some slop.
                limitC = MathUtils.clamp(limitC + Settings.angularSlop, -Settings.maxAngularCorrection, 0.0f);
                limitImpulse = -m_motorMass * limitC;
                float oldLimitImpulse = m_limitPositionImpulse;
                m_limitPositionImpulse = Math.max(m_limitPositionImpulse
                        + limitImpulse, 0.0f);
                limitImpulse = m_limitPositionImpulse - oldLimitImpulse;
            }
            else if (m_limitState == LimitState.AT_UPPER_LIMIT) {
                float limitC = angle - m_upperAngle;
                angularError = Math.max(0.0f, limitC);

                // Prevent large angular corrections and allow some slop.
                limitC = MathUtils.clamp(limitC - Settings.angularSlop, 0.0f, Settings.maxAngularCorrection);
                limitImpulse = -m_motorMass * limitC;
                float oldLimitImpulse = m_limitPositionImpulse;
                m_limitPositionImpulse = Math.min(m_limitPositionImpulse
                        + limitImpulse, 0.0f);
                limitImpulse = m_limitPositionImpulse - oldLimitImpulse;
            }

            b1.m_rotation -= b1.m_invI * limitImpulse;
            b1.m_R.setAngle(b1.m_rotation);
            b2.m_rotation += b2.m_invI * limitImpulse;
            b2.m_R.setAngle(b2.m_rotation);
        }

        return positionError <= Settings.linearSlop
                && angularError <= Settings.angularSlop;
    }

    @Override
    public Vec2 getAnchor1() {
        Body b1 = m_body1;
        return b1.m_R.mul(m_localAnchor1).addLocal(b1.m_position);
    }

    @Override
    public Vec2 getAnchor2() {
        Body b2 = m_body2;
        return b2.m_R.mul(m_localAnchor2).addLocal(b2.m_position);
    }

    float getJointAngle() {
        Body b1 = m_body1;
        Body b2 = m_body2;
        return b2.m_rotation - b1.m_rotation;
    }

    float getJointSpeed() {
        Body b1 = m_body1;
        Body b2 = m_body2;
        return b2.m_angularVelocity - b1.m_angularVelocity;
    }

    float getMotorTorque(float invTimeStep) {
        return m_motorImpulse * invTimeStep;
    }
}
