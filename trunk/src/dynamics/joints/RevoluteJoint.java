package dynamics.joints;

import common.MathUtils;
import common.Settings;
import common.Vec2;
import common.Mat22;
import dynamics.Body;

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

    float m_maxMotorTorque;

    float m_motorSpeed;

    public boolean m_enableLimit;

    public boolean m_enableMotor;

    LimitState m_limitState;

    public RevoluteJoint(RevoluteDescription description) {
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
    public void PreSolve() {
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
                m_limitState = LimitState.equalLimits;
            }
            else if (jointAngle <= m_lowerAngle) {
                if (m_limitState != LimitState.atLowerLimit) {
                    m_limitImpulse = 0.0f;
                }
                m_limitState = LimitState.atLowerLimit;
            }
            else if (jointAngle >= m_upperAngle) {
                if (m_limitState != LimitState.atUpperLimit) {
                    m_limitImpulse = 0.0f;
                }
                m_limitState = LimitState.atUpperLimit;
            }
            else {
                m_limitState = LimitState.inactiveLimit;
                m_limitImpulse = 0.0f;
            }
        }
        else {
            m_limitImpulse = 0.0f;
        }

        // Warm starting.
        b1.m_linearVelocity.subLocal(m_ptpImpulse.mul(invMass1));
        b1.m_angularVelocity -= invI1
                * (Vec2.cross(r1, m_ptpImpulse) + m_motorImpulse + m_limitImpulse);

        b2.m_linearVelocity.addLocal(m_ptpImpulse.mul(invMass2));
        b2.m_angularVelocity += invI2
                * (Vec2.cross(r2, m_ptpImpulse) + m_motorImpulse + m_limitImpulse);

        m_limitPositionImpulse = 0.0f;
    }

    @Override
    public void SolveVelocityConstraints(float dt) {
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

        Vec2 ptpImpulse = m_ptpMass.mul(ptpCdot).negate();
        m_ptpImpulse.addLocal(ptpImpulse);

        b1.m_linearVelocity.subLocal(ptpImpulse.mul(b1.m_invMass));
        b1.m_angularVelocity -= b1.m_invI * Vec2.cross(r1, ptpImpulse);

        b2.m_linearVelocity.addLocal(ptpImpulse.mul(b2.m_invMass));
        b2.m_angularVelocity += b2.m_invI * Vec2.cross(r2, ptpImpulse);

        if (m_enableMotor && m_limitState != LimitState.equalLimits) {
            float motorCdot = b2.m_angularVelocity - b1.m_angularVelocity
                    - m_motorSpeed;
            float motorImpulse = -m_motorMass * motorCdot;
            float oldMotorImpulse = m_motorImpulse;
            m_motorImpulse = MathUtils.clamp(m_motorImpulse + motorImpulse, -dt
                    * m_maxMotorTorque, dt * m_maxMotorTorque);
            motorImpulse = m_motorImpulse - oldMotorImpulse;
            b1.m_angularVelocity -= b1.m_invI * motorImpulse;
            b2.m_angularVelocity += b2.m_invI * motorImpulse;
        }

        if (m_enableLimit && m_limitState != LimitState.inactiveLimit) {
            float limitCdot = b2.m_angularVelocity - b1.m_angularVelocity;
            float limitImpulse = -m_motorMass * limitCdot;

            if (m_limitState == LimitState.equalLimits) {
                m_limitImpulse += limitImpulse;
            }
            else if (m_limitState == LimitState.atLowerLimit) {
                float oldLimitImpulse = m_limitImpulse;
                m_limitImpulse = Math.max(m_limitImpulse + limitImpulse, 0.0f);
                limitImpulse = m_limitImpulse - oldLimitImpulse;
            }
            else if (m_limitState == LimitState.atUpperLimit) {
                float oldLimitImpulse = m_limitImpulse;
                m_limitImpulse = Math.min(m_limitImpulse + limitImpulse, 0.0f);
                limitImpulse = m_limitImpulse - oldLimitImpulse;
            }

            b1.m_angularVelocity -= b1.m_invI * limitImpulse;
            b2.m_angularVelocity += b2.m_invI * limitImpulse;
        }
    }

    @Override
    public boolean SolvePositionConstraints() {
        Body b1 = m_body1;
        Body b2 = m_body2;

        // Solve point-to-point position error.
        Vec2 r1 = b1.m_R.mul(m_localAnchor1);
        Vec2 r2 = b2.m_R.mul(m_localAnchor2);

        Vec2 p1 = b1.m_position.add(r1);
        Vec2 p2 = b2.m_position.add(r2);
        Vec2 ptpC = p2.sub(p1);

        // Prevent overly large corrections.
        Vec2 dpMax = new Vec2(Settings.maxLinearCorrection,
                Settings.maxLinearCorrection);
        ptpC = MathUtils.clamp(ptpC, dpMax.negate(), dpMax);

        Vec2 impulse = m_ptpMass.mul(ptpC).negate();

        b1.m_position.subLocal(impulse.mul(b1.m_invMass));
        b1.m_rotation -= b1.m_invI * Vec2.cross(r1, impulse);
        b1.m_R.set(b1.m_rotation);

        b2.m_position.addLocal(impulse.mul(b2.m_invMass));
        b2.m_rotation += b2.m_invI * Vec2.cross(r2, impulse);
        b2.m_R.set(b2.m_rotation);

        float positionError = ptpC.length();

        // Handle limits.
        float angularError = 0.0f;

        if (m_enableLimit && m_limitState != LimitState.inactiveLimit) {
            float angle = b2.m_rotation - b1.m_rotation - m_intialAngle;
            float limitImpulse = 0.0f;

            if (m_limitState == LimitState.equalLimits) {
                // Prevent large angular corrections
                float limitC = MathUtils.clamp(angle,
                        -Settings.maxAngularCorrection,
                        Settings.maxAngularCorrection);
                limitImpulse = -m_motorMass * limitC;
                angularError = Math.abs(limitC);
            }
            else if (m_limitState == LimitState.atLowerLimit) {
                // Prevent large angular corrections
                float limitC = MathUtils.clamp(angle - m_lowerAngle,
                        -Settings.maxAngularCorrection,
                        Settings.maxAngularCorrection);
                limitImpulse = -m_motorMass * (limitC + Settings.angularSlop);
                float oldLimitImpulse = m_limitPositionImpulse;
                m_limitPositionImpulse = Math.max(m_limitPositionImpulse
                        + limitImpulse, 0.0f);
                limitImpulse = m_limitPositionImpulse - oldLimitImpulse;
                angularError = Math.max(0.0f, -limitC);
            }
            else if (m_limitState == LimitState.atUpperLimit) {
                // Prevent large angular corrections
                float limitC = MathUtils.clamp(angle - m_upperAngle,
                        -Settings.maxAngularCorrection,
                        Settings.maxAngularCorrection);
                limitImpulse = -m_motorMass * (limitC - Settings.angularSlop);
                float oldLimitImpulse = m_limitPositionImpulse;
                m_limitPositionImpulse = Math.min(m_limitPositionImpulse
                        + limitImpulse, 0.0f);
                limitImpulse = m_limitPositionImpulse - oldLimitImpulse;
                angularError = Math.max(0.0f, limitC);
            }

            b1.m_rotation -= b1.m_invI * limitImpulse;
            b1.m_R.set(b1.m_rotation);
            b2.m_rotation += b2.m_invI * limitImpulse;
            b2.m_R.set(b2.m_rotation);
        }

        return positionError <= Settings.linearSlop
                && angularError <= Settings.angularSlop;
    }

    @Override
    public Vec2 GetAnchor1() {
        Body b1 = m_body1;
        return b1.m_position.add(b1.m_R.mul(m_localAnchor1));
    }

    @Override
    public Vec2 GetAnchor2() {
        Body b2 = m_body2;
        return b2.m_position.add(b2.m_R.mul(m_localAnchor2));
    }

    float GetJointAngle() {
        Body b1 = m_body1;
        Body b2 = m_body2;
        return b2.m_rotation - b1.m_rotation;
    }

    float GetJointSpeed() {
        Body b1 = m_body1;
        Body b2 = m_body2;
        return b2.m_angularVelocity - b1.m_angularVelocity;
    }

    float GetMotorTorque(float invTimeStep) {
        return m_motorImpulse * invTimeStep;
    }
}
