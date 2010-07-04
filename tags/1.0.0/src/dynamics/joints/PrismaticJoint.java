package dynamics.joints;

import common.MathUtils;
import common.Settings;
import common.Vec2;
import dynamics.Body;

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

public class PrismaticJoint extends Joint {

    Vec2 m_localAnchor1;

    Vec2 m_localAnchor2;

    Vec2 m_localXAxis1;

    Vec2 m_localYAxis1;

    float m_initialAngle;

    Jacobian m_linearJacobian;

    float m_linearMass; // effective mass for point-to-line constraint.

    float m_linearImpulse;

    float m_angularMass; // effective mass for angular constraint.

    float m_angularImpulse;

    Jacobian m_motorJacobian;

    float m_motorMass; // effective mass for motor/limit translational

    // constraint.
    float m_motorImpulse;

    float m_limitImpulse;

    float m_limitPositionImpulse;

    float m_lowerTranslation;

    float m_upperTranslation;

    float m_maxMotorForce;

    public float m_motorSpeed;

    public boolean m_enableLimit;

    public boolean m_enableMotor;

    LimitState m_limitState;

    public PrismaticJoint(PrismaticJointDescription description) {
        super(description);
        m_localAnchor1 = m_body1.m_R.mulT(description.anchorPoint
                .sub(m_body1.m_position));
        m_localAnchor2 = m_body2.m_R.mulT(description.anchorPoint
                .sub(m_body2.m_position));
        m_localXAxis1 = m_body1.m_R.mulT(description.axis);
        m_localYAxis1 = Vec2.cross(1.0f, m_localXAxis1);
        m_initialAngle = m_body2.m_rotation - m_body1.m_rotation;

        m_linearJacobian = new Jacobian();
        m_linearMass = 0.0f;
        m_linearImpulse = 0.0f;

        m_angularMass = 0.0f;
        m_angularImpulse = 0.0f;

        m_motorJacobian = new Jacobian();
        m_motorMass = 0.0f;
        m_motorImpulse = 0.0f;
        m_limitImpulse = 0.0f;
        m_limitPositionImpulse = 0.0f;

        m_lowerTranslation = description.lowerTranslation;
        m_upperTranslation = description.upperTranslation;
        m_maxMotorForce = description.motorForce;
        m_motorSpeed = description.motorSpeed;
        m_enableLimit = description.enableLimit;
        m_enableMotor = description.enableMotor;
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

    public void PreSolve() {
        Body b1 = m_body1;
        Body b2 = m_body2;

        // Compute the effective masses.
        Vec2 r1 = b1.m_R.mul(m_localAnchor1);
        Vec2 r2 = b2.m_R.mul(m_localAnchor2);

        float invMass1 = b1.m_invMass, invMass2 = b2.m_invMass;
        float invI1 = b1.m_invI, invI2 = b2.m_invI;

        // Compute point to line constraint effective mass.
        // J = [-ay1 -cross(d+r1,ay1) ay1 cross(r2,ay1)]
        Vec2 ay1 = b1.m_R.mul(m_localYAxis1);
        Vec2 e = b2.m_position.clone().addLocal(r2).subLocal(b1.m_position);

        m_linearJacobian.set(ay1.negate(), -Vec2.cross(e, ay1), ay1, Vec2
                .cross(r2, ay1));
        m_linearMass = invMass1 + invI1 * m_linearJacobian.angular1
                * m_linearJacobian.angular1 + invMass2 + invI2
                * m_linearJacobian.angular2 * m_linearJacobian.angular2;

        assert m_linearMass > Settings.EPSILON;

        m_linearMass = 1.0f / m_linearMass;

        // Compute angular constraint effective mass.
        m_angularMass = 1.0f / (invI1 + invI2);

        // Compute motor and limit terms.
        if (m_enableLimit || m_enableMotor) {
            // The motor and limit share a Jacobian and effective mass.
            Vec2 ax1 = b1.m_R.mul(m_localXAxis1);
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
                    m_limitState = LimitState.equalLimits;
                }
                else if (jointTranslation <= m_lowerTranslation) {
                    if (m_limitState != LimitState.atLowerLimit) {
                        m_limitImpulse = 0.0f;
                    }
                    m_limitState = LimitState.atLowerLimit;
                }
                else if (jointTranslation >= m_upperTranslation) {
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
        }

        if (m_enableMotor == false) {
            m_motorImpulse = 0.0f;
        }

        if (m_enableLimit == false) {
            m_limitImpulse = 0.0f;
        }

        // Warm starting.
        // Vec2 P1 = m_linearJacobian.linear1.mul(m_linearImpulse).add(
        // m_motorJacobian.linear1.mul(m_motorImpulse + m_limitImpulse));
        // Vec2 P2 = m_linearJacobian.linear2.mul(m_linearImpulse).add(
        // m_motorJacobian.linear2.mul(m_motorImpulse + m_limitImpulse));
        Vec2 P1 = m_linearJacobian.linear1.mul(m_linearImpulse).addLocal(
                m_motorJacobian.linear1.mul(m_motorImpulse + m_limitImpulse));
        Vec2 P2 = m_linearJacobian.linear2.mul(m_linearImpulse).addLocal(
                m_motorJacobian.linear2.mul(m_motorImpulse + m_limitImpulse));
        float L1 = m_linearImpulse * m_linearJacobian.angular1
                - m_angularImpulse + (m_motorImpulse + m_limitImpulse)
                * m_motorJacobian.angular1;
        float L2 = m_linearImpulse * m_linearJacobian.angular2
                + m_angularImpulse + (m_motorImpulse + m_limitImpulse)
                * m_motorJacobian.angular2;

        b1.m_linearVelocity.addLocal(P1.mul(invMass1));
        b1.m_angularVelocity += invI1 * L1;

        b2.m_linearVelocity.addLocal(P2.mul(invMass2));
        b2.m_angularVelocity += invI2 * L2;

        m_limitPositionImpulse = 0.0f;
    }

    public void SolveVelocityConstraints(float dt) {
        Body b1 = m_body1;
        Body b2 = m_body2;

        float invMass1 = b1.m_invMass, invMass2 = b2.m_invMass;
        float invI1 = b1.m_invI, invI2 = b2.m_invI;

        // Solve linear constraint.
        float linearCdot = m_linearJacobian
                .Compute(b1.m_linearVelocity, b1.m_angularVelocity,
                        b2.m_linearVelocity, b2.m_angularVelocity);
        float linearImpulse = -m_linearMass * linearCdot;
        m_linearImpulse += linearImpulse;

        b1.m_linearVelocity.addLocal(m_linearJacobian.linear1.mul(invMass1
                * linearImpulse));
        b1.m_angularVelocity += invI1 * linearImpulse
                * m_linearJacobian.angular1;

        b2.m_linearVelocity.addLocal(m_linearJacobian.linear2.mul(invMass2
                * linearImpulse));
        b2.m_angularVelocity += invI2 * linearImpulse
                * m_linearJacobian.angular2;

        // Solve angular constraint.
        float angularCdot = b2.m_angularVelocity - b1.m_angularVelocity;
        float angularImpulse = -m_angularMass * angularCdot;
        m_angularImpulse += angularImpulse;

        b1.m_angularVelocity -= invI1 * angularImpulse;
        b2.m_angularVelocity += invI2 * angularImpulse;

        // Solve linear motor constraint.
        if (m_enableMotor && m_limitState != LimitState.equalLimits) {
            float motorCdot = m_motorJacobian.Compute(b1.m_linearVelocity,
                    b1.m_angularVelocity, b2.m_linearVelocity,
                    b2.m_angularVelocity)
                    - m_motorSpeed;
            float motorImpulse = -m_motorMass * motorCdot;
            float oldMotorImpulse = m_motorImpulse;
            m_motorImpulse = MathUtils.clamp(m_motorImpulse + motorImpulse, -dt
                    * m_maxMotorForce, dt * m_maxMotorForce);
            motorImpulse = m_motorImpulse - oldMotorImpulse;

            b1.m_linearVelocity.addLocal(m_motorJacobian.linear1.mul(invMass1
                    * motorImpulse));
            b1.m_angularVelocity += invI1 * motorImpulse
                    * m_motorJacobian.angular1;

            b2.m_linearVelocity.addLocal(m_motorJacobian.linear2.mul(invMass2
                    * motorImpulse));
            b2.m_angularVelocity += invI2 * motorImpulse
                    * m_motorJacobian.angular2;
        }

        // Solve linear limit constraint.
        if (m_enableLimit && m_limitState != LimitState.inactiveLimit) {
            float limitCdot = m_motorJacobian.Compute(b1.m_linearVelocity,
                    b1.m_angularVelocity, b2.m_linearVelocity,
                    b2.m_angularVelocity);
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

            b1.m_linearVelocity.addLocal(m_motorJacobian.linear1.mul(invMass1
                    * limitImpulse));
            b1.m_angularVelocity += invI1 * limitImpulse
                    * m_motorJacobian.angular1;

            b2.m_linearVelocity.addLocal(m_motorJacobian.linear2.mul(invMass2
                    * limitImpulse));
            b2.m_angularVelocity += invI2 * limitImpulse
                    * m_motorJacobian.angular2;
        }
    }

    public boolean SolvePositionConstraints() {
        Body b1 = m_body1;
        Body b2 = m_body2;

        float invMass1 = b1.m_invMass, invMass2 = b2.m_invMass;
        float invI1 = b1.m_invI, invI2 = b2.m_invI;

        Vec2 r1 = b1.m_R.mul(m_localAnchor1);
        Vec2 r2 = b2.m_R.mul(m_localAnchor2);
        Vec2 p1 = b1.m_position.add(r1);
        Vec2 p2 = b2.m_position.add(r2);
        Vec2 d = p2.sub(p1);
        Vec2 ay1 = b1.m_R.mul(m_localYAxis1);

        // Solve linear (point-to-line) constraint.
        float linearC = Vec2.dot(ay1, d);
        // Prevent overly large corrections.
        linearC = MathUtils.clamp(linearC,
                (float) -Settings.maxLinearCorrection,
                (float) Settings.maxLinearCorrection);

        float linearImpulse = -m_linearMass * linearC;

        b1.m_position.addLocal(m_linearJacobian.linear1.mul(invMass1
                * linearImpulse));
        b1.m_rotation += invI1 * linearImpulse * m_linearJacobian.angular1;
        // b1.m_R.Set(b1.m_rotation); // updated by angular constraint
        b2.m_position.addLocal(m_linearJacobian.linear2.mul(invMass2
                * linearImpulse));
        b2.m_rotation += invI2 * linearImpulse * m_linearJacobian.angular2;
        // b2.m_R.Set(b2.m_rotation); // updated by angular constraint

        float positionError = Math.abs(linearC);

        // Solve angular constraint.
        float angularC = b2.m_rotation - b1.m_rotation - m_initialAngle;
        // Prevent overly large corrections.
        angularC = MathUtils.clamp(angularC,
                (float) -Settings.maxAngularCorrection,
                (float) Settings.maxAngularCorrection);
        float angularImpulse = -m_angularMass * angularC;

        b1.m_rotation -= b1.m_invI * angularImpulse;
        b1.m_R.set(b1.m_rotation);
        b2.m_rotation += b2.m_invI * angularImpulse;
        b2.m_R.set(b2.m_rotation);

        float angularError = Math.abs(angularC);

        // Solve linear limit constraint.
        if (m_enableLimit && m_limitState != LimitState.inactiveLimit) {
            Vec2 rr1 = b1.m_R.mul(m_localAnchor1);
            Vec2 rr2 = b2.m_R.mul(m_localAnchor2);
            Vec2 pp1 = b1.m_position.add(rr1);
            Vec2 pp2 = b2.m_position.add(rr2);
            Vec2 dd = pp2.sub(pp1);
            Vec2 ax1 = b1.m_R.mul(m_localXAxis1);

            float translation = Vec2.dot(ax1, dd);
            float limitImpulse = 0.0f;

            if (m_limitState == LimitState.equalLimits) {
                // Prevent large angular corrections
                float limitC = MathUtils.clamp(translation,
                        -Settings.maxLinearCorrection,
                        Settings.maxLinearCorrection);
                limitImpulse = -m_motorMass * limitC;
                positionError = Math.max(positionError, Math.abs(angularC));
            }
            else if (m_limitState == LimitState.atLowerLimit) {
                // Prevent large angular corrections
                float limitC = MathUtils.clamp(
                        translation - m_lowerTranslation,
                        -Settings.maxLinearCorrection,
                        Settings.maxLinearCorrection);
                limitImpulse = -m_motorMass * (limitC + Settings.linearSlop);
                float oldLimitImpulse = m_limitPositionImpulse;
                m_limitPositionImpulse = Math.max(m_limitPositionImpulse
                        + limitImpulse, 0.0f);
                limitImpulse = m_limitPositionImpulse - oldLimitImpulse;
                positionError = Math.max(positionError, -limitC);
            }
            else if (m_limitState == LimitState.atUpperLimit) {
                // Prevent large angular corrections
                float limitC = MathUtils.clamp(
                        translation - m_upperTranslation,
                        -Settings.maxLinearCorrection,
                        Settings.maxLinearCorrection);
                limitImpulse = -m_motorMass * (limitC - Settings.linearSlop);
                float oldLimitImpulse = m_limitPositionImpulse;
                m_limitPositionImpulse = Math.min(m_limitPositionImpulse
                        + limitImpulse, 0.0f);
                limitImpulse = m_limitPositionImpulse - oldLimitImpulse;
                positionError = Math.max(positionError, limitC);
            }

            b1.m_position.addLocal(m_motorJacobian.linear1.mul(invMass1
                    * limitImpulse));
            b1.m_rotation += invI1 * limitImpulse * m_motorJacobian.angular1;
            b1.m_R.set(b1.m_rotation);
            b2.m_position.addLocal(m_motorJacobian.linear2.mul(invMass2
                    * limitImpulse));
            b2.m_rotation += invI2 * limitImpulse * m_motorJacobian.angular2;
            b2.m_R.set(b2.m_rotation);
        }

        return positionError <= Settings.linearSlop
                && angularError <= Settings.angularSlop;
    }

    float GetJointTranslation() {
        Body b1 = m_body1;
        Body b2 = m_body2;

        Vec2 r1 = b1.m_R.mul(m_localAnchor1);
        Vec2 r2 = b2.m_R.mul(m_localAnchor2);
        Vec2 p1 = b1.m_position.add(r1);
        Vec2 p2 = b2.m_position.add(r2);
        Vec2 d = p2.sub(p1);
        Vec2 ax1 = b1.m_R.mul(m_localXAxis1);

        float translation = Vec2.dot(ax1, d);
        return translation;
    }

    float GetJointSpeed() {
        Body b1 = m_body1;
        Body b2 = m_body2;

        Vec2 r1 = b1.m_R.mul(m_localAnchor1);
        Vec2 r2 = b2.m_R.mul(m_localAnchor2);
        Vec2 p1 = b1.m_position.add(r1);
        Vec2 p2 = b2.m_position.add(r2);
        Vec2 d = p2.sub(p1);
        Vec2 ax1 = b1.m_R.mul(m_localXAxis1);

        Vec2 v1 = b1.m_linearVelocity;
        Vec2 v2 = b2.m_linearVelocity;
        float w1 = b1.m_angularVelocity;
        float w2 = b2.m_angularVelocity;

        float speed = Vec2.dot(d, Vec2.cross(w1, ax1))
                + Vec2.dot(ax1, v2.add(Vec2.cross(w2, r2)).sub(v1).sub(
                        Vec2.cross(w1, r1)));
        return speed;
    }

    float GetMotorForce(float invTimeStep) {
        return m_motorImpulse * invTimeStep;
    }
}
