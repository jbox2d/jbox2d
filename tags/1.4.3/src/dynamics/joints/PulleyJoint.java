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

import common.Settings;
import dynamics.Body;
import dynamics.TimeStep;
import common.Vec2;
import common.MathUtils;

//Updated to rev. 56 of b2PulleyJoint.cpp/.h

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
    public static final float MIN_PULLEY_LENGTH = Settings.lengthUnitsPerMeter;

    public Body m_ground;

    public Vec2 m_groundAnchor1;

    public Vec2 m_groundAnchor2;

    public Vec2 m_localAnchor1;

    public Vec2 m_localAnchor2;

    public Vec2 m_u1;

    public Vec2 m_u2;

    public float m_constant;

    public float m_ratio;

    public float m_maxLength1;

    public float m_maxLength2;

    // Effective masses
    public float m_pulleyMass;

    public float m_limitMass1;

    public float m_limitMass2;

    // Impulses for accumulation/warm starting.
    public float m_pulleyImpulse;

    public float m_limitImpulse1;

    public float m_limitImpulse2;

    // Position impulses for accumulation.
    public float m_limitPositionImpulse1;

    public float m_limitPositionImpulse2;

    public LimitState m_limitState1;

    public LimitState m_limitState2;

    public PulleyJoint(PulleyJointDef def) {
        super(def);
        m_ground = m_body1.m_world.m_groundBody;
        m_groundAnchor1 = def.groundPoint1.sub(m_ground.m_position);
        m_groundAnchor2 = def.groundPoint2.sub(m_ground.m_position);
        m_localAnchor1 = m_body1.m_R.mulT(def.anchorPoint1
                .sub(m_body1.m_position));
        m_localAnchor2 = m_body2.m_R.mulT(def.anchorPoint2
                .sub(m_body2.m_position));

        m_ratio = def.ratio;

        Vec2 d1 = def.groundPoint1.sub(def.anchorPoint1);
        Vec2 d2 = def.groundPoint2.sub(def.anchorPoint2);

        float length1 = Math.max(0.5f * MIN_PULLEY_LENGTH, d1.length());
        float length2 = Math.max(0.5f * MIN_PULLEY_LENGTH, d2.length());

        m_constant = length1 + m_ratio * length2;

        m_maxLength1 = MathUtils.clamp(def.maxLength1, length1, m_constant
                - m_ratio * MIN_PULLEY_LENGTH);
        m_maxLength2 = MathUtils.clamp(def.maxLength2, length2,
                (m_constant - MIN_PULLEY_LENGTH) / m_ratio);

        m_pulleyImpulse = 0.0f;
        m_limitImpulse1 = 0.0f;
        m_limitImpulse2 = 0.0f;
        m_u1 = new Vec2();
        m_u2 = new Vec2();
        m_limitState1 = LimitState.INACTIVE_LIMIT;
        m_limitState2 = LimitState.INACTIVE_LIMIT;
    }

    public void prepareVelocitySolver() {
        Body b1 = m_body1;
        Body b2 = m_body2;

        Vec2 r1 = b1.m_R.mul(m_localAnchor1);
        Vec2 r2 = b2.m_R.mul(m_localAnchor2);

        Vec2 p1 = b1.m_position.add(r1);
        Vec2 p2 = b2.m_position.add(r2);

        Vec2 s1 = m_ground.m_position.add(m_groundAnchor1);
        Vec2 s2 = m_ground.m_position.add(m_groundAnchor2);

        // Get the pulley axes.
        m_u1 = p1.sub(s1);
        m_u2 = p2.sub(s2);

        float length1 = m_u1.length();
        float length2 = m_u2.length();

        // System.out.println(length1+" "+length2+" "+m_constant);
        // System.out.println(m_maxLength1);
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

        if (length1 < m_maxLength1) {
            m_limitState1 = LimitState.INACTIVE_LIMIT;
            m_limitImpulse1 = 0.0f;
        }
        else {
            m_limitState1 = LimitState.AT_UPPER_LIMIT;
            m_limitPositionImpulse1 = 0.0f;
        }

        if (length2 < m_maxLength2) {
            m_limitState2 = LimitState.INACTIVE_LIMIT;
            m_limitImpulse2 = 0.0f;
        }
        else {
            m_limitState2 = LimitState.AT_UPPER_LIMIT;
            m_limitPositionImpulse2 = 0.0f;
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

        // Warm starting.
        Vec2 P1 = m_u1.mul(-m_pulleyImpulse - m_limitImpulse1);
        Vec2 P2 = m_u2.mul(-m_ratio * m_pulleyImpulse - m_limitImpulse2);
        b1.m_linearVelocity.addLocal(P1.mul(b1.m_invMass));
        b1.m_angularVelocity += b1.m_invI * Vec2.cross(r1, P1);
        b2.m_linearVelocity.addLocal(P2.mul(b2.m_invMass));
        b2.m_angularVelocity += b2.m_invI * Vec2.cross(r2, P2);
    }

    public void solveVelocityConstraints(TimeStep step) {
        Body b1 = m_body1;
        Body b2 = m_body2;

        Vec2 r1 = b1.m_R.mul(m_localAnchor1);
        Vec2 r2 = b2.m_R.mul(m_localAnchor2);

        {
            Vec2 v1 = b1.m_linearVelocity.add(Vec2.cross(b1.m_angularVelocity,
                    r1));
            Vec2 v2 = b2.m_linearVelocity.add(Vec2.cross(b2.m_angularVelocity,
                    r2));

            float Cdot = -Vec2.dot(m_u1, v1) - m_ratio * Vec2.dot(m_u2, v2);
            float impulse = -m_pulleyMass * Cdot;
            m_pulleyImpulse += impulse;

            Vec2 P1 = m_u1.mul(-impulse);
            Vec2 P2 = m_u2.mul(-m_ratio * impulse);
            b1.m_linearVelocity.addLocal(P1.mul(b1.m_invMass));
            b1.m_angularVelocity += b1.m_invI * Vec2.cross(r1, P1);
            b2.m_linearVelocity.addLocal(P2.mul(b2.m_invMass));
            b2.m_angularVelocity += b2.m_invI * Vec2.cross(r2, P2);
            // System.out.println(P1.mul(b1.m_invMass).y);
        }

        if (m_limitState1 == LimitState.AT_UPPER_LIMIT) {
            Vec2 v1 = b1.m_linearVelocity.add(Vec2.cross(b1.m_angularVelocity,
                    r1));
            float Cdot = -Vec2.dot(m_u1, v1);
            float impulse = -m_limitMass1 * Cdot;
            float oldLimitImpulse = m_limitImpulse1;
            m_limitImpulse1 = Math.max(0.0f, m_limitImpulse1 + impulse);
            impulse = m_limitImpulse1 - oldLimitImpulse;
            Vec2 P1 = m_u1.mul(-impulse);
            b1.m_linearVelocity.addLocal(P1.mul(b1.m_invMass));
            b1.m_angularVelocity += b1.m_invI * Vec2.cross(r1, P1);
        }

        if (m_limitState2 == LimitState.AT_UPPER_LIMIT) {
            Vec2 v2 = b2.m_linearVelocity.add(Vec2.cross(b2.m_angularVelocity,
                    r2));
            float Cdot = -Vec2.dot(m_u2, v2);
            float impulse = -m_limitMass2 * Cdot;
            float oldLimitImpulse = m_limitImpulse2;
            m_limitImpulse2 = Math.max(0.0f, m_limitImpulse2 + impulse);
            impulse = m_limitImpulse2 - oldLimitImpulse;
            Vec2 P2 = m_u2.mul(-impulse);
            b2.m_linearVelocity.addLocal(P2.mul(b2.m_invMass));
            b2.m_angularVelocity += b2.m_invI * Vec2.cross(r2, P2);
        }

        // System.out.println(m_pulleyImpulse);
    }

    public boolean solvePositionConstraints() {
        // if (true) return false;
        Body b1 = m_body1;
        Body b2 = m_body2;

        Vec2 s1 = m_ground.m_position.add(m_groundAnchor1);
        Vec2 s2 = m_ground.m_position.add(m_groundAnchor2);

        float linearError = 0.0f;

        {
            Vec2 r1 = b1.m_R.mul(m_localAnchor1);
            Vec2 r2 = b2.m_R.mul(m_localAnchor2);

            Vec2 p1 = b1.m_position.add(r1);
            Vec2 p2 = b2.m_position.add(r2);

            // Get the pulley axes.
            m_u1 = p1.sub(s1);
            m_u2 = p2.sub(s2);

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
            linearError = Math.max(linearError, Math.abs(C));
            C = MathUtils.clamp(C, -Settings.maxLinearCorrection,
                    Settings.maxLinearCorrection);
            float impulse = -m_pulleyMass * C;

            Vec2 P1 = m_u1.mul(-impulse);
            Vec2 P2 = m_u2.mul(-m_ratio * impulse);

            b1.m_position.addLocal(P1.mul(b1.m_invMass));
            b1.m_rotation += b1.m_invI * Vec2.cross(r1, P1);
            b2.m_position.addLocal(P2.mul(b2.m_invMass));
            b2.m_rotation += b2.m_invI * Vec2.cross(r2, P2);

            b1.m_R.setAngle(b1.m_rotation);
            b2.m_R.setAngle(b2.m_rotation);
        }

        if (m_limitState1 == LimitState.AT_UPPER_LIMIT) {
            Vec2 r1 = b1.m_R.mul(m_localAnchor1);
            Vec2 p1 = b1.m_position.add(r1);

            m_u1 = p1.sub(s1);
            float length1 = m_u1.length();

            if (length1 > Settings.linearSlop) {
                m_u1.mulLocal(1.0f / length1);
            }
            else {
                m_u1.setZero();
            }

            float C = m_maxLength1 - length1;
            linearError = Math.max(linearError, -C);
            C = MathUtils.clamp(C + Settings.linearSlop,
                    -Settings.maxLinearCorrection, 0.0f);
            float impulse = -m_limitMass1 * C;
            float oldLimitPositionImpulse = m_limitPositionImpulse1;
            m_limitPositionImpulse1 = Math.max(0.0f, m_limitPositionImpulse1
                    + impulse);
            impulse = m_limitPositionImpulse1 - oldLimitPositionImpulse;

            Vec2 P1 = m_u1.mul(-impulse);
            b1.m_position.addLocal(P1.mul(b1.m_invMass));
            b1.m_rotation += b1.m_invI * Vec2.cross(r1, P1);
            b1.m_R.setAngle(b1.m_rotation);
        }

        if (m_limitState2 == LimitState.AT_UPPER_LIMIT) {
            Vec2 r2 = b2.m_R.mul(m_localAnchor2);
            Vec2 p2 = b2.m_position.add(r2);

            m_u2 = p2.sub(s2);
            float length2 = m_u2.length();

            if (length2 > Settings.linearSlop) {
                m_u2.mulLocal(1.0f / length2);
            }
            else {
                m_u2.setZero();
            }

            float C = m_maxLength2 - length2;
            linearError = Math.max(linearError, -C);
            C = MathUtils.clamp(C + Settings.linearSlop,
                    -Settings.maxLinearCorrection, 0.0f);
            float impulse = -m_limitMass2 * C;
            float oldLimitPositionImpulse = m_limitPositionImpulse2;
            m_limitPositionImpulse2 = Math.max(0.0f, m_limitPositionImpulse2
                    + impulse);
            impulse = m_limitPositionImpulse2 - oldLimitPositionImpulse;

            Vec2 P2 = m_u2.mul(-impulse);
            b2.m_position.addLocal(P2.mul(b2.m_invMass));
            b2.m_rotation += b2.m_invI * Vec2.cross(r2, P2);
            b2.m_R.setAngle(b2.m_rotation);
        }

        return linearError < Settings.linearSlop;
    }

    public Vec2 getAnchor1() {
        return m_body1.m_R.mul(m_localAnchor1).addLocal(m_body1.m_position);
    }

    public Vec2 getAnchor2() {
        return m_body2.m_R.mul(m_localAnchor2).addLocal(m_body2.m_position);
    }

    public Vec2 getGroundPoint1() {
        return m_ground.m_position.add(m_groundAnchor1);
    }

    public Vec2 getGroundPoint2() {
        return m_ground.m_position.add(m_groundAnchor2);
    }

    public Vec2 getReactionForce(float invTimeStep) {
        // NOT_USED(invTimeStep);
        Vec2 F = new Vec2(0.0f, 0.0f); // = (m_pulleyImpulse * invTimeStep) *
        // m_u;
        return F;
    }

    public float getReactionTorque(float invTimeStep) {
        // NOT_USED(invTimeStep);
        return 0.0f;
    }

    public float getLength1() {
        Vec2 p = m_body1.m_R.mul(m_localAnchor1).addLocal(m_body1.m_position);
        Vec2 s = m_ground.m_position.add(m_groundAnchor1);
        Vec2 d = p.sub(s);
        return d.length();
    }

    public float getLength2() {
        Vec2 p = m_body2.m_R.mul(m_localAnchor2).addLocal(m_body2.m_position);
        Vec2 s = m_ground.m_position.add(m_groundAnchor2);
        Vec2 d = p.sub(s);
        return d.length();
    }

    public float getRatio() {
        return m_ratio;
    }

}