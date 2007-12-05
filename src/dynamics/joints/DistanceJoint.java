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

import common.*;
import dynamics.TimeStep;
import dynamics.World;

//Updated to rev 56 of b2DistanceJoint.cpp/.h

//C = norm(p2 - p1) - L
//u = (p2 - p1) / norm(p2 - p1)
//Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
//J = [-u -cross(r1, u) u cross(r2, u)]
//K = J * invM * JT
//= invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

public class DistanceJoint extends Joint {
    Vec2 m_localAnchor1;

    Vec2 m_localAnchor2;

    Vec2 m_u;

    float m_impulse;

    float m_mass; // effective mass for the constraint.

    float m_length;

    public DistanceJoint(DistanceJointDef description) {
        super(description);
        m_localAnchor1 = m_body1.m_R.mulT(description.anchorPoint1
                .sub(m_body1.m_position));
        m_localAnchor2 = m_body2.m_R.mulT(description.anchorPoint2
                .sub(m_body2.m_position));

        m_length = description.anchorPoint2.sub(description.anchorPoint1)
                .length();
        m_impulse = 0.0f;
    }

    @Override
    public Vec2 getAnchor1() {
        return m_body1.m_R.mul(m_localAnchor1).addLocal(m_body1.m_position);
    }

    @Override
    public Vec2 getAnchor2() {
        return m_body2.m_R.mul(m_localAnchor2).addLocal(m_body2.m_position);
    }

    public Vec2 getReactionForce(float invTimeStep) {
        Vec2 F = m_u.mul(m_impulse * invTimeStep);
        return F;
    }

    public float getReactionTorque(float invTimeStep) {
        return 0.0f;
    }

    @Override
    public void prepareVelocitySolver() {
        // Compute the effective mass matrix.
        Vec2 r1 = m_body1.m_R.mul(m_localAnchor1);
        Vec2 r2 = m_body2.m_R.mul(m_localAnchor2);

        // m_u = m_body2.m_position.add(r2).sub(m_body1.m_position).sub(r1);
        m_u = m_body2.m_position.clone().addLocal(r2).subLocal(
                m_body1.m_position).subLocal(r1);

        // Handle singularity.
        float length = m_u.length();
        if (length > Settings.linearSlop) {
            m_u.mulLocal(1.0f / length);
        }
        else {
            m_u.set(0.0f, 0.0f);
        }

        float cr1u = Vec2.cross(r1, m_u);
        float cr2u = Vec2.cross(r2, m_u);
        m_mass = m_body1.m_invMass + m_body1.m_invI * cr1u * cr1u
                + m_body2.m_invMass + m_body2.m_invI * cr2u * cr2u;

        assert m_mass > Settings.EPSILON;

        m_mass = 1.0f / m_mass;

        if (World.ENABLE_WARM_STARTING) {
            // Warm starting.
            Vec2 p = m_u.mul(m_impulse);

            m_body1.m_linearVelocity.subLocal(p.mul(m_body1.m_invMass));
            m_body1.m_angularVelocity -= m_body1.m_invI * Vec2.cross(r1, p);
            m_body2.m_linearVelocity.addLocal(p.mul(m_body2.m_invMass));
            m_body2.m_angularVelocity += m_body2.m_invI * Vec2.cross(r2, p);
        }
        else {
            m_impulse = 0.0f;
        }
    }

    @Override
    public boolean solvePositionConstraints() {
        Vec2 r1 = m_body1.m_R.mul(m_localAnchor1);
        Vec2 r2 = m_body2.m_R.mul(m_localAnchor2);
        // Vec2 d = m_body2.m_position.add(r2).sub(m_body1.m_position).sub(r1);
        Vec2 d = m_body2.m_position.clone().addLocal(r2).subLocal(
                m_body1.m_position).subLocal(r1);

        //float c = d.length() - m_length;
        //float impulse = -m_mass * c;

        //Vec2 P = m_u.mul(impulse);
        
        float length = d.normalize();
        float C = length - m_length;
        C = MathUtils.clamp(C, -Settings.maxLinearCorrection, Settings.maxLinearCorrection);

        float impulse = -m_mass * C;
        m_u = d;
        Vec2 P = m_u.mul(impulse);

        m_body1.m_position.subLocal(P.mul(m_body1.m_invMass));
        m_body1.m_rotation -= m_body1.m_invI * Vec2.cross(r1, P);
        m_body2.m_position.addLocal(P.mul(m_body2.m_invMass));
        m_body2.m_rotation += m_body2.m_invI * Vec2.cross(r2, P);

        m_body1.m_R.setAngle(m_body1.m_rotation);
        m_body2.m_R.setAngle(m_body2.m_rotation);

        return Math.abs(C) < Settings.linearSlop;
    }

    @Override
    public void solveVelocityConstraints(TimeStep step) {
        Vec2 r1 = m_body1.m_R.mul(m_localAnchor1);
        Vec2 r2 = m_body2.m_R.mul(m_localAnchor2);

        // Cdot = dot(u, v + cross(w, r))
        Vec2 v1 = m_body1.m_linearVelocity.add(Vec2.cross(
                m_body1.m_angularVelocity, r1));
        Vec2 v2 = m_body2.m_linearVelocity.add(Vec2.cross(
                m_body2.m_angularVelocity, r2));
        float Cdot = Vec2.dot(m_u, v2.sub(v1));
        float impulse = -m_mass * Cdot;
        m_impulse += impulse;

        Vec2 p = m_u.mul(impulse);

        m_body1.m_linearVelocity.subLocal(p.mul(m_body1.m_invMass));
        m_body1.m_angularVelocity -= m_body1.m_invI * Vec2.cross(r1, p);
        m_body2.m_linearVelocity.addLocal(p.mul(m_body2.m_invMass));
        m_body2.m_angularVelocity += m_body2.m_invI * Vec2.cross(r2, p);
    }
}