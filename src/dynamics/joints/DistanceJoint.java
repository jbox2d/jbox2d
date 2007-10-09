package dynamics.joints;

import common.Settings;
import common.Vec2;

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

    public DistanceJoint(DistanceJointDescription description) {
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
    public Vec2 GetAnchor1() {
        return m_body1.m_position.add(m_body1.m_R.mul(m_localAnchor1));
    }

    @Override
    public Vec2 GetAnchor2() {
        return m_body2.m_position.add(m_body2.m_R.mul(m_localAnchor2));
    }

    @Override
    public void PreSolve() {
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

        // Warm starting.
        Vec2 p = m_u.mul(m_impulse);

        m_body1.m_linearVelocity.subLocal(p.mul(m_body1.m_invMass));
        m_body1.m_angularVelocity -= m_body1.m_invI * Vec2.cross(r1, p);
        m_body2.m_linearVelocity.addLocal(p.mul(m_body2.m_invMass));
        m_body2.m_angularVelocity += m_body2.m_invI * Vec2.cross(r2, p);
    }

    @Override
    public boolean SolvePositionConstraints() {
        Vec2 r1 = m_body1.m_R.mul(m_localAnchor1);
        Vec2 r2 = m_body2.m_R.mul(m_localAnchor2);
        // Vec2 d = m_body2.m_position.add(r2).sub(m_body1.m_position).sub(r1);
        Vec2 d = m_body2.m_position.clone().addLocal(r2).subLocal(
                m_body1.m_position).subLocal(r1);

        float c = d.length() - m_length;
        float impulse = -m_mass * c;

        Vec2 P = m_u.mul(impulse);

        m_body1.m_position.subLocal(P.mul(m_body1.m_invMass));
        m_body1.m_rotation -= m_body1.m_invI * Vec2.cross(r1, P);
        m_body2.m_position.addLocal(P.mul(m_body2.m_invMass));
        m_body2.m_rotation += m_body2.m_invI * Vec2.cross(r2, P);

        m_body1.m_R.set(m_body1.m_rotation);
        m_body2.m_R.set(m_body2.m_rotation);

        return Math.abs(c) < Settings.linearSlop;
    }

    @Override
    public void SolveVelocityConstraints(float dt) {
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