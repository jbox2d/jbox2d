package dynamics.joints;

import common.MathUtils;
import common.Settings;
import common.Vec2;
import dynamics.Body;

//p = attached point, m = mouse point
//C = norm(p - m) - L
//u = (p - m) / norm(p - m)
//Cdot = dot(u, v + cross(w, r))
//   = [u^T cross(r, u)^T] [v; w]
//K = J * invM * JT
//= [u^T cross(r, u)^T][invMass  0][u]
//                     [0     invI][cross(r, u)]
//= [u^T cross(r, u)^T][invMass*u]
//                     [invI * cross(r, u)]
//= invMass + invI * cross(r, u) * cross(r, u)

public class MouseJoint extends Joint {

    public Vec2 m_localAnchor;

    public Vec2 m_target;

    Vec2 m_u;

    float m_positionError;

    float m_impulse;

    float m_mEff; // effective mass

    float m_motorForce;

    float m_length;

    float m_beta;

    public MouseJoint(MouseDescription description) {
        super(description);
        m_target = description.target;
        m_localAnchor = m_body2.m_R.mulT(m_target.sub(m_body2.m_position));

        m_motorForce = description.motorForce;
        m_length = description.length;
        m_beta = description.beta;

        m_impulse = 0.0f;
    }

    public void SetTarget(Vec2 target) {
        m_body2.wakeUp();
        m_target = target;
    }

    @Override
    public Vec2 GetAnchor1() {
        return m_target;
    }

    @Override
    public Vec2 GetAnchor2() {
        return m_body2.m_position.add(m_body2.m_R.mul(m_localAnchor));
    }

    @Override
    public void PreSolve() {
        Body body = m_body2;

        // Compute the effective mass matrix.
        Vec2 r = body.m_R.mul(m_localAnchor);
        // m_u = body.m_position.add(r).sub(m_target);
        m_u = body.m_position.clone().addLocal(r).subLocal(m_target);

        // Handle singularity.
        float length = m_u.length();
        if (length > Settings.EPSILON) {
            m_u.mulLocal(1.0f / length);
        }
        else {
            m_u.set(0.0f, 1.0f);
        }

        m_positionError = length - m_length;

        float cru = Vec2.cross(r, m_u);
        m_mEff = body.m_invMass + body.m_invI * cru * cru;

        assert m_mEff > Settings.EPSILON;

        m_mEff = 1.0f / m_mEff;

        // Warm starting.
        Vec2 P = m_u.mul(m_impulse);
        body.m_linearVelocity.addLocal(P.mul(body.m_invMass));
        body.m_angularVelocity += body.m_invI * Vec2.cross(r, P);
    }

    @Override
    public boolean SolvePositionConstraints() {
        return true;
    }

    @Override
    public void SolveVelocityConstraints(float dt) {
        Body body = m_body2;

        Vec2 r = body.m_R.mul(m_localAnchor);

        // Cdot = dot(u, v + cross(w, r))
        float Cdot = Vec2.dot(m_u, body.m_linearVelocity.add(Vec2.cross(
                body.m_angularVelocity, r)));
        float impulse = -m_mEff * (Cdot + m_beta / dt * m_positionError);

        float oldImpulse = m_impulse;
        m_impulse = MathUtils.clamp(m_impulse + impulse, -dt * m_motorForce,
                0.0f);
        impulse = m_impulse - oldImpulse;

        Vec2 p = m_u.mul(impulse);
        body.m_linearVelocity.addLocal(p.mul(body.m_invMass));
        body.m_angularVelocity += body.m_invI * Vec2.cross(r, p);
    }
}
