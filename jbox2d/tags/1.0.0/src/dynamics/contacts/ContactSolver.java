package dynamics.contacts;

import java.util.ArrayList;
import java.util.List;

import common.MathUtils;
import common.Settings;
import common.Vec2;

import collision.ContactPoint;
import collision.Manifold;

import dynamics.Body;

public class ContactSolver {
    List<ContactConstraint> m_constraints;

    int m_constraintCount;

    public ContactSolver(Contact[] contacts, int contactCount, float inv_dt) {
        m_constraintCount = 0;

        for (int i = 0; i < contactCount; i++) {// Contact c : contacts) {
            m_constraintCount += contacts[i].GetManifoldCount();
        }

        m_constraints = new ArrayList<ContactConstraint>(m_constraintCount);
        for (int i = 0; i < m_constraintCount; i++) {
            m_constraints.add(new ContactConstraint());
        }

        int count = 0;
        for (int i = 0; i < contactCount; i++) {// Contact contact : contacts) {
            Contact contact = contacts[i];
            Body b1 = contact.m_shape1.m_body;
            Body b2 = contact.m_shape2.m_body;
            int manifoldCount = contact.GetManifoldCount();
            List<Manifold> manifolds = contact.GetManifolds();
            float friction = contact.m_friction;
            float restitution = contact.m_restitution;

            Vec2 v1 = b1.m_linearVelocity.clone();
            Vec2 v2 = b2.m_linearVelocity.clone();
            float w1 = b1.m_angularVelocity;
            float w2 = b2.m_angularVelocity;

            for (int j = 0; j < manifoldCount; ++j) {// Manifold manifold :
                // manifolds) {
                Manifold manifold = manifolds.get(j);

                assert (manifold.pointCount > 0):"Manifold "+j+" has length 0";
               
                Vec2 normal = manifold.normal.clone();

                assert (count < m_constraintCount);

                ContactConstraint c = m_constraints.get(count);
                c.body1 = b1;
                c.body2 = b2;
                c.manifold = manifold;
                c.normal = normal.clone();
                c.pointCount = manifold.pointCount;
                c.friction = friction;
                c.restitution = restitution;

                for (int k = 0; k < c.pointCount; ++k) {
                    ContactPoint cp = manifold.points[k];
                    ContactConstraintPoint ccp = c.points[k];

                    ccp.normalImpulse = cp.normalImpulse;
                    ccp.tangentImpulse = cp.tangentImpulse;
                    ccp.separation = cp.separation;

                    Vec2 r1 = cp.position.sub(b1.m_position);
                    Vec2 r2 = cp.position.sub(b2.m_position);

                    ccp.localAnchor1 = b1.m_R.mulT(r1);
                    ccp.localAnchor2 = b2.m_R.mulT(r2);

                    float r1Sqr = Vec2.dot(r1, r1);
                    float r2Sqr = Vec2.dot(r2, r2);

                    float rn1 = Vec2.dot(r1, normal);
                    float rn2 = Vec2.dot(r2, normal);

                    float kNormal = b1.m_invMass + b2.m_invMass;
                    kNormal += b1.m_invI * (r1Sqr - rn1 * rn1) + b2.m_invI
                            * (r2Sqr - rn2 * rn2);

                    assert (kNormal > Settings.EPSILON);

                    ccp.normalMass = 1.0f / kNormal;

                    Vec2 tangent = Vec2.cross(normal, 1.0f);

                    float rt1 = Vec2.dot(r1, tangent);
                    float rt2 = Vec2.dot(r2, tangent);
                    float kTangent = b1.m_invMass + b2.m_invMass;
                    kTangent += b1.m_invI * (r1Sqr - rt1 * rt1) + b2.m_invI
                            * (r2Sqr - rt2 * rt2);
                    assert (kTangent > Settings.EPSILON);
                    ccp.tangentMass = 1.0f / kTangent;

                    // Setup a velocity bias for restitution.
                    // float vRel = Vec2.dot(c.normal, v2.add(Vec2.cross(w2,
                    // r2)).sub(v1).sub(Vec2.cross(w1, r1)));
                    float vRel = Vec2.dot(c.normal, v2.clone().addLocal(
                            Vec2.cross(w2, r2)).subLocal(v1).subLocal(
                            Vec2.cross(w1, r1)));
                    if (vRel < -Settings.velocityThreshold) {
                        ccp.velocityBias = -c.restitution * vRel;
                    }
                    else {
                        ccp.velocityBias = 0.0f;
                    }
                }

                ++count;
            }
        }

        assert (count == m_constraintCount);
    }

    public void PreSolve() {
        // Warm start.
        for (int i = 0; i < m_constraintCount; ++i) {// ContactConstraint c :
            // m_constraints) {
            ContactConstraint c = m_constraints.get(i);
            Body b1 = c.body1;
            Body b2 = c.body2;
            float invMass1 = b1.m_invMass;
            float invI1 = b1.m_invI;
            float invMass2 = b2.m_invMass;
            float invI2 = b2.m_invI;
            Vec2 normal = c.normal;
            Vec2 tangent = Vec2.cross(normal, 1.0f);

            for (int j = 0; j < c.pointCount; ++j) {// ContactConstraintPoint
                // ccp : c.points) {
                ContactConstraintPoint ccp = c.points[j];
                // Vec2 P = normal.mul(ccp.normalImpulse).add(
                // tangent.mul(ccp.tangentImpulse));
                // Vec2 r1 = b1.m_R.mul(ccp.localAnchor1);
                // Vec2 r2 = b2.m_R.mul(ccp.localAnchor2);
                float px = normal.x * ccp.normalImpulse + tangent.x
                        * ccp.tangentImpulse;
                float py = normal.y * ccp.normalImpulse + tangent.y
                        * ccp.tangentImpulse;
                float r1x = b1.m_R.col1.x * ccp.localAnchor1.x + b1.m_R.col2.x
                        * ccp.localAnchor1.y;
                float r1y = b1.m_R.col1.y * ccp.localAnchor1.x + b1.m_R.col2.y
                        * ccp.localAnchor1.y;
                float r2x = b2.m_R.col1.x * ccp.localAnchor2.x + b2.m_R.col2.x
                        * ccp.localAnchor2.y;
                float r2y = b2.m_R.col1.y * ccp.localAnchor2.x + b2.m_R.col2.y
                        * ccp.localAnchor2.y;

                // b1.m_angularVelocity -= invI1 * Vec2.cross(r1, P);
                b1.m_angularVelocity -= invI1 * (r1x * py - r1y * px);
                // b1.m_linearVelocity.subLocal(P.mul(invMass1));
                b1.m_linearVelocity.x -= px * invMass1;
                b1.m_linearVelocity.y -= py * invMass1;
                // b2.m_angularVelocity += invI2 * Vec2.cross(r2, P);
                b2.m_angularVelocity += invI2 * (r2x * py - r2y * px);
                // b2.m_linearVelocity.addLocal(P.mul(invMass2));
                b2.m_linearVelocity.x += px * invMass2;
                b2.m_linearVelocity.y += py * invMass2;

                ccp.positionImpulse = 0.0f;
            }
        }
    }

    public void SolveVelocityConstraints() {
        for (ContactConstraint c : m_constraints) {
            Body b1 = c.body1;
            Body b2 = c.body2;
            float invMass1 = b1.m_invMass;
            float invI1 = b1.m_invI;
            float invMass2 = b2.m_invMass;
            float invI2 = b2.m_invI;
            Vec2 normal = c.normal;
            Vec2 tangent = Vec2.cross(normal, 1.0f);

            // Solver normal constraints
            for (ContactConstraintPoint ccp : c.points) {

                // Vec2 r1 = b1.m_R.mul(ccp.localAnchor1);
                // Vec2 r2 = b2.m_R.mul(ccp.localAnchor2);
                // new Vec2(col1.x * v.x + col2.x * v.y,
                // col1.y * v.x + col2.y * v.y)
                float r1x = b1.m_R.col1.x * ccp.localAnchor1.x + b1.m_R.col2.x
                        * ccp.localAnchor1.y;
                float r1y = b1.m_R.col1.y * ccp.localAnchor1.x + b1.m_R.col2.y
                        * ccp.localAnchor1.y;
                float r2x = b2.m_R.col1.x * ccp.localAnchor2.x + b2.m_R.col2.x
                        * ccp.localAnchor2.y;
                float r2y = b2.m_R.col1.y * ccp.localAnchor2.x + b2.m_R.col2.y
                        * ccp.localAnchor2.y;

                // Relative velocity at contact
                // Vec2 dv = b2.m_linearVelocity.add(
                // Vec2.cross(b2.m_angularVelocity, r2)).sub(
                // b1.m_linearVelocity).sub(
                // Vec2.cross(b1.m_angularVelocity, r1));
                float dvx = b2.m_linearVelocity.x
                        + (-b2.m_angularVelocity * r2y) - b1.m_linearVelocity.x
                        - (-b1.m_angularVelocity * r1y);
                float dvy = b2.m_linearVelocity.y
                        + (b2.m_angularVelocity * r2x) - b1.m_linearVelocity.y
                        - (b1.m_angularVelocity * r1x);

                // Compute normal impulse
                // float vn = Vec2.dot(dv, normal);
                float vn = dvx * normal.x + dvy * normal.y;
                float lambda = -ccp.normalMass * (vn - ccp.velocityBias);

                // b2Clamp the accumulated impulse
                float newImpulse = Math.max(ccp.normalImpulse + lambda, 0.0f);
                lambda = newImpulse - ccp.normalImpulse;

                // Apply contact impulse
                // Vec2 P = normal.mul(lambda);
                float px = normal.x * lambda;
                float py = normal.y * lambda;

                // b1.m_linearVelocity.subLocal(P.mul(invMass1));
                b1.m_linearVelocity.x -= px * invMass1;
                b1.m_linearVelocity.y -= py * invMass1;
                // b1.m_angularVelocity -= invI1 * Vec2.cross(r1, P);
                b1.m_angularVelocity -= invI1 * (r1x * py - r1y * px);

                // b2.m_linearVelocity.addLocal(P.mul(invMass2));
                b2.m_linearVelocity.x += px * invMass2;
                b2.m_linearVelocity.y += py * invMass2;
                // b2.m_angularVelocity += invI2 * Vec2.cross(r2, P);
                b2.m_angularVelocity += invI2 * (r2x * py - r2y * px);

                ccp.normalImpulse = newImpulse;
            }

            // Solver tangent constraints
            for (ContactConstraintPoint ccp : c.points) {
                // 12 temp Vec2 creations per run
                // Vec2 r1 = b1.m_R.mul(ccp.localAnchor1);
                // Vec2 r2 = b2.m_R.mul(ccp.localAnchor2);
                float r1x = b1.m_R.col1.x * ccp.localAnchor1.x + b1.m_R.col2.x
                        * ccp.localAnchor1.y;
                float r1y = b1.m_R.col1.y * ccp.localAnchor1.x + b1.m_R.col2.y
                        * ccp.localAnchor1.y;
                float r2x = b2.m_R.col1.x * ccp.localAnchor2.x + b2.m_R.col2.x
                        * ccp.localAnchor2.y;
                float r2y = b2.m_R.col1.y * ccp.localAnchor2.x + b2.m_R.col2.y
                        * ccp.localAnchor2.y;

                // Relative velocity at contact
                // Vec2 dv = b2.m_linearVelocity.add(
                // Vec2.cross(b2.m_angularVelocity, r2)).sub(
                // b1.m_linearVelocity).sub(
                // Vec2.cross(b1.m_angularVelocity, r1));
                // cross -> new Vec2(-s * a.y, s * a.x);
                float dvx = b2.m_linearVelocity.x
                        + (-b2.m_angularVelocity * r2y) - b1.m_linearVelocity.x
                        - (-b1.m_angularVelocity * r1y);
                float dvy = b2.m_linearVelocity.y
                        + (b2.m_angularVelocity * r2x) - b1.m_linearVelocity.y
                        - (b1.m_angularVelocity * r1x);

                // Compute tangent impulse
                // float vt = Vec2.dot(dv, tangent);
                float vt = dvx * tangent.x + dvy * tangent.y;
                float lambda = ccp.tangentMass * (-vt);

                // b2Clamp the accumulated impulse
                float maxFriction = c.friction * ccp.normalImpulse;
                float newImpulse = MathUtils.clamp(ccp.tangentImpulse + lambda,
                        -maxFriction, maxFriction);
                lambda = newImpulse - ccp.tangentImpulse;

                // Apply contact impulse
                // Vec2 P = tangent.mul(lambda);
                float px = tangent.x * lambda;
                float py = tangent.y * lambda;

                // b1.m_linearVelocity.subLocal(P.mul(invMass1));
                b1.m_linearVelocity.x -= px * invMass1;
                b1.m_linearVelocity.y -= py * invMass1;
                // b1.m_angularVelocity -= invI1 * Vec2.cross(r1, P);
                b1.m_angularVelocity -= invI1 * (r1x * py - r1y * px);

                // b2.m_linearVelocity.addLocal(P.mul(invMass2));
                b2.m_linearVelocity.x += px * invMass2;
                b2.m_linearVelocity.y += py * invMass2;
                // b2.m_angularVelocity += invI2 * Vec2.cross(r2, P);
                b2.m_angularVelocity += invI2 * (r2x * py - r2y * px);

                ccp.tangentImpulse = newImpulse;
            }
        }
    }

    public boolean SolvePositionConstraints(float beta) {
        float minSeparation = 0.0f;

        for (ContactConstraint c : m_constraints) {
            Body b1 = c.body1;
            Body b2 = c.body2;
            float invMass1 = b1.m_invMass;
            float invI1 = b1.m_invI;
            float invMass2 = b2.m_invMass;
            float invI2 = b2.m_invI;
            Vec2 normal = c.normal;
            // Vec2 tangent = Vec2.cross(normal, 1.0f);

            // Solver normal constraints
            for (ContactConstraintPoint ccp : c.points) {
                // 10 temp Vec2 creations per run
                // Vec2 r1 = b1.m_R.mul(ccp.localAnchor1);
                // Vec2 r2 = b2.m_R.mul(ccp.localAnchor2);

                // m_R.mul -> new Vec2(col1.x * v.x + col2.x * v.y, col1.y * v.x
                // + col2.y
                // * v.y)
                float r1x = b1.m_R.col1.x * ccp.localAnchor1.x + b1.m_R.col2.x
                        * ccp.localAnchor1.y;
                float r1y = b1.m_R.col1.y * ccp.localAnchor1.x + b1.m_R.col2.y
                        * ccp.localAnchor1.y;
                float r2x = b2.m_R.col1.x * ccp.localAnchor2.x + b2.m_R.col2.x
                        * ccp.localAnchor2.y;
                float r2y = b2.m_R.col1.y * ccp.localAnchor2.x + b2.m_R.col2.y
                        * ccp.localAnchor2.y;

                // Vec2 p1 = b1.m_position.add(r1);
                // Vec2 p2 = b2.m_position.add(r2);
                // Vec2 dp = p2.sub(p1);
                float p1x = b1.m_position.x + r1x;
                float p1y = b1.m_position.y + r1y;
                float p2x = b2.m_position.x + r2x;
                float p2y = b2.m_position.y + r2y;
                float dpx = p2x - p1x;
                float dpy = p2y - p1y;

                // Approximate the current separation.
                // float separation = Vec2.dot(dp, normal) + ccp.separation;
                float separation = dpx * normal.x + dpy * normal.y
                        + ccp.separation;

                // Prevent large corrections
                separation = Math
                        .max(separation, -Settings.maxLinearCorrection);

                // Track max constraint error.
                minSeparation = Math.min(minSeparation, separation);

                // Compute normal impulse
                float dImpulse = -ccp.normalMass * beta
                        * Math.min(0.0f, separation + Settings.linearSlop);

                // b2Clamp the accumulated impulse
                float impulse0 = ccp.positionImpulse;
                ccp.positionImpulse = Math.max(impulse0 + dImpulse, 0.0f);
                dImpulse = ccp.positionImpulse - impulse0;

                // Vec2 impulse = normal.mul(dImpulse);
                float impulsex = normal.x * dImpulse;
                float impulsey = normal.y * dImpulse;

                // b1.m_position.subLocal(impulse.mul(invMass1));
                b1.m_position.x -= impulsex * invMass1;
                b1.m_position.y -= impulsey * invMass1;
                // b1.m_rotation -= invI1 * Vec2.cross(r1, impulse);
                b1.m_rotation -= invI1 * (r1x * impulsey - r1y * impulsex);
                b1.m_R.set(b1.m_rotation);

                // b2.m_position.addLocal(impulse.mul(invMass2));
                b2.m_position.x += impulsex * invMass2;
                b2.m_position.y += impulsey * invMass2;
                // b2.m_rotation += invI2 * Vec2.cross(r2, impulse);
                b2.m_rotation += invI2 * (r2x * impulsey - r2y * impulsex);
                b2.m_R.set(b2.m_rotation);
            }
        }

        return minSeparation >= -Settings.linearSlop;
    }

    public void PostSolve() {
        for (ContactConstraint c : m_constraints) {
            Manifold m = c.manifold;

            for (int j = 0; j < c.pointCount; ++j) {
                m.points[j].normalImpulse = c.points[j].normalImpulse;
                m.points[j].tangentImpulse = c.points[j].tangentImpulse;
            }
        }
    }
}
