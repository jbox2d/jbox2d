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

import common.Mat22;
import common.Settings;
import common.Vec2;

import dynamics.Body;
import dynamics.TimeStep;

//Updated to rev 56 of b2MouseJoint.cpp/.h

//p = attached point, m = mouse point
//C = p - m
//Cdot = v
//   = v + cross(w, r)
//J = [I r_skew]
//Identity used:
//w k % (rx i + ry j) = w * (-ry i + rx j)

public class MouseJoint extends Joint {

    public Vec2 m_localAnchor;

    public Vec2 m_target;

    Vec2 m_impulse;

    Mat22 m_ptpMass; // effective mass for point-to-point constraint.

    Vec2 m_C; // position error

    float m_maxForce;

    float m_beta; // bias factor

    float m_gamma; // softness

    public MouseJoint(MouseJointDef def) {
        super(def);
        m_target = def.target;
        m_localAnchor = m_body2.m_R.mulT(m_target.sub(m_body2.m_position));

        m_maxForce = def.maxForce;
        m_impulse = new Vec2();

        float mass = m_body2.m_mass;

        // Frequency
        float omega = 2.0f * Settings.pi * def.frequencyHz;

        // Damping coefficient
        float d = 2.0f * mass * def.dampingRatio * omega;

        // Spring stiffness
        float k = mass * omega * omega;

        // magic formulas
        m_gamma = 1.0f / (d + def.timeStep * k);
        m_beta = def.timeStep * k / (d + def.timeStep * k);

    }

    public void setTarget(Vec2 target) {
        m_body2.wakeUp();
        m_target = target;
    }

    @Override
    public Vec2 getAnchor1() {
        return m_target;
    }

    @Override
    public Vec2 getAnchor2() {
        return m_body2.m_R.mul(m_localAnchor).addLocal(m_body2.m_position);
    }

    @Override
    public void prepareVelocitySolver() {
        Body b = m_body2;

        // Compute the effective mass matrix.
        Vec2 r = b.m_R.mul(m_localAnchor);

        // K = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2)
        // * invI2 * skew(r2)]
        // = [1/m1+1/m2 0 ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 *
        // [r1.y*r1.y -r1.x*r1.y]
        // [ 0 1/m1+1/m2] [-r1.x*r1.y r1.x*r1.x] [-r1.x*r1.y r1.x*r1.x]
        float invMass = b.m_invMass;
        float invI = b.m_invI;

        Mat22 K1 = new Mat22(invMass, 0.0f, 0.0f, invMass);

        Mat22 K2 = new Mat22(invI * r.y * r.y, -invI * r.x * r.y, -invI * r.x
                * r.y, invI * r.x * r.x);

        Mat22 K = K1.add(K2);
        K.col1.x += m_gamma;
        K.col2.y += m_gamma;

        m_ptpMass = K.invert();

        m_C = b.m_position.clone().addLocal(r).subLocal(m_target);

        // Cheat with some damping
        b.m_angularVelocity *= 0.98f;

        // Warm starting.
        Vec2 P = m_impulse.clone();
        b.m_linearVelocity.addLocal(P.mul(invMass));
        b.m_angularVelocity += invI * Vec2.cross(r, P);
    }

    @Override
    public boolean solvePositionConstraints() {
        return true;
    }

    @Override
    public void solveVelocityConstraints(TimeStep step) {
        Body body = m_body2;

        Vec2 r = body.m_R.mul(m_localAnchor);

        // Cdot = v + cross(w, r)
        Vec2 Cdot = body.m_linearVelocity.add(Vec2.cross(
                body.m_angularVelocity, r));

        // Vec2 impulse = m_ptpMass.mul(
        // Cdot.add(m_C.mul(m_beta * step.inv_dt).add(
        // m_impulse.mul(m_gamma)))).negate();

        Vec2 impulse = m_ptpMass.mul(
                m_C.clone().mulLocal(m_beta * step.inv_dt).addLocal(
                        m_impulse.mul(m_gamma)).addLocal(Cdot)).negateLocal();

        Vec2 oldImpulse = m_impulse.clone();
        m_impulse.addLocal(impulse);
        float length = m_impulse.length();
        if (length > step.dt * m_maxForce) {
            m_impulse.mulLocal(step.dt * m_maxForce / length);
        }
        impulse = m_impulse.sub(oldImpulse);

        body.m_linearVelocity.addLocal(impulse.mul(body.m_invMass));
        body.m_angularVelocity += body.m_invI * Vec2.cross(r, impulse);
    }

    Vec2 getReactionForce(float invTimeStep) {
        return m_impulse.mul(invTimeStep);
    }
}
