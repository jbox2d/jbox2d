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
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.TimeStep;


//Updated to rev 56->130 of b2MouseJoint.cpp/.h

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

    public Vec2 m_force;

    public Mat22 m_mass; // effective mass for point-to-point constraint.

    public Vec2 m_C; // position error

    public float m_maxForce;

    public float m_beta; // bias factor

    public float m_gamma; // softness

    public MouseJoint(MouseJointDef def) {
        super(def);

        m_force = new Vec2();
        m_target = new Vec2();
        m_C = new Vec2();
        m_mass = new Mat22();
        m_target = def.target;
        m_localAnchor = XForm.mulT(m_body2.m_xf, m_target);

        m_maxForce = def.maxForce;

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

    /** Use this to update the target point. */
    public void setTarget(Vec2 target) {
        if (m_body2.isSleeping()) m_body2.wakeUp();
        m_target = target;
    }

    @Override
    public Vec2 getAnchor1() {
        return m_target;
    }

    @Override
    public Vec2 getAnchor2() {
    	return m_body2.getWorldPoint(m_localAnchor);
    }

    @Override
    public void initVelocityConstraints(TimeStep step) {
        Body b = m_body2;

     // Compute the effective mass matrix.
    	Vec2 r = Mat22.mul(b.m_xf.R, m_localAnchor.sub(b.getLocalCenter()));

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

        m_mass.set(K);
        m_mass = m_mass.invert();

        m_C.set(b.m_sweep.c.x + r.x - m_target.x, b.m_sweep.c.y + r.y - m_target.y);

        // Cheat with some damping
        b.m_angularVelocity *= 0.98f;

        // Warm starting.
    	float Px = step.dt * m_force.x;
    	float Py = step.dt * m_force.y;
    	b.m_linearVelocity.x += invMass * Px;
    	b.m_linearVelocity.y += invMass * Py;
    	b.m_angularVelocity += invI * (r.x*Py-r.y*Px);
    }

    @Override
    public boolean solvePositionConstraints() {
        return true;
    }

    @Override
    public void solveVelocityConstraints(TimeStep step) {
    	Body b = m_body2;

    	Vec2 r = Mat22.mul(b.m_xf.R, m_localAnchor.sub(b.getLocalCenter()));

    	// Cdot = v + cross(w, r)
    	Vec2 Cdot = b.m_linearVelocity.add(Vec2.cross(b.m_angularVelocity, r));

    	//Vec2 force = -step.inv_dt * Mat22.mul(m_mass, Cdot + (m_beta * step.inv_dt) * m_C + m_gamma * step.dt * m_force);
    	Vec2 force = new Vec2( Cdot.x + (m_beta*step.inv_dt)*m_C.x + m_gamma * step.dt * m_force.x, 
    						   Cdot.y + (m_beta*step.inv_dt)*m_C.y + m_gamma * step.dt * m_force.y );
    	force = Mat22.mul(m_mass,force);
    	force.mulLocal(-step.inv_dt);

    	Vec2 oldForce = m_force.clone();
    	m_force.addLocal(force);
    	float forceMagnitude = m_force.length();
    	if (forceMagnitude > m_maxForce) {
    		m_force.mulLocal(m_maxForce / forceMagnitude);
    	}
    	force.set(m_force.x - oldForce.x, m_force.y - oldForce.y);

    	Vec2 P = new Vec2(step.dt * force.x, step.dt * force.y);
    	b.m_linearVelocity.addLocal(P.mul(b.m_invMass));
    	b.m_angularVelocity += b.m_invI * Vec2.cross(r, P);
    }

    @Override
    public Vec2 getReactionForce() {
        return m_force;
    }

	@Override
	public float getReactionTorque() {
		return 0.0f;
	}
}
