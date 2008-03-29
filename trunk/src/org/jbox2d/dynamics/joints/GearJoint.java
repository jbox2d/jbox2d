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
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.TimeStep;
import org.jbox2d.dynamics.World;


//Updated to rev 56->97 of b2GearJoint.cpp/.h

/**
 * A gear joint is used to connect two joints together. Either joint
 * can be a revolute or prismatic joint. You specify a gear ratio
 * to bind the motions together:
 * coordinate1 + ratio * coordinate2 = constant
 * The ratio can be negative or positive. If one joint is a revolute joint
 * and the other joint is a prismatic joint, then the ratio will have units
 * of length or units of 1/length.
 * <BR><em>Warning</em>: The revolute and prismatic joints must be attached to
 * fixed bodies (which must be body1 on those joints).
 */
public class GearJoint extends Joint {

    // Gear Joint:
    // C0 = (coordinate1 + ratio * coordinate2)_initial
    // C = C0 - (cordinate1 + ratio * coordinate2) = 0
    // Cdot = -(Cdot1 + ratio * Cdot2)
    // J = -[J1 ratio * J2]
    // K = J * invM * JT
    // = J1 * invM1 * J1T + ratio * ratio * J2 * invM2 * J2T
    //
    // Revolute:
    // coordinate = rotation
    // Cdot = angularVelocity
    // J = [0 0 1]
    // K = J * invM * JT = invI
    //
    // Prismatic:
    // coordinate = dot(p - pg, ug)
    // Cdot = dot(v + cross(w, r), ug)
    // J = [ug cross(r, ug)]
    // K = J * invM * JT = invMass + invI * cross(r, ug)^2

	public Body m_ground1;
	public Body m_ground2;

	// One of these is NULL.
	public RevoluteJoint m_revolute1;
	public PrismaticJoint m_prismatic1;

	// One of these is NULL.
	public RevoluteJoint m_revolute2;
	public PrismaticJoint m_prismatic2;

	public Vec2 m_groundAnchor1;
	public Vec2 m_groundAnchor2;

	public Vec2 m_localAnchor1;
	public Vec2 m_localAnchor2;

	public Jacobian m_J;

	public float m_constant;
	public float m_ratio;

	/** Effective mass */
	float m_mass;

	/** Force for accumulation/warm starting. */
	float m_force;

    public GearJoint(GearJointDef def) {
        super(def);

        m_J = new Jacobian();

        assert (def.joint1.m_type == JointType.REVOLUTE_JOINT || def.joint1.m_type == JointType.PRISMATIC_JOINT);
        assert (def.joint2.m_type == JointType.REVOLUTE_JOINT || def.joint2.m_type == JointType.PRISMATIC_JOINT);
        assert (def.joint1.m_body1.isStatic());
        assert (def.joint2.m_body1.isStatic());

        m_revolute1 = null;
        m_prismatic1 = null;
        m_revolute2 = null;
        m_prismatic2 = null;

        float coordinate1, coordinate2;

        m_ground1 = def.joint1.m_body1;
        m_body1 = def.joint1.m_body2;
        if (def.joint1.m_type == JointType.REVOLUTE_JOINT) {
            m_revolute1 = (RevoluteJoint) def.joint1;
            m_groundAnchor1 = m_revolute1.m_localAnchor1;
            m_localAnchor1 = m_revolute1.m_localAnchor2;
            coordinate1 = m_revolute1.getJointAngle();
        }
        else {
            m_prismatic1 = (PrismaticJoint) def.joint1;
            m_groundAnchor1 = m_prismatic1.m_localAnchor1;
            m_localAnchor1 = m_prismatic1.m_localAnchor2;
            coordinate1 = m_prismatic1.getJointTranslation();
        }

        m_ground2 = def.joint2.m_body1;
        m_body2 = def.joint2.m_body2;
        if (def.joint2.m_type == JointType.REVOLUTE_JOINT) {
            m_revolute2 = (RevoluteJoint) def.joint2;
            m_groundAnchor2 = m_revolute2.m_localAnchor1;
            m_localAnchor2 = m_revolute2.m_localAnchor2;
            coordinate2 = m_revolute2.getJointAngle();
        }
        else {
            m_prismatic2 = (PrismaticJoint) def.joint2;
            m_groundAnchor2 = m_prismatic2.m_localAnchor1;
            m_localAnchor2 = m_prismatic2.m_localAnchor2;
            coordinate2 = m_prismatic2.getJointTranslation();
        }

        m_ratio = def.ratio;

        m_constant = coordinate1 + m_ratio * coordinate2;

        m_force = 0.0f;
    }

    public void initVelocityConstraints(TimeStep step) {
        Body g1 = m_ground1;
        Body g2 = m_ground2;
        Body b1 = m_body1;
        Body b2 = m_body2;

        float K = 0.0f;
        m_J.setZero();

        if (m_revolute1 != null) {
            m_J.angular1 = -1.0f;
            K += b1.m_invI;
        }
        else {
        	Vec2 ug = Mat22.mul(g1.m_xf.R, m_prismatic1.m_localXAxis1);
    		Vec2 r = Mat22.mul(b1.m_xf.R, m_localAnchor1.sub(b1.getLocalCenter()));
    		float crug = Vec2.cross(r, ug);
            m_J.linear1 = ug.negate();
            m_J.angular1 = -crug;
            K += b1.m_invMass + b1.m_invI * crug * crug;
        }

        if (m_revolute2 != null) {
            m_J.angular2 = -m_ratio;
            K += m_ratio * m_ratio * b2.m_invI;
        }
        else {
            Vec2 ug = Mat22.mul(g2.m_xf.R, m_prismatic2.m_localXAxis1);
    		Vec2 r = Mat22.mul(b2.m_xf.R, m_localAnchor2.sub(b2.getLocalCenter()));
            float crug = Vec2.cross(r, ug);
            m_J.linear2 = ug.mulLocal(-m_ratio);
            m_J.angular2 = -m_ratio * crug;
            K += m_ratio * m_ratio * (b2.m_invMass + b2.m_invI * crug * crug);
        }

        // Compute effective mass.
        assert (K > 0.0f);
        m_mass = 1.0f / K;

        if (World.ENABLE_WARM_STARTING) {
    		// Warm starting.
    		float P = step.dt * m_force;
    		b1.m_linearVelocity.x += b1.m_invMass * P * m_J.linear1.x;
    		b1.m_linearVelocity.y += b1.m_invMass * P * m_J.linear1.y;
    		b1.m_angularVelocity += b1.m_invI * P * m_J.angular1;
    		b2.m_linearVelocity.x += b2.m_invMass * P * m_J.linear2.x;
    		b2.m_linearVelocity.y += b2.m_invMass * P * m_J.linear2.y;
    		b2.m_angularVelocity += b2.m_invI * P * m_J.angular2;
    	} else {
    		m_force = 0.0f;
    	}
    }

    public void solveVelocityConstraints(TimeStep step) {
    	Body b1 = m_body1;
    	Body b2 = m_body2;

    	float Cdot = m_J.compute(	b1.m_linearVelocity, b1.m_angularVelocity,
    								b2.m_linearVelocity, b2.m_angularVelocity);

    	float force = -step.inv_dt * m_mass * Cdot;
    	m_force += force;

    	float P = step.dt * force;
    	b1.m_linearVelocity.x += b1.m_invMass * P * m_J.linear1.x;
    	b1.m_linearVelocity.y += b1.m_invMass * P * m_J.linear1.y;
    	b1.m_angularVelocity += b1.m_invI * P * m_J.angular1;
    	b2.m_linearVelocity.x += b2.m_invMass * P * m_J.linear2.x;
    	b2.m_linearVelocity.y += b2.m_invMass * P * m_J.linear2.y;
    	b2.m_angularVelocity += b2.m_invI * P * m_J.angular2;
    }

    public boolean solvePositionConstraints() {
        float linearError = 0.0f;

        Body b1 = m_body1;
        Body b2 = m_body2;

        float coordinate1, coordinate2;
        if (m_revolute1 != null) {
            coordinate1 = m_revolute1.getJointAngle();
        }
        else {
            coordinate1 = m_prismatic1.getJointTranslation();
        }

        if (m_revolute2 != null) {
            coordinate2 = m_revolute2.getJointAngle();
        }
        else {
            coordinate2 = m_prismatic2.getJointTranslation();
        }

        float C = m_constant - (coordinate1 + m_ratio * coordinate2);

        float impulse = -m_mass * C;

        b1.m_sweep.c.x += b1.m_invMass * impulse * m_J.linear1.x;
        b1.m_sweep.c.y += b1.m_invMass * impulse * m_J.linear1.y;
        b1.m_sweep.a += b1.m_invI * impulse * m_J.angular1;
    	b2.m_sweep.c.x += b2.m_invMass * impulse * m_J.linear2.x;
    	b2.m_sweep.c.y += b2.m_invMass * impulse * m_J.linear2.y;
    	b2.m_sweep.a += b2.m_invI * impulse * m_J.angular2;

    	b1.synchronizeTransform();
    	b2.synchronizeTransform();

    	return linearError < Settings.linearSlop;
    }

    public Vec2 getAnchor1() {
        return m_body1.getWorldPoint(m_localAnchor1);
    }

    public Vec2 getAnchor2() {
        return m_body2.getWorldPoint(m_localAnchor2);
    }

    public Vec2 getReactionForce() {
    	// TODO_ERIN not tested
        return new Vec2(m_force * m_J.linear2.x, m_force * m_J.linear2.y);
    }

    public float getReactionTorque() {
    	// TODO_ERIN not tested
    	Vec2 r = Mat22.mul(m_body2.m_xf.R, m_localAnchor2.sub(m_body2.getLocalCenter()));
    	Vec2 F = new Vec2(m_force * m_J.linear2.x, m_force * m_J.linear2.y);
    	float T = m_force * m_J.angular2 - Vec2.cross(r, F);
    	return T;
    }

    public float getRatio() {
        return m_ratio;
    }
}