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
package org.jbox2d.testbed.tests;

import org.jbox2d.collision.PolygonDef;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.joints.PrismaticJoint;
import org.jbox2d.dynamics.joints.PulleyJoint;
import org.jbox2d.dynamics.joints.PulleyJointDef;
import org.jbox2d.testbed.AbstractExample;
import org.jbox2d.testbed.TestbedMain;


public class Pulleys extends AbstractExample {

    PulleyJoint m_joint1;

    PrismaticJoint m_joint2;

    public Pulleys(TestbedMain p) {
        super(p);
    }

    @Override
    public void create() {
    	Body ground = null;
		{
			PolygonDef sd = new PolygonDef();
			sd.setAsBox(50.0f, 10.0f);

			BodyDef bd = new BodyDef();
			bd.position.set(0.0f, -10.0f);
			ground = m_world.createStaticBody(bd);
			ground.createShape(sd);
		}

		{
			float a = 2.0f;
			float b = 4.0f;
			float y = 16.0f;
			float L = 12.0f;

			PolygonDef sd = new PolygonDef();
			sd.setAsBox(a, b);
			sd.density = 5.0f;

			BodyDef bd = new BodyDef();

			bd.position.set(-10.0f, y);
			Body body1 = m_world.createDynamicBody(bd);
			body1.createShape(sd);
			body1.setMassFromShapes();

			bd.position.set(10.0f, y);
			Body body2 = m_world.createDynamicBody(bd);
			body2.createShape(sd);
			body2.setMassFromShapes();

			PulleyJointDef pulleyDef = new PulleyJointDef();
			Vec2 anchor1 = new Vec2(-10.0f, y + b);
			Vec2 anchor2 = new Vec2(10.0f, y + b);
			Vec2 groundAnchor1 = new Vec2(-10.0f, y + b + L);
			Vec2 groundAnchor2 = new Vec2(10.0f, y + b + L);
			pulleyDef.initialize(body1, body2, groundAnchor1, groundAnchor2, anchor1, anchor2, 2.0f);

			m_joint1 = (PulleyJoint)m_world.createJoint(pulleyDef);
		}

    }
    
    public void postStep() {
		float ratio = m_joint1.getRatio();
		float L = m_joint1.getLength1() + ratio * m_joint1.getLength2();
		m_debugDraw.drawString(5, m_textLine, "L1 + "+ratio+" * L2 = "+L,white);
		m_textLine += textLineHeight;
	}

    public String getName() {
    	return "Pulleys";
    }
}
