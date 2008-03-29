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

import org.jbox2d.collision.CircleDef;
import org.jbox2d.collision.PolygonDef;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.joints.GearJoint;
import org.jbox2d.dynamics.joints.GearJointDef;
import org.jbox2d.dynamics.joints.PrismaticJoint;
import org.jbox2d.dynamics.joints.PrismaticJointDef;
import org.jbox2d.dynamics.joints.RevoluteJoint;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import org.jbox2d.testbed.AbstractExample;
import org.jbox2d.testbed.TestbedMain;

import processing.core.PApplet;

public class Gears extends AbstractExample {

    RevoluteJoint m_joint1;

    RevoluteJoint m_joint2;

    PrismaticJoint m_joint3;

    GearJoint m_joint4;

    GearJoint m_joint5;

    public Gears(TestbedMain _parent) {
        super(_parent);
    }
    
    public String getName() {
    	return "Gears";
    }

    @Override
    public void create() {
    	Body ground = null;
		{
			BodyDef bd = new BodyDef();
			bd.position.set(0.0f, -10.0f);
			ground = m_world.createStaticBody(bd);

			PolygonDef sd = new PolygonDef();
			sd.setAsBox(50.0f, 10.0f);
			ground.createShape(sd);
		}

		{
			CircleDef circle1 = new CircleDef();
			circle1.radius = 1.0f;
			circle1.density = 5.0f;
			
			CircleDef circle2 = new CircleDef();
			circle2.radius = 2.0f;
			circle2.density = 5.0f;

			PolygonDef box = new PolygonDef();
			box.setAsBox(0.5f, 5.0f);
			box.density = 5.0f;

			BodyDef bd1 = new BodyDef();
			bd1.position.set(-3.0f, 12.0f);
			Body body1 = m_world.createDynamicBody(bd1);
			body1.createShape(circle1);
			body1.setMassFromShapes();

			RevoluteJointDef jd1 = new RevoluteJointDef();
			jd1.body1 = ground;
			jd1.body2 = body1;
			jd1.localAnchor1 = ground.getLocalPoint(bd1.position);
			jd1.localAnchor2 = body1.getLocalPoint(bd1.position);
			jd1.referenceAngle = body1.getAngle() - ground.getAngle();
			m_joint1 = (RevoluteJoint)(m_world.createJoint(jd1));

			BodyDef bd2 = new BodyDef();
			bd2.position.set(0.0f, 12.0f);
			Body body2 = m_world.createDynamicBody(bd2);
			body2.createShape(circle2);
			body2.setMassFromShapes();

			RevoluteJointDef jd2 = new RevoluteJointDef();
			jd2.initialize(ground, body2, bd2.position);
			m_joint2 = (RevoluteJoint)(m_world.createJoint(jd2));

			BodyDef bd3 = new BodyDef();
			bd3.position.set(2.5f, 12.0f);
			Body body3 = m_world.createDynamicBody(bd3);
			body3.createShape(box);
			body3.setMassFromShapes();

			PrismaticJointDef jd3 = new PrismaticJointDef();
			jd3.initialize(ground, body3, bd3.position, new Vec2(0.0f, 1.0f));
			jd3.lowerTranslation = -5.0f;
			jd3.upperTranslation = 5.0f;
			jd3.enableLimit = true;

			m_joint3 = (PrismaticJoint)(m_world.createJoint(jd3));

			GearJointDef jd4 = new GearJointDef();
			jd4.body1 = body1;
			jd4.body2 = body2;
			jd4.joint1 = m_joint1;
			jd4.joint2 = m_joint2;
			jd4.ratio = circle2.radius / circle1.radius;
			m_joint4 = (GearJoint)(m_world.createJoint(jd4));

			GearJointDef jd5 = new GearJointDef();
			jd5.body1 = body2;
			jd5.body2 = body3;
			jd5.joint1 = m_joint2;
			jd5.joint2 = m_joint3;
			jd5.ratio = -1.0f / circle2.radius;
			m_joint5 = (GearJoint)(m_world.createJoint(jd5));
		}
    }

}
