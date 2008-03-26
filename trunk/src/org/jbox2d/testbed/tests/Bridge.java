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
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import org.jbox2d.testbed.AbstractExample;
import org.jbox2d.testbed.TestbedMain;

import processing.core.PApplet;



public class Bridge extends AbstractExample {
	private boolean firstTime = true;
	
    public Bridge(TestbedMain _parent) {
		super(_parent);
	}

	public void create() {
		if (firstTime) {
			setCamera(0.0f,10.0f,20.0f);
			firstTime = false;
		}
		
    	Body ground = null;
		{
			PolygonDef sd = new PolygonDef();
			sd.setAsBox(50.0f, 0.2f);

			BodyDef bd = new BodyDef();
			bd.position.set(0.0f, 0.0f);
			ground = m_world.createStaticBody(bd);
			ground.createShape(sd);
		}

		{
			PolygonDef sd = new PolygonDef();
			sd.setAsBox(0.65f, 0.125f);
			sd.density = 20.0f;
			sd.friction = 0.2f;

			RevoluteJointDef jd = new RevoluteJointDef();
			int numPlanks = 30;

			Body prevBody = ground;
			for (int i = 0; i < numPlanks; ++i) {
				BodyDef bd = new BodyDef();
				bd.position.set(-14.5f + 1.0f * i, 5.0f);
				Body body = m_world.createDynamicBody(bd);
				body.createShape(sd);
				body.setMassFromShapes();

				Vec2 anchor = new Vec2(-15.0f + 1.0f * i, 5.0f);
				jd.initialize(prevBody, body, anchor);
				m_world.createJoint(jd);

				prevBody = body;
			}

			Vec2 anchor = new Vec2(-15.0f + 1.0f * numPlanks, 5.0f);
			jd.initialize(prevBody, ground, anchor);
			m_world.createJoint(jd);
			
			PolygonDef pd2 = new PolygonDef();
			pd2.setAsBox(1.0f,1.0f);
			pd2.density = 5.0f;
			pd2.friction = 0.2f;
			pd2.restitution = 0.1f;
			BodyDef bd2 = new BodyDef();
			bd2.position.set(0.0f, 10.0f);
			Body body2 = m_world.createDynamicBody(bd2);
			body2.createShape(pd2);
			body2.setMassFromShapes();
			
			CircleDef cd = new CircleDef();
			cd.radius = 0.9f;
			cd.density = 5.0f;
			cd.friction = 0.2f;
			BodyDef bd3 = new BodyDef();
			bd3.position.set(0.0f, 12.0f);
			Body body3 = m_world.createDynamicBody(bd3);
			body3.createShape(cd);
			cd.localPosition.set(0.0f,1.0f);
			body3.createShape(cd);
			body3.setMassFromShapes();
		}
    }

	@Override
	public String getName() {
		return "Bridge";
	}

}
