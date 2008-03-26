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

import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import org.jbox2d.collision.*;
import org.jbox2d.testbed.AbstractExample;
import org.jbox2d.testbed.TestbedMain;

import processing.core.PApplet;



public class Chain extends AbstractExample {
	private boolean firstTime = true;
	
	public Chain(TestbedMain _parent) {
		super(_parent);
	}
	
	public void create() {
		if (firstTime) {
			setCamera(0.0f,10.0f,10.0f);
			firstTime = false;
		}
		
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
			PolygonDef sd = new PolygonDef();
			sd.setAsBox(0.6f, 0.125f);
			sd.density = 20.0f;
			sd.friction = 0.2f;

			RevoluteJointDef jd = new RevoluteJointDef();
			jd.collideConnected = false;

			float y = 25.0f;
			Body prevBody = ground;
			for (int i = 0; i < 30; ++i) {
				BodyDef bd = new BodyDef();
				bd.position.set(0.5f + i, y);
				Body body = m_world.createDynamicBody(bd);
				body.createShape(sd);
				body.setMassFromShapes();
				
				Vec2 anchor = new Vec2((float)i, y);
				jd.initialize(prevBody, body, anchor);
				m_world.createJoint(jd);
				
				prevBody = body;
			}
		}
	}
	
	public String getName() {
		return "Chain Test";
	}
}
