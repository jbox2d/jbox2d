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

package org.jbox2d.testbed.tests;

import org.jbox2d.collision.CircleDef;
import org.jbox2d.collision.PolygonDef;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.testbed.AbstractExample;
import org.jbox2d.testbed.TestbedMain;

public class CCDTest extends AbstractExample {
	private boolean firstTime = true;
	
	public CCDTest(TestbedMain _parent) {
		super(_parent);
	}
	
	@Override
	public void create() {
		
		if (firstTime) {
			setCamera(0f, 20f, 20f);
			firstTime = false;
		}
		
		final float k_restitution = 1.4f;

		{
			BodyDef bd = new BodyDef();
			bd.position.set(0.0f, 20.0f);
			Body body = m_world.createStaticBody(bd);

			PolygonDef sd = new PolygonDef();
			sd.density = 0.0f;
			sd.restitution = k_restitution;

			sd.setAsBox(0.1f, 10.0f, new Vec2(-10.0f, 0.0f), 0.0f);
			body.createShape(sd);

			sd.setAsBox(0.1f, 10.0f, new Vec2(10.0f, 0.0f), 0.0f);
			body.createShape(sd);

			sd.setAsBox(0.1f, 10.0f,new Vec2(0.0f, -10.0f), 0.5f * 3.1415f);
			body.createShape(sd);

			sd.setAsBox(0.1f, 10.0f, new Vec2(0.0f, 10.0f), -0.5f * 3.1415f);
			body.createShape(sd);
		}

		{
			PolygonDef sd_bottom = new PolygonDef();
			sd_bottom.setAsBox( 1.5f, 0.15f );
			sd_bottom.density = 4.0f;

			PolygonDef sd_left = new PolygonDef();
			sd_left.setAsBox(0.15f, 2.7f, new Vec2(-1.45f, 2.35f), 0.2f);
			sd_left.density = 4.0f;

			PolygonDef sd_right = new PolygonDef();
			sd_right.setAsBox(0.15f, 2.7f, new Vec2(1.45f, 2.35f), -0.2f);
			sd_right.density = 4.0f;

			BodyDef bd = new BodyDef();
			bd.position.set( 0.0f, 15.0f );
			bd.isBullet = true;
			Body body = m_world.createDynamicBody(bd);
			body.createShape(sd_bottom);
			body.createShape(sd_left);
			body.createShape(sd_right);
			body.setMassFromShapes();
		}


		for (int i = 0; i < 10; ++i) {
			BodyDef bd = new BodyDef();
			bd.position.set(0.0f, 15.5f + i);
			bd.isBullet = true;
			Body body = m_world.createDynamicBody(bd);
			body.setAngularVelocity(parent.random(-50.0f, 50.0f));

			CircleDef sd = new CircleDef();
			sd.radius = 0.25f;
			sd.density = 1.0f;
			sd.restitution = 0.0f;
			sd.friction = 0.05f;
			body.createShape(sd);
			body.setMassFromShapes();
		}

	}

	@Override
	public String getName() {
		return "Continuous Collision Test";
	}

}
