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

public class BugTest extends AbstractExample {
	private boolean firstTime;
	
	public BugTest(TestbedMain _parent) {
		super(_parent);
		firstTime = true;
	}
	
	@Override
	public void create() {
		
		if (firstTime) {
			setCamera(0f, 20f, 20f);
			firstTime = false;
		}
		
		final float k_restitution = 1.4f;
		this.settings.pause = true;
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
			sd_bottom.restitution = 1.0f;

			PolygonDef sd_left = new PolygonDef();
			sd_left.setAsBox(0.15f, 2.7f, new Vec2(-1.45f, 2.35f), 0.2f);
			sd_left.density = 4.0f;

			PolygonDef sd_right = new PolygonDef();
			sd_right.setAsBox(0.15f, 2.7f, new Vec2(1.45f, 2.35f), -0.2f);
			sd_right.density = 4.0f;

			BodyDef bd = new BodyDef();
			bd.position.set( 0.0f, 12.0f );
			bd.isBullet = true;
			Body body = m_world.createDynamicBody(bd);
			body.createShape(sd_bottom);
			//body.createShape(sd_left);
			//body.createShape(sd_right);
			body.setMassFromShapes();
			
			CircleDef cd = new CircleDef();
			cd.radius = 0.5f;
			cd.density = 40.0f;
			BodyDef bd2 = new BodyDef();
			bd2.position.set(0.0f,14.0f);
			//bd2.linearVelocity.Set(0.1f,-100.0f);
			bd2.isBullet = true;
			Body myBod = m_world.createDynamicBody(bd2);
			myBod.createShape(cd);
			myBod.setLinearVelocity(new Vec2(0.1f,-100.0f));
			myBod.setMassFromShapes();
			
		}

	}

	@Override
	public String getName() {
		return "Bug Test";
	}

}