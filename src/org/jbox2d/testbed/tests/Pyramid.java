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
import org.jbox2d.dynamics.World;
import org.jbox2d.testbed.AbstractExample;
import org.jbox2d.testbed.TestbedMain;

import processing.core.PApplet;



public class Pyramid extends AbstractExample {
	private boolean firstTime = true;
	
    public Pyramid(TestbedMain t) {
        super(t);
    }
    
    public String getName() {
    	return "Pyramid Stress Test";
    }

    @Override
    public void create() {
    	if (firstTime) {
			setCamera(2f, 12f, 10f);
			firstTime = false;
		}
    	
    	{
			PolygonDef sd = new PolygonDef();
			sd.setAsBox(50.0f, 10.0f);

			BodyDef bd = new BodyDef();
			bd.position.set(0.0f, -10.0f);
			Body ground = m_world.createStaticBody(bd);
			ground.createShape(sd);
		}

		{
			PolygonDef sd = new PolygonDef();
			float a = 0.5f;
			sd.setAsBox(a, a);
			sd.density = 5.0f;
			sd.restitution = 0.0f;
			sd.friction = 0.9f;

			Vec2 x = new Vec2(-10.0f, 0.75f);
			Vec2 y = new Vec2();
			Vec2 deltaX = new Vec2(0.5625f, 2.0f);
			Vec2 deltaY = new Vec2(1.125f, 0.0f);

			for (int i = 0; i < 25; ++i) {
				y.set(x);

				for (int j = i; j < 25; ++j) {
					BodyDef bd = new BodyDef();
					bd.position.set(y);
					Body body = m_world.createDynamicBody(bd);
					body.createShape(sd);
					body.setMassFromShapes();

					y.addLocal(deltaY);
				}

				x.addLocal(deltaX);
			}
		}
    }

}
