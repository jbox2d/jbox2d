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



public class VerticalStack extends AbstractExample {
	private boolean firstTime = true;
	
    public VerticalStack(TestbedMain parent) {
        super(parent);
    }
    
    public String getName() {
    	return "Vertical Stack";
    }

    public void create() {
    	if (firstTime) {
			setCamera(0f, 10f, 10f);
			firstTime = false;
		}
    	
    	{
			PolygonDef sd = new PolygonDef();
			sd.setAsBox(50.0f, 10.0f, new Vec2(0.0f, -10.0f), 0.0f);

			BodyDef bd = new BodyDef();
			bd.position.set(0.0f, 0.0f);
			Body ground = m_world.createStaticBody(bd);
			ground.createShape(sd);

			sd.setAsBox(0.1f, 10.0f, new Vec2(20.0f, 10.0f), 0.0f);
			ground.createShape(sd);
		}

		float[] xs = {0.0f, -10.0f, -5.0f, 5.0f, 10.0f};

		for (int j = 0; j < xs.length; ++j)
		{
			PolygonDef sd = new PolygonDef();
			sd.setAsBox(0.5f, 0.5f);
			sd.density = 1.0f;
			sd.friction = 0.3f;

			for (int i = 0; i < 12; ++i)
			{
				BodyDef bd = new BodyDef();

				// For this test we are using continuous physics for all boxes.
				// This is a stress test, you normally wouldn't do this for
				// performance reasons.
				bd.isBullet = true;
				bd.allowSleep = true;

				//float32 x = b2Random(-0.1f, 0.1f);
				//float32 x = i % 2 == 0 ? -0.025f : 0.025f;
				bd.position.set(xs[j]+parent.random(-.05f,.05f), 0.752f + 1.54f * i);
				//bd.position.Set(xs[j], 2.51f + 4.02f * i);
				Body body = m_world.createDynamicBody(bd);

				body.createShape(sd);
				body.setMassFromShapes();
			}
		}
    }
    
    public String getExampleInstructions() {
    	return "Press , to shoot sideways bullet\n";
    }
    
    public void postStep() {
    	if (newKeyDown[',']) {
    		launchBomb(new Vec2(-40.0f,parent.random(1.0f,10.0f)),new Vec2(200.0f,parent.random(-5.0f,5.0f)));
    	}
    }

}
