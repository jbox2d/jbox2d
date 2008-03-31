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

import org.jbox2d.collision.PolygonDef;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.testbed.AbstractExample;
import org.jbox2d.testbed.TestbedMain;

public class VaryingFriction extends AbstractExample {

    public VaryingFriction(TestbedMain _parent) {
        super(_parent);
    }
    
    public String getName() {
    	return "Varying Friction";
    }

    @Override
    public void create() {
		{
			PolygonDef sd = new PolygonDef();
			sd.setAsBox(100.0f, 20.0f);

			BodyDef bd = new BodyDef();
			bd.position.set(0.0f, -20.0f);
			Body ground = m_world.createStaticBody(bd);
			ground.createShape(sd);
		}

		{
			PolygonDef sd = new PolygonDef();
			sd.setAsBox(13.0f, 0.25f);

			BodyDef bd = new BodyDef();
			bd.position.set(-4.0f, 22.0f);
			bd.angle = -0.25f;

			Body ground = m_world.createStaticBody(bd);
			ground.createShape(sd);
		}

		{
			PolygonDef sd = new PolygonDef();
			sd.setAsBox(0.25f, 1.0f);

			BodyDef bd = new BodyDef();
			bd.position.set(10.5f, 19.0f);

			Body ground = m_world.createStaticBody(bd);
			ground.createShape(sd);
		}

		{
			PolygonDef sd = new PolygonDef();
			sd.setAsBox(13.0f, 0.25f);

			BodyDef bd = new BodyDef();
			bd.position.set(4.0f, 14.0f);
			bd.angle = 0.25f;

			Body ground = m_world.createStaticBody(bd);
			ground.createShape(sd);
		}

		{
			PolygonDef sd = new PolygonDef();
			sd.setAsBox(0.25f, 1.0f);

			BodyDef bd = new BodyDef();
			bd.position.set(-10.5f, 11.0f);

			Body ground = m_world.createStaticBody(bd);
			ground.createShape(sd);
		}

		{
			PolygonDef sd = new PolygonDef();
			sd.setAsBox(13.0f, 0.25f);

			BodyDef bd = new BodyDef();
			bd.position.set(-4.0f, 6.0f);
			bd.angle = -0.25f;

			Body ground = m_world.createStaticBody(bd);
			ground.createShape(sd);
		}

		{
			PolygonDef sd = new PolygonDef();
			sd.setAsBox(0.5f, 0.5f);
			sd.density = 25.0f;

			float[] friction = {0.75f, 0.5f, 0.35f, 0.1f, 0.0f};

			for (int i = 0; i < 5; ++i)
			{
				BodyDef bd = new BodyDef();
				bd.position.set(-15.0f + 4.0f * i, 28.0f);
				Body body = m_world.createDynamicBody(bd);

				sd.friction = friction[i];
				body.createShape(sd);
				body.setMassFromShapes();
			}
		}
	}

}
