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



public class Domino extends AbstractExample {

    public Domino(TestbedMain _parent) {
        super(_parent);
    }

    @Override
    public void create() {
        { // Floor
            PolygonDef sd = new PolygonDef();
            sd.setAsBox(50.0f, 10.0f);

            BodyDef bd = new BodyDef();
            bd.position = new Vec2(0.0f, -10.0f);
            m_world.createStaticBody(bd).createShape(sd);
            
        }

        { // Platforms
            for (int i = 0; i < 4; i++) {
            	PolygonDef sd = new PolygonDef();
                sd.setAsBox(15.0f, 0.125f);

                BodyDef bd = new BodyDef();
                bd.position = new Vec2(0.0f, 5f + 5f * i);
                m_world.createStaticBody(bd).createShape(sd);
            }
        }

        {
        	PolygonDef sd = new PolygonDef();
            sd.setAsBox(0.125f, 2f);
            sd.density = 25.0f;

            BodyDef bd = new BodyDef();
            
            float friction = .5f;
            int numPerRow = 25;

            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < numPerRow; j++) {
                    sd.friction = friction;
                    bd.position = new Vec2(-14.75f + j
                            * (29.5f / (numPerRow - 1)), 7.3f + 5f * i);
                    if (i == 2 && j == 0) {
                        bd.angle = -0.1f;
                        bd.position.x += .1f;
                    }
                    else if (i == 3 && j == numPerRow - 1) {
                        bd.angle = .1f;
                        bd.position.x -= .1f;
                    }
                    else
                        bd.angle = 0f;
                    Body myBody = m_world.createDynamicBody(bd);
                    myBody.createShape(sd);
                    myBody.setMassFromShapes();
                }
            }
        }
    }

    public String getName() {
    	return "Domino Test";
    }
}
