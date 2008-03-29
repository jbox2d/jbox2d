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


public class DominoTower extends AbstractExample {
	private boolean firstTime = true;
    final float dwidth = .20f;
    final float dheight = 1.0f;
    float ddensity;// = 10f;
    final float dfriction = 0.1f;
    int baseCount = 25;
    
    public DominoTower(TestbedMain _parent) {
        super(_parent);
    }

    public void makeDomino(float x, float y, boolean horizontal, World world) {

    	PolygonDef sd = new PolygonDef();
        sd.setAsBox(.5f*dwidth, .5f*dheight);
        sd.density = ddensity;
        BodyDef bd = new BodyDef();
        sd.friction = dfriction;
        sd.restitution = 0.65f;
        bd.position = new Vec2(x, y);
        bd.angle = horizontal? (float)(Math.PI/2.0):0f;
        Body myBody = world.createDynamicBody(bd);
        myBody.createShape(sd);
        myBody.setMassFromShapes();
    }

    @Override
    public void create() {
    	if (firstTime) {
			setCamera(0f, 12f, 10f);
			firstTime = false;
	    	settings.hz = 120;
		}
    	
        { // Floor
            PolygonDef sd = new PolygonDef();
            sd.setAsBox(50.0f, 10.0f);

            BodyDef bd = new BodyDef();
            bd.position = new Vec2(0.0f, -10.0f);
            m_world.createStaticBody(bd).createShape(sd);
        }
        
        {
            ddensity = 10f;
            //Make bullet
            PolygonDef sd = new PolygonDef();
            sd.setAsBox(.7f, .7f);
            sd.density = 35f;
            BodyDef bd = new BodyDef();
            sd.friction = 0f;
            sd.restitution = 0.85f;
            bd.isBullet = true;
            //bd.addShape(sd);
            bd.position = new Vec2(30f, 50f);
            Body b = m_world.createDynamicBody(bd);
            b.createShape(sd);
            b.setLinearVelocity(new Vec2(-25f,-25f));
            b.setAngularVelocity(6.7f);
            b.setMassFromShapes();
            sd.density = 25f;
            bd.position = new Vec2(-30, 25f);
            b = m_world.createDynamicBody(bd);
            b.createShape(sd);
            b.setLinearVelocity(new Vec2(35f, -10f));
            b.setAngularVelocity(-8.3f);
            b.setMassFromShapes();
        }

        { 
            
            //Make base
            for (int i=0; i<baseCount; ++i) {
                float currX = i*1.5f*dheight - (1.5f*dheight*baseCount/2f);
                makeDomino(currX, dheight/2.0f, false, m_world);
                makeDomino(currX, dheight+dwidth/2.0f, true, m_world);
            }
            //Make 'I's
            for (int j=1; j<baseCount; ++j) {
                if (j > 3) ddensity *= .8f;
                float currY = dheight*.5f + (dheight+2f*dwidth)*.99f*j; //y at center of 'I' structure
                
                for (int i=0; i<baseCount - j; ++i) {
                    float currX = i*1.5f*dheight - (1.5f*dheight*(baseCount-j)/2f);// + random(-.05f, .05f);
                    ddensity *= 2.5f;
                    if (i==0) {
                        makeDomino(currX - (1.25f*dheight) + .5f*dwidth, currY-dwidth, false, m_world);
                    }
                    if (i==baseCount-j-1) {
                        if (j != 1) makeDomino(currX + (1.25f*dheight) - .5f*dwidth, currY-dwidth, false, m_world);
                    }
                    ddensity /= 2.5f;
                    makeDomino(currX, currY, false, m_world);
                    makeDomino(currX, currY+.5f*(dwidth+dheight), true, m_world);
                    makeDomino(currX, currY-.5f*(dwidth+dheight), true, m_world);
                }
            }
        }
    }

    public String getName() {
    	return "Domino Tower Stress Test";
    }
}
