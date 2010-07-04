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
package testbed.tests;

import processing.core.PApplet;
import testbed.PTest;
import collision.BoxDef;

import common.Vec2;

import dynamics.BodyDef;
import dynamics.World;

public class DominoTower extends PTest {

    final float dwidth = .20f;
    final float dheight = 1.0f;
    float ddensity;// = 10f;
    final float dfriction = 0.1f;
    int baseCount = 25;
    
    public DominoTower() {
        super("DominoTower");
    }

    public void makeDomino(float x, float y, boolean horizontal, World world) {

        BoxDef sd = new BoxDef();
        sd.extents = new Vec2(.5f*dwidth, .5f*dheight);
        sd.density = ddensity;
        BodyDef bd = new BodyDef();
        sd.friction = dfriction;
        sd.restitution = 0.65f;
        bd.addShape(sd);
        bd.position = new Vec2(x, y);
        bd.rotation = horizontal? (float)(Math.PI/2.0):0f;
        world.createBody(bd);
    }
    
    public void postSetup() {
        transY += 100f;
        settings.hz = 120.0f;
    }

    @Override
    public void go(World world) {
        { // Floor
            BoxDef sd = new BoxDef();
            sd.extents = new Vec2(50.0f, 10.0f);

            BodyDef bd = new BodyDef();
            bd.position = new Vec2(0.0f, -10.0f);
            bd.addShape(sd);
            world.createBody(bd);
        }
        
        {
            ddensity = 10f;
            //Make bullet
            BoxDef sd = new BoxDef();
            sd.extents = new Vec2(.7f, .7f);
            sd.density = 35f;
            BodyDef bd = new BodyDef();
            sd.friction = 0f;
            sd.restitution = 0.85f;
            bd.addShape(sd);
            bd.position = new Vec2(30f, 50f);
            bd.linearVelocity = new Vec2(-25f, -25f);
            bd.angularVelocity = 6.7f;
            world.createBody(bd);
            sd.density = 25f;
            bd.position = new Vec2(-30, 25f);
            bd.linearVelocity = new Vec2(35f, -10f);
            bd.angularVelocity = -8.3f;
            world.createBody(bd);
        }

        { 
            
            //Make base
            for (int i=0; i<baseCount; ++i) {
                float currX = i*1.5f*dheight - (1.5f*dheight*baseCount/2f);
                makeDomino(currX, dheight/2.0f, false, world);
                makeDomino(currX, dheight+dwidth/2.0f, true, world);
            }
            //Make 'I's
            for (int j=1; j<baseCount; ++j) {
                if (j > 3) ddensity *= .8f;
                float currY = dheight*.5f + (dheight+2f*dwidth)*.99f*j; //y at center of 'I' structure
                
                for (int i=0; i<baseCount - j; ++i) {
                    float currX = i*1.5f*dheight - (1.5f*dheight*(baseCount-j)/2f);// + random(-.05f, .05f);
                    ddensity *= 2.5f;
                    if (i==0) {
                        makeDomino(currX - (1.25f*dheight) + .5f*dwidth, currY-dwidth, false, world);
                    }
                    if (i==baseCount-j-1) {
                        if (j != 1) makeDomino(currX + (1.25f*dheight) - .5f*dwidth, currY-dwidth, false, world);
                    }
                    ddensity /= 2.5f;
                    makeDomino(currX, currY, false, world);
                    makeDomino(currX, currY+.5f*(dwidth+dheight), true, world);
                    makeDomino(currX, currY-.5f*(dwidth+dheight), true, world);
                }
            }
        }
    }

    public static void main(String[] args) {
        PApplet.main(new String[] { "testbed.tests.DominoTower" });
    }
}
