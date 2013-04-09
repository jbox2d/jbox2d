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

public class Domino extends PTest {

    public Domino() {
        super("Domino");
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

        { // Platforms
            for (int i = 0; i < 4; i++) {
                BoxDef sd = new BoxDef();
                sd.extents = new Vec2(15.0f, 0.125f);

                BodyDef bd = new BodyDef();
                bd.position = new Vec2(0.0f, 5f + 5f * i);
                bd.addShape(sd);
                world.createBody(bd);
            }
        }

        {
            BoxDef sd = new BoxDef();
            sd.extents = new Vec2(0.125f, 2f);
            sd.density = 25.0f;

            BodyDef bd = new BodyDef();
            bd.addShape(sd);

            float friction = .5f;
            int numPerRow = 25;

            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < numPerRow; j++) {
                    sd.friction = friction;
                    bd.position = new Vec2(-14.75f + j
                            * (29.5f / (numPerRow - 1)), 7.3f + 5f * i);
                    if (i == 2 && j == 0) {
                        bd.rotation = -0.1f;
                        bd.position.x += .1f;
                    }
                    else if (i == 3 && j == numPerRow - 1) {
                        bd.rotation = .1f;
                        bd.position.x -= .1f;
                    }
                    else
                        bd.rotation = 0f;
                    world.createBody(bd);
                }
            }
        }
    }

    public static void main(String[] args) {
        PApplet.main(new String[] { "testbed.tests.Domino" });
    }
}
