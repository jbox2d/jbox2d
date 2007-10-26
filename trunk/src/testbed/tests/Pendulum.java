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

import dynamics.Body;
import dynamics.BodyDef;
import dynamics.World;
import dynamics.joints.RevoluteJointDef;

public class Pendulum extends PTest {

    public Pendulum() {
        super("Pendulum");
    }

    @Override
    public void go(World world) {
        Body ground = null;
        {
            BoxDef sd = new BoxDef();
            sd.extents = new Vec2(50.0f, 10.0f);

            BodyDef bd = new BodyDef();
            bd.position = new Vec2(0.0f, -10.0f);
            bd.addShape(sd);
            ground = world.createBody(bd);
        }

        {
            BoxDef sd = new BoxDef();
            sd.extents = new Vec2(0.375f, 0.125f);
            sd.density = 20.0f;
            sd.friction = 0.2f;

            BodyDef bd = new BodyDef();
            bd.addShape(sd);

            RevoluteJointDef jd = new RevoluteJointDef();

            final float y = 25.0f;
            Body prevBody = ground;
            for (int i = 0; i < 30; ++i) {
                bd.position = new Vec2(0.5f + i, y);
                Body body = world.createBody(bd);

                jd.anchorPoint = new Vec2(i, y);
                jd.body1 = prevBody;
                jd.body2 = body;
                world.createJoint(jd);

                prevBody = body;
            }
        }
    }

    /**
     * Entry point
     */
    public static void main(String[] argv) {
        PApplet.main(new String[] { "testbed.tests.Pendulum" });
    }
}
