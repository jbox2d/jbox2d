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

public class Bridge extends PTest {

    public Bridge() {
        super("Bridge");
    }

    public void go(World world) {
        BoxDef sd = new BoxDef();
        sd.extents = new Vec2(50.0f, 10.0f);
        // System.out.println(sd.box.m_extents.y);
        BodyDef bd = new BodyDef();
        bd.position = new Vec2(0.0f, -10.0f);
        bd.addShape(sd);
        Body ground = world.createBody(bd);

        BoxDef sd2 = new BoxDef();
        sd2.extents = new Vec2(0.5f, 0.125f);
        sd2.density = 20.0f;
        sd2.friction = 0.2f;

        BodyDef bd2 = new BodyDef();
        bd2.addShape(sd2);

        RevoluteJointDef jd = new RevoluteJointDef();
        int numPlanks = 25;

        Body prevBody = ground;
        for (int i = 0; i < numPlanks; ++i) {
            bd2.position = new Vec2(-15.5f + 1.25f * i, 5.0f);
            // System.out.println("Creating plank "+i);
            Body body = world.createBody(bd2);

            jd.anchorPoint = new Vec2(-16.125f + 1.25f * i, 5.0f);
            jd.body1 = prevBody;
            jd.body2 = body;
            world.createJoint(jd);

            prevBody = body;
        }

        jd.anchorPoint = new Vec2(-16.125f + 1.25f * numPlanks, 5.0f);
        jd.body1 = prevBody;
        jd.body2 = ground;
        world.createJoint(jd);
    }

    /**
     * Entry point
     */
    public static void main(String[] argv) {
        PApplet.main(new String[] { "testbed.tests.Bridge" });
    }
}
