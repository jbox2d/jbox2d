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
import dynamics.joints.PulleyJointDef;
import dynamics.joints.PulleyJoint;
import dynamics.joints.PrismaticJoint;
import dynamics.joints.PrismaticJointDef;

public class Pulleys extends PTest {

    PulleyJoint m_joint1;

    PrismaticJoint m_joint2;

    public Pulleys() {
        super("Pulleys");
    }

    @Override
    public void go(World world) {
        Body ground = null;
        {
            BoxDef sd = new BoxDef();
            sd.extents.set(50f, 10f);

            BodyDef bd = new BodyDef();
            bd.position.set(0f, -10f);
            bd.addShape(sd);
            ground = world.createBody(bd);

        }

        {
            float a = 2f;
            float y = 16f;
            float L = 12f;

            BoxDef sd = new BoxDef();
            sd.extents.set(2f * a, a);
            sd.density = 5f;

            BodyDef bd = new BodyDef();
            bd.addShape(sd);

            bd.position.set(-10f, y);
            Body body1 = world.createBody(bd);

            bd.position.set(10f, y);
            Body body2 = world.createBody(bd);

            PulleyJointDef pulleyDef = new PulleyJointDef();
            pulleyDef.body1 = body1;
            pulleyDef.body2 = body2;
            pulleyDef.anchorPoint1 = new Vec2(-10.0f, y + a);
            pulleyDef.anchorPoint2 = new Vec2(10.0f, y + a);
            pulleyDef.groundPoint1 = new Vec2(-10.0f, y + a + L);
            pulleyDef.groundPoint2 = new Vec2(10.0f, y + a + L);
            pulleyDef.ratio = 2.0f;

            pulleyDef.maxLength1 = 28.0f;
            pulleyDef.maxLength2 = 12.0f;

            m_joint1 = (PulleyJoint) (world.createJoint(pulleyDef));

            PrismaticJointDef prismDef = new PrismaticJointDef();
            prismDef.body1 = ground;
            prismDef.body2 = body2;
            prismDef.axis.set(0f, 1f);
            prismDef.anchorPoint = body2.getCenterPosition();
            m_joint2 = (PrismaticJoint) (world.createJoint(prismDef));
        }

    }

    /**
     * Entry point
     */
    public static void main(String[] argv) {
        PApplet.main(new String[] { "testbed.tests.Pulleys" });
    }
}
