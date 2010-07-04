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
import collision.CircleDef;
import dynamics.Body;
import dynamics.BodyDef;
import dynamics.World;
import dynamics.joints.GearJoint;
import dynamics.joints.GearJointDef;
import dynamics.joints.PrismaticJoint;
import dynamics.joints.PrismaticJointDef;
import dynamics.joints.RevoluteJoint;
import dynamics.joints.RevoluteJointDef;

public class Gears extends PTest {

    RevoluteJoint m_joint1;

    RevoluteJoint m_joint2;

    PrismaticJoint m_joint3;

    GearJoint m_joint4;

    GearJoint m_joint5;

    public Gears() {
        super("Gears");
    }

    @Override
    public void go(World world) {
        Body ground = null;
        {
            BoxDef sd = new BoxDef();
            sd.extents.set(50.0f, 10.0f);

            BodyDef bd = new BodyDef();
            bd.position.set(0.0f, -10.0f);
            bd.addShape(sd);
            ground = world.createBody(bd);
        }

        {
            CircleDef circle1 = new CircleDef();
            circle1.radius = 1.0f;
            circle1.density = 5.0f;

            CircleDef circle2 = new CircleDef();
            circle2.radius = 2.0f;
            circle2.density = 5.0f;

            BoxDef box = new BoxDef();
            box.extents.set(0.5f, 5.0f);
            box.density = 5.0f;

            BodyDef bd1 = new BodyDef();
            bd1.addShape(circle1);
            bd1.position.set(-3.0f, 12.0f);
            Body body1 = world.createBody(bd1);

            RevoluteJointDef jd1 = new RevoluteJointDef();
            jd1.anchorPoint = bd1.position;
            jd1.body1 = ground;
            jd1.body2 = body1;
            m_joint1 = (RevoluteJoint) world.createJoint(jd1);

            BodyDef bd2 = new BodyDef();
            bd2.addShape(circle2);
            bd2.position.set(0.0f, 12.0f);
            Body body2 = world.createBody(bd2);

            RevoluteJointDef jd2 = new RevoluteJointDef();
            jd2.body1 = ground;
            jd2.body2 = body2;
            jd2.anchorPoint = bd2.position;
            m_joint2 = (RevoluteJoint) world.createJoint(jd2);

            BodyDef bd3 = new BodyDef();
            bd3.addShape(box);
            bd3.position.set(2.5f, 12.0f);
            Body body3 = world.createBody(bd3);

            PrismaticJointDef jd3 = new PrismaticJointDef();
            jd3.body1 = ground;
            jd3.body2 = body3;
            jd3.anchorPoint = bd3.position;
            jd3.axis.set(0.0f, 1.0f);
            jd3.lowerTranslation = -5.0f;
            jd3.upperTranslation = 5.0f;
            jd3.enableLimit = true;

            m_joint3 = (PrismaticJoint) world.createJoint(jd3);

            GearJointDef jd4 = new GearJointDef();
            jd4.body1 = body1;
            jd4.body2 = body2;
            jd4.joint1 = m_joint1;
            jd4.joint2 = m_joint2;
            jd4.ratio = circle2.radius / circle1.radius;
            m_joint4 = (GearJoint) world.createJoint(jd4);

            GearJointDef jd5 = new GearJointDef();
            jd5.body1 = body2;
            jd5.body2 = body3;
            jd5.joint1 = m_joint2;
            jd5.joint2 = m_joint3;
            jd5.ratio = -1.0f / circle2.radius;
            m_joint5 = (GearJoint) m_world.createJoint(jd5);
        }
    }

    /**
     * Entry point
     */
    public static void main(String[] argv) {
        PApplet.main(new String[] { "testbed.tests.Gears" });
    }
}
