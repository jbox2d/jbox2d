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
import dynamics.joints.DistanceJointDef;
import dynamics.joints.Joint;

public class Web extends PTest {

    Joint m_joints[];

    public Web() {
        super("Web");
    }

    @Override
    public void go(World world) {
        m_joints = new Joint[8];

        Body ground;
        {
            BoxDef sd = new BoxDef();
            sd.extents = new Vec2(50.0f, 10.0f);

            BodyDef bd = new BodyDef();
            bd.position = new Vec2(0.0f, -10.0f);
            bd.addShape(sd);
            ground = m_world.createBody(bd);
        }

        BoxDef sd = new BoxDef();
        sd.extents = new Vec2(0.5f, 0.5f);
        sd.density = 5.0f;
        sd.friction = 0.2f;

        BodyDef bd = new BodyDef();
        bd.addShape(sd);

        bd.position = new Vec2(-5.0f, 5.0f);
        Body b1 = world.createBody(bd);

        bd.position = new Vec2(5.0f, 5.0f);
        Body b2 = world.createBody(bd);

        bd.position = new Vec2(5.0f, 15.0f);
        Body b3 = world.createBody(bd);

        bd.position = new Vec2(-5.0f, 15.0f);
        Body b4 = world.createBody(bd);

        DistanceJointDef jd = new DistanceJointDef();

        jd.body1 = ground;
        jd.body2 = b2;
        jd.anchorPoint1 = new Vec2(10.0f, 0.0f);
        jd.anchorPoint2 = b2.m_position.add(new Vec2(0.5f, -0.5f));
        m_joints[0] = world.createJoint(jd);

        jd.body1 = ground;
        jd.body2 = b4;
        jd.anchorPoint1 = new Vec2(-10.0f, 20.0f);
        jd.anchorPoint2 = b4.m_position.add(new Vec2(-0.5f, 0.5f));
        m_joints[1] = world.createJoint(jd);

        jd.body1 = ground;
        jd.body2 = b1;
        jd.anchorPoint1 = new Vec2(-10.0f, 0.0f);
        jd.anchorPoint2 = b1.m_position.add(new Vec2(-0.5f, -0.5f));
        m_joints[2] = world.createJoint(jd);

        jd.body1 = ground;
        jd.body2 = b3;
        jd.anchorPoint1 = new Vec2(10.0f, 20.0f);
        jd.anchorPoint2 = b3.m_position.add(new Vec2(0.5f, 0.5f));
        m_joints[3] = world.createJoint(jd);

        jd.body1 = b1;
        jd.body2 = b2;
        jd.anchorPoint1 = b1.m_position.add(new Vec2(0.5f, 0.0f));
        jd.anchorPoint2 = b2.m_position.add(new Vec2(-0.5f, 0.0f));
        m_joints[4] = world.createJoint(jd);

        jd.body1 = b2;
        jd.body2 = b3;
        jd.anchorPoint1 = b2.m_position.add(new Vec2(0.0f, 0.5f));
        jd.anchorPoint2 = b3.m_position.add(new Vec2(0.0f, -0.5f));
        m_joints[5] = world.createJoint(jd);

        jd.body1 = b3;
        jd.body2 = b4;
        jd.anchorPoint1 = b3.m_position.add(new Vec2(-0.5f, 0.0f));
        jd.anchorPoint2 = b4.m_position.add(new Vec2(0.5f, 0.0f));
        m_joints[6] = world.createJoint(jd);

        jd.body1 = b4;
        jd.body2 = b1;
        jd.anchorPoint1 = b4.m_position.add(new Vec2(0.0f, -0.5f));
        jd.anchorPoint2 = b1.m_position.add(new Vec2(0.0f, 0.5f));
        m_joints[7] = world.createJoint(jd);
    }

    @Override
    protected void checkKeys() {
        if (newKeyDown['b']) {
            for (int i = 0; i < 8; ++i) {
                if (m_joints[i] != null) {
                    m_world.destroyJoint(m_joints[i]);
                    m_joints[i] = null;
                    break;
                }
            }
        }
    }

    /**
     * Entry point
     */
    public static void main(String[] argv) {
        PApplet.main(new String[] { "testbed.tests.Web" });
    }
}
