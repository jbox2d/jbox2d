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
import dynamics.joints.PrismaticJoint;
import dynamics.joints.PrismaticJointDef;
import dynamics.joints.RevoluteJointDef;
import dynamics.joints.RevoluteJoint;

public class MotorsAndLimits extends PTest {

    RevoluteJoint m_joint1;

    RevoluteJoint m_joint2;

    PrismaticJoint m_joint3;

    public MotorsAndLimits() {
        super("MotorsAndLimits");
    }

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
            sd.extents = new Vec2(2.0f, 0.5f);
            sd.density = 5.0f;
            sd.friction = 0.05f;

            BodyDef bd = new BodyDef();
            bd.addShape(sd);

            RevoluteJointDef rjd = new RevoluteJointDef();

            Body body = null;
            Body prevBody = ground;
            final float y = 8.0f;

            bd.position = new Vec2(3.0f, y);
            body = world.createBody(bd);

            rjd.anchorPoint = new Vec2(0.0f, y);
            rjd.body1 = prevBody;
            rjd.body2 = body;
            rjd.motorSpeed = (float) Math.PI;
            rjd.motorTorque = 10000.0f;
            rjd.enableMotor = true;

            m_joint1 = (RevoluteJoint) world.createJoint(rjd);

            prevBody = body;

            bd.position = new Vec2(9.0f, y);
            body = world.createBody(bd);

            rjd.anchorPoint = new Vec2(6.0f, y);
            rjd.body1 = prevBody;
            rjd.body2 = body;
            rjd.motorSpeed = (float) (0.5f * Math.PI);
            rjd.motorTorque = 2000.0f;
            rjd.enableMotor = true;
            rjd.lowerAngle = (float) (-0.5f * Math.PI);
            rjd.upperAngle = (float) (0.5f * Math.PI);
            // rjd.enableMotor = false;
            // rjd.minAngle = 0.0f;
            // rjd.maxAngle = 0.0f;
            rjd.enableLimit = true;

            m_joint2 = (RevoluteJoint) world.createJoint(rjd);

            bd.position = new Vec2(-10.0f, 10.0f);
            bd.rotation = (float) (0.5f * Math.PI);
            body = world.createBody(bd);

            PrismaticJointDef pjd = new PrismaticJointDef();
            pjd.anchorPoint = new Vec2(-10.0f, 10.0f);
            pjd.body1 = ground;
            pjd.body2 = body;
            pjd.axis = new Vec2(1.0f, 0.0f);
            pjd.motorSpeed = 10.0f;
            pjd.motorForce = 1000.0f;
            pjd.enableMotor = true;
            pjd.lowerTranslation = 0.0f;
            pjd.upperTranslation = 20.0f;
            pjd.enableLimit = true;

            m_joint3 = (PrismaticJoint) world.createJoint(pjd);
        }
    }

    @Override
    protected void checkKeys() {
        if (newKeyDown['l']) {
            m_joint2.m_enableLimit = !m_joint2.m_enableLimit;
            m_joint3.m_enableLimit = !m_joint3.m_enableLimit;
            m_joint2.m_body1.wakeUp();
            m_joint2.m_body2.wakeUp();
            m_joint3.m_body2.wakeUp();
        }

        if (newKeyDown['m']) {
            m_joint1.m_enableMotor = !m_joint1.m_enableMotor;
            m_joint2.m_enableMotor = !m_joint2.m_enableMotor;
            m_joint3.m_enableMotor = !m_joint3.m_enableMotor;
            m_joint2.m_body1.wakeUp();
            m_joint2.m_body2.wakeUp();
            m_joint3.m_body2.wakeUp();
        }

        if (newKeyDown['p']) {
            m_joint3.m_body2.wakeUp();
            m_joint3.m_motorSpeed = -m_joint3.m_motorSpeed;
        }
    }

    /**
     * Entry point
     */
    public static void main(String[] argv) {
        // new MotorsAndLimits().start();
        PApplet.main(new String[] { "testbed.tests.MotorsAndLimits" });

    }
}
