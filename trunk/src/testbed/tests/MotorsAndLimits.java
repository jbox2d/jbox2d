package testbed.tests;

import processing.core.PApplet;
import testbed.PTest;
import collision.ShapeDescription;
import collision.ShapeType;

import common.Vec2;

import dynamics.Body;
import dynamics.BodyDescription;
import dynamics.World;
import dynamics.joints.PrismaticJoint;
import dynamics.joints.PrismaticJointDescription;
import dynamics.joints.RevoluteDescription;
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
            ShapeDescription sd = new ShapeDescription(ShapeType.BOX_SHAPE);
            sd.box.m_extents = new Vec2(50.0f, 10.0f);

            BodyDescription bd = new BodyDescription();
            bd.position = new Vec2(0.0f, -10.0f);
            bd.addShape(sd);
            ground = world.CreateBody(bd);
        }

        {
            ShapeDescription sd = new ShapeDescription(ShapeType.BOX_SHAPE);
            sd.box.m_extents = new Vec2(2.0f, 0.5f);
            sd.density = 5.0f;
            sd.friction = 0.05f;

            BodyDescription bd = new BodyDescription();
            bd.addShape(sd);

            RevoluteDescription rjd = new RevoluteDescription();

            Body body = null;
            Body prevBody = ground;
            final float y = 8.0f;

            bd.position = new Vec2(3.0f, y);
            body = world.CreateBody(bd);

            rjd.anchorPoint = new Vec2(0.0f, y);
            rjd.body1 = prevBody;
            rjd.body2 = body;
            rjd.motorSpeed = (float) Math.PI;
            rjd.motorTorque = 10000.0f;
            rjd.enableMotor = true;

            m_joint1 = (RevoluteJoint) world.CreateJoint(rjd);

            prevBody = body;

            bd.position = new Vec2(9.0f, y);
            body = world.CreateBody(bd);

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

            m_joint2 = (RevoluteJoint) world.CreateJoint(rjd);

            bd.position = new Vec2(-10.0f, 10.0f);
            bd.rotation = (float) (0.5f * Math.PI);
            body = world.CreateBody(bd);

            PrismaticJointDescription pjd = new PrismaticJointDescription();
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

            m_joint3 = (PrismaticJoint) world.CreateJoint(pjd);
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

    /*
     * @Override protected void renderGUI(Graphics2D g) { super.renderGUI(g); //
     * g.drawString(5, m_textLine, // "Keys: l - limits, m - motors, p -
     * prismatic speed"); // m_textLine += 15; // float torque1 =
     * m_joint1.GetMotorTorque(settings.hz); // float torque2 =
     * m_joint2.GetMotorTorque(settings.hz); // float force3 =
     * m_joint3.GetMotorForce(settings.hz); // g.drawString(5, m_textLine, //
     * "Motor Torque = %4.0f, %4.0f : Motor Force = %4.0f", torque1, // torque2,
     * force3); // m_textLine += 15; }
     */
    /**
     * Entry point
     */
    public static void main(String[] argv) {
        // new MotorsAndLimits().start();
        PApplet.main(new String[] { "testbed.tests.MotorsAndLimits" });

    }
}
