package testbed.tests;

import processing.core.PApplet;
import testbed.PTest;
import collision.ShapeDescription;
import collision.ShapeType;

import common.Vec2;

import dynamics.Body;
import dynamics.BodyDescription;
import dynamics.World;
import dynamics.joints.DistanceJointDescription;
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
            ShapeDescription sd = new ShapeDescription(ShapeType.BOX_SHAPE);
            sd.box.m_extents = new Vec2(50.0f, 10.0f);

            BodyDescription bd = new BodyDescription();
            bd.position = new Vec2(0.0f, -10.0f);
            bd.addShape(sd);
            ground = m_world.CreateBody(bd);
        }

        ShapeDescription sd = new ShapeDescription(ShapeType.BOX_SHAPE);
        sd.box.m_extents = new Vec2(0.5f, 0.5f);
        sd.density = 5.0f;
        sd.friction = 0.2f;

        BodyDescription bd = new BodyDescription();
        bd.addShape(sd);

        bd.position = new Vec2(-5.0f, 5.0f);
        Body b1 = world.CreateBody(bd);

        bd.position = new Vec2(5.0f, 5.0f);
        Body b2 = world.CreateBody(bd);

        bd.position = new Vec2(5.0f, 15.0f);
        Body b3 = world.CreateBody(bd);

        bd.position = new Vec2(-5.0f, 15.0f);
        Body b4 = world.CreateBody(bd);

        DistanceJointDescription jd = new DistanceJointDescription();

        jd.body1 = ground;
        jd.body2 = b1;
        jd.anchorPoint1 = new Vec2(-10.0f, 0.0f);
        jd.anchorPoint2 = b1.m_position.add(new Vec2(-0.5f, -0.5f));
        m_joints[0] = world.CreateJoint(jd);

        jd.body1 = ground;
        jd.body2 = b2;
        jd.anchorPoint1 = new Vec2(10.0f, 0.0f);
        jd.anchorPoint2 = b2.m_position.add(new Vec2(0.5f, -0.5f));
        m_joints[1] = world.CreateJoint(jd);

        jd.body1 = ground;
        jd.body2 = b3;
        jd.anchorPoint1 = new Vec2(10.0f, 20.0f);
        jd.anchorPoint2 = b3.m_position.add(new Vec2(0.5f, 0.5f));
        m_joints[2] = world.CreateJoint(jd);

        jd.body1 = ground;
        jd.body2 = b4;
        jd.anchorPoint1 = new Vec2(-10.0f, 20.0f);
        jd.anchorPoint2 = b4.m_position.add(new Vec2(-0.5f, 0.5f));
        m_joints[3] = world.CreateJoint(jd);

        jd.body1 = b1;
        jd.body2 = b2;
        jd.anchorPoint1 = b1.m_position.add(new Vec2(0.5f, 0.0f));
        jd.anchorPoint2 = b2.m_position.add(new Vec2(-0.5f, 0.0f));
        m_joints[4] = world.CreateJoint(jd);

        jd.body1 = b2;
        jd.body2 = b3;
        jd.anchorPoint1 = b2.m_position.add(new Vec2(0.0f, 0.5f));
        jd.anchorPoint2 = b3.m_position.add(new Vec2(0.0f, -0.5f));
        m_joints[5] = world.CreateJoint(jd);

        jd.body1 = b3;
        jd.body2 = b4;
        jd.anchorPoint1 = b3.m_position.add(new Vec2(-0.5f, 0.0f));
        jd.anchorPoint2 = b4.m_position.add(new Vec2(0.5f, 0.0f));
        m_joints[6] = world.CreateJoint(jd);

        jd.body1 = b4;
        jd.body2 = b1;
        jd.anchorPoint1 = b4.m_position.add(new Vec2(0.0f, -0.5f));
        jd.anchorPoint2 = b1.m_position.add(new Vec2(0.0f, 0.5f));
        m_joints[7] = world.CreateJoint(jd);
    }

    /*
     * @Override protected void renderGUI(Graphics2D g) { g.drawString("Press
     * (b) to break constraints", 5, m_textLine); super.renderGUI(g); }
     * 
     * @Override protected void keyHit(char c) { if (c == 'b') { for (int i = 0;
     * i < 8; ++i) { if (m_joints[i] != null) {
     * m_world.DestroyJoint(m_joints[i]); m_joints[i] = null; break; } } }
     * super.keyHit(c); }
     */

    /**
     * Entry point
     */
    public static void main(String[] argv) {
        PApplet.main(new String[] { "testbed.tests.Web" });
    }
}
