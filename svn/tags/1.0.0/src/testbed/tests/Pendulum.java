package testbed.tests;

import processing.core.PApplet;
import testbed.PTest;
import collision.ShapeDescription;
import collision.ShapeType;

import common.Vec2;

import dynamics.Body;
import dynamics.BodyDescription;
import dynamics.World;
import dynamics.joints.RevoluteDescription;

public class Pendulum extends PTest {

    public Pendulum() {
        super("Pendulum");
    }

    @Override
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
            sd.box.m_extents = new Vec2(0.375f, 0.125f);
            sd.density = 20.0f;
            sd.friction = 0.2f;

            BodyDescription bd = new BodyDescription();
            bd.addShape(sd);

            RevoluteDescription jd = new RevoluteDescription();

            final float y = 25.0f;
            Body prevBody = ground;
            for (int i = 0; i < 30; ++i) {
                bd.position = new Vec2(0.5f + i, y);
                Body body = world.CreateBody(bd);

                jd.anchorPoint = new Vec2(i, y);
                jd.body1 = prevBody;
                jd.body2 = body;
                world.CreateJoint(jd);

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
