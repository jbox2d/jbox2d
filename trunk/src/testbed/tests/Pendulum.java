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
            ground = world.CreateBody(bd);
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
