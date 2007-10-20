package testbed.tests;

import processing.core.PApplet;
import testbed.PTest;
import collision.BoxDef;

import common.Vec2;

import dynamics.BodyDef;
import dynamics.World;

public class VerticalStack extends PTest {

    public VerticalStack() {
        super("VerticalStack");
    }

    @Override
    public void go(World world) {
        {
            BoxDef sd = new BoxDef();
            sd.extents = new Vec2(50.0f, 10.0f);

            BodyDef bd = new BodyDef();
            bd.position = new Vec2(0.0f, -10.0f);
            bd.addShape(sd);
            world.CreateBody(bd);
        }

        {
            BoxDef sd = new BoxDef();
            float a = 0.75f;
            sd.extents = new Vec2(a, a);
            sd.density = 1.0f;
            sd.friction = 0.3f;

            BodyDef bd = new BodyDef();
            bd.addShape(sd);

            for (int i = 0; i < 14; ++i) {
                // float32 x = b2Random(-0.1f, 0.1f);
                // float32 x = i % 2 == 0 ? -0.025f : 0.025f;
                // bd.position= new Vec2(x, 0.752f + 1.54f * i);

                bd.position = new Vec2(0.0f, 0.752f + 1.54f * i);

                m_world.CreateBody(bd);
            }
        }
    }

    public static void main(String[] args) {
        PApplet.main(new String[] { "testbed.tests.VerticalStack" });
    }
}
