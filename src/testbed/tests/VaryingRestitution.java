package testbed.tests;

import processing.core.PApplet;
import testbed.PTest;
import collision.BoxDef;
import collision.CircleDef;

import common.Vec2;

import dynamics.BodyDef;
import dynamics.World;

public class VaryingRestitution extends PTest {

    public VaryingRestitution() {
        super("VaryingRestitution");
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
            CircleDef sd = new CircleDef();;
            // sd.poly.m_vertexCount = 8;
            sd.radius = .6f;
            sd.density = 5.0f;

            BodyDef bd = new BodyDef();
            bd.addShape(sd);

            float restitution[] = new float[] { 0.0f, 0.1f, 0.3f, 0.5f, 0.75f,
                    0.9f, 1.0f };

            for (int i = 0; i < restitution.length; ++i) {
                sd.restitution = restitution[i];
                bd.position = new Vec2(-10.0f + 3.0f * i, 10.0f);
                m_world.CreateBody(bd);
            }
        }
    }

    public static void main(String[] args) {
        PApplet.main(new String[] { "testbed.tests.VaryingRestitution" });
    }
}
