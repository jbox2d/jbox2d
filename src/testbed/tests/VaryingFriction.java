package testbed.tests;

import processing.core.PApplet;
import testbed.PTest;
import collision.BoxDef;

import common.Vec2;

import dynamics.BodyDef;
import dynamics.World;

public class VaryingFriction extends PTest {

    public VaryingFriction() {
        super("VaryingFriction");
    }

    @Override
    public void go(World world) {
        {
            BoxDef sd = new BoxDef();
            sd.extents = new Vec2(50.0f, 10.0f);

            BodyDef bd = new BodyDef();
            bd.position = new Vec2(0.0f, -10.0f);
            bd.addShape(sd);
            world.createBody(bd);
        }

        {
            BoxDef sd = new BoxDef();
            sd.extents = new Vec2(6.5f, 0.125f);

            BodyDef bd = new BodyDef();
            bd.position = new Vec2(-2.0f, 11.0f);
            bd.rotation = -0.25f;
            bd.addShape(sd);
            world.createBody(bd);
        }

        {
            BoxDef sd = new BoxDef();
            sd.extents = new Vec2(0.125f, 0.5f);

            BodyDef bd = new BodyDef();
            bd.position = new Vec2(5.25f, 9.5f);
            bd.addShape(sd);
            world.createBody(bd);
        }

        {
            BoxDef sd = new BoxDef();
            sd.extents = new Vec2(6.5f, 0.125f);

            BodyDef bd = new BodyDef();
            bd.position = new Vec2(2.0f, 7.0f);
            bd.rotation = 0.25f;
            bd.addShape(sd);
            world.createBody(bd);
        }

        {
            BoxDef sd = new BoxDef();
            sd.extents = new Vec2(0.125f, 0.5f);

            BodyDef bd = new BodyDef();
            bd.position = new Vec2(-5.25f, 5.5f);
            bd.addShape(sd);
            world.createBody(bd);
        }

        {
            BoxDef sd = new BoxDef();
            sd.extents = new Vec2(6.5f, 0.125f);

            BodyDef bd = new BodyDef();
            bd.position = new Vec2(-2.0f, 3.0f);
            bd.rotation = -0.25f;
            bd.addShape(sd);
            world.createBody(bd);
        }

        {
            BoxDef sd = new BoxDef();
            sd.extents = new Vec2(0.25f, 0.25f);
            sd.density = 25.0f;

            BodyDef bd = new BodyDef();
            bd.addShape(sd);

            float friction[] = new float[] { 0.75f, 0.5f, 0.35f, 0.1f, 0.0f };

            for (int i = 0; i < 5; ++i) {
                sd.friction = friction[i];
                bd.position = new Vec2(-7.5f + 2.0f * i, 14.0f);
                world.createBody(bd);
            }
        }
    }

    public static void main(String[] args) {
        PApplet.main(new String[] { "testbed.tests.VaryingFriction" });
    }
}
