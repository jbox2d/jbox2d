package testbed.tests;

import processing.core.PApplet;
import testbed.PTest;
import collision.ShapeDescription;
import collision.ShapeType;

import common.Vec2;

import dynamics.BodyDescription;
import dynamics.World;

public class VaryingFriction extends PTest {

    public VaryingFriction() {
        super("VaryingFriction");
    }

    @Override
    public void go(World world) {
        {
            ShapeDescription sd = new ShapeDescription(ShapeType.BOX_SHAPE);
            sd.box.m_extents = new Vec2(50.0f, 10.0f);

            BodyDescription bd = new BodyDescription();
            bd.position = new Vec2(0.0f, -10.0f);
            bd.addShape(sd);
            world.CreateBody(bd);
        }

        {
            ShapeDescription sd = new ShapeDescription(ShapeType.BOX_SHAPE);
            sd.box.m_extents = new Vec2(6.5f, 0.125f);

            BodyDescription bd = new BodyDescription();
            bd.position = new Vec2(-2.0f, 11.0f);
            bd.rotation = -0.25f;
            bd.addShape(sd);
            world.CreateBody(bd);
        }

        {
            ShapeDescription sd = new ShapeDescription(ShapeType.BOX_SHAPE);
            sd.box.m_extents = new Vec2(0.125f, 0.5f);

            BodyDescription bd = new BodyDescription();
            bd.position = new Vec2(5.25f, 9.5f);
            bd.addShape(sd);
            world.CreateBody(bd);
        }

        {
            ShapeDescription sd = new ShapeDescription(ShapeType.BOX_SHAPE);
            sd.box.m_extents = new Vec2(6.5f, 0.125f);

            BodyDescription bd = new BodyDescription();
            bd.position = new Vec2(2.0f, 7.0f);
            bd.rotation = 0.25f;
            bd.addShape(sd);
            world.CreateBody(bd);
        }

        {
            ShapeDescription sd = new ShapeDescription(ShapeType.BOX_SHAPE);
            sd.box.m_extents = new Vec2(0.125f, 0.5f);

            BodyDescription bd = new BodyDescription();
            bd.position = new Vec2(-5.25f, 5.5f);
            bd.addShape(sd);
            world.CreateBody(bd);
        }

        {
            ShapeDescription sd = new ShapeDescription(ShapeType.BOX_SHAPE);
            sd.box.m_extents = new Vec2(6.5f, 0.125f);

            BodyDescription bd = new BodyDescription();
            bd.position = new Vec2(-2.0f, 3.0f);
            bd.rotation = -0.25f;
            bd.addShape(sd);
            world.CreateBody(bd);
        }

        {
            ShapeDescription sd = new ShapeDescription(ShapeType.BOX_SHAPE);
            sd.box.m_extents = new Vec2(0.25f, 0.25f);
            sd.density = 25.0f;

            BodyDescription bd = new BodyDescription();
            bd.addShape(sd);

            float friction[] = new float[] { 0.75f, 0.5f, 0.35f, 0.1f, 0.0f };

            for (int i = 0; i < 5; ++i) {
                sd.friction = friction[i];
                bd.position = new Vec2(-7.5f + 2.0f * i, 14.0f);
                world.CreateBody(bd);
            }
        }
    }

    public static void main(String[] args) {
        PApplet.main(new String[] { "testbed.tests.VaryingFriction" });
    }
}
