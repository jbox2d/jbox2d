package testbed.tests;

import java.util.Random;

import processing.core.PApplet;

import testbed.PTest;
import collision.AABB;
import collision.ShapeDescription;
import collision.ShapeType;

import common.Vec2;

import dynamics.BodyDescription;
import dynamics.World;

public class CompoundShapes extends PTest {

    public CompoundShapes() {
        super("CompoundShapes");
    }

    @Override
    public void setupWorld() {
        m_world = new World(new AABB(new Vec2(-100f, -100f), new Vec2(100f,
                200f)), new Vec2(0.0f, -10.0f), true);
        m_bomb = null;
        m_mouseJoint = null;
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
            ShapeDescription sd1 = new ShapeDescription(ShapeType.BOX_SHAPE);
            sd1.box.m_extents = new Vec2(0.25f, 0.5f);
            // sd1.localPosition=new Vec2(-0.5f, 0.0f);
            sd1.density = 1.0f;

            ShapeDescription sd2 = new ShapeDescription(ShapeType.BOX_SHAPE);
            sd2.box.m_extents = new Vec2(0.25f, 0.5f);
            // sd2.localPosition=new Vec2(0.5f, 0.0f);
            sd2.localPosition = new Vec2(0.0f, -0.5f);
            sd2.localRotation = (float) (0.5f * Math.PI);
            sd2.density = 1.0f;

            BodyDescription bd = new BodyDescription();
            bd.addShape(sd1);
            bd.addShape(sd2);

            Random r = new Random();

            for (int i = 0; i < 100; ++i) {
                float x = r.nextFloat() * 0.2f - 0.1f;
                bd.position = new Vec2(x, 1.05f + 1.5f * i);
                // bd.position=new Vec2(0.0f, 0.45f);
                bd.rotation = (float) (r.nextFloat() * Math.PI * 4 - Math.PI * 2);

                world.CreateBody(bd);
            }
        }
    }

    /**
     * Entry point
     */
    public static void main(String[] argv) {
        PApplet.main(new String[] { "testbed.tests.CompoundShapes" });
    }
}
