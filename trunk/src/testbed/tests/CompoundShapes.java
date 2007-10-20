package testbed.tests;

import java.util.Random;

import processing.core.PApplet;

import testbed.PTest;
import collision.AABB;
import collision.BoxDef;

import common.Vec2;

import dynamics.BodyDef;
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
            BoxDef sd = new BoxDef();
            sd.extents = new Vec2(50.0f, 10.0f);

            BodyDef bd = new BodyDef();
            bd.position = new Vec2(0.0f, -10.0f);
            bd.addShape(sd);
            world.createBody(bd);
        }

        {
            BoxDef sd1 = new BoxDef();
            sd1.extents = new Vec2(0.25f, 0.5f);
            // sd1.localPosition=new Vec2(-0.5f, 0.0f);
            sd1.density = 1.0f;

            BoxDef sd2 = new BoxDef();
            sd2.extents = new Vec2(0.25f, 0.5f);
            // sd2.localPosition=new Vec2(0.5f, 0.0f);
            sd2.localPosition = new Vec2(0.0f, -0.5f);
            sd2.localRotation = (float) (0.5f * Math.PI);
            sd2.density = 1.0f;

            BodyDef bd = new BodyDef();
            bd.addShape(sd1);
            bd.addShape(sd2);

            Random r = new Random();

            for (int i = 0; i < 100; ++i) {
                float x = r.nextFloat() * 0.2f - 0.1f;
                bd.position = new Vec2(x, 1.05f + 1.5f * i);
                // bd.position=new Vec2(0.0f, 0.45f);
                bd.rotation = (float) (r.nextFloat() * Math.PI * 4 - Math.PI * 2);

                world.createBody(bd);
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
