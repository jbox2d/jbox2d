package testbed.tests;

import processing.core.PApplet;
import testbed.PTest;
import collision.BoxDef;

import common.Vec2;

import dynamics.BodyDef;
import dynamics.World;

public class Pyramid extends PTest {

    public Pyramid() {
        super("Pyramid");
    }

    @Override
    public void go(World world) {
        {
            BoxDef sd = new BoxDef();
            sd.extents = new Vec2(50.0f, 10.0f);

            BodyDef bd = new BodyDef();
            bd.position = new Vec2(0.0f, -10.0f);
            bd.addShape(sd);
            m_world.createBody(bd);
        }

        {
            BoxDef sd = new BoxDef();
            float a = 0.5f;
            sd.extents = new Vec2(a, a);
            sd.density = 2.0f;

            BodyDef bd = new BodyDef();
            bd.addShape(sd);

            Vec2 x = new Vec2(-10.0f, 0.75f);
            Vec2 y;
            Vec2 deltaX = new Vec2(0.5625f, 2.0f);
            Vec2 deltaY = new Vec2(1.125f, 0.0f);

            for (int i = 0; i < 25; ++i) {
                y = x.clone();

                for (int j = i; j < 25; ++j) {
                    bd.position = y;
                    m_world.createBody(bd);

                    y.addLocal(deltaY);
                }

                x.addLocal(deltaX);
            }
        }
    }

    /**
     * Entry point
     */
    public static void main(String[] argv) {
        PApplet.main(new String[] { "testbed.tests.Pyramid" });
    }
}
