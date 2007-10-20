package testbed.tests;

import processing.core.PApplet;
import testbed.PTest;
import collision.BoxDef;

import common.Vec2;

import dynamics.BodyDef;
import dynamics.World;

public class Overhang extends PTest {

    public Overhang() {
        super("Overhang");
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
            float w = 4.0f;
            float h = 0.25f;
            sd.extents = new Vec2(w, h);
            sd.density = 1.0f;
            sd.friction = 0.3f;
            sd.restitution = 0.0f;

            BodyDef bd = new BodyDef();
            bd.addShape(sd);
            
            int numSlats = 8;
            float lastCMX = 0.0f;
            float eps = 0.14f;
            for (int i = 0; i < numSlats; ++i) {
                float newX = lastCMX + w - eps;
                lastCMX = (i*lastCMX + newX)/(i+1);
                bd.position = new Vec2(newX , .25f + 2*h * (numSlats - i - 1));
                m_world.createBody(bd);
            }

        }
    }

    public static void main(String[] args) {
        PApplet.main(new String[] { "testbed.tests.Overhang" });
    }
}