package testbed.tests;

import processing.core.PApplet;
import testbed.PTest;
import collision.ShapeDescription;
import collision.ShapeType;

import common.Vec2;

import dynamics.BodyDescription;
import dynamics.World;

public class VaryingRestitution extends PTest {

    public VaryingRestitution() {
        super("VaryingRestitution");
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
            ShapeDescription sd = new ShapeDescription(ShapeType.POLY_SHAPE);
            // sd.poly.m_vertexCount = 8;
            float w = 1.5f;
            float b = w / (2.0f + (float) Math.sqrt(2.0f));
            float s = (float) Math.sqrt(2.0f) * b;
            sd.poly.m_vertices.add(new Vec2(0.5f * s, 0.0f));
            sd.poly.m_vertices.add(new Vec2(0.5f * w, b));
            sd.poly.m_vertices.add(new Vec2(0.5f * w, b + s));
            sd.poly.m_vertices.add(new Vec2(0.5f * s, w));
            sd.poly.m_vertices.add(new Vec2(-0.5f * s, w));
            sd.poly.m_vertices.add(new Vec2(-0.5f * w, b + s));
            sd.poly.m_vertices.add(new Vec2(-0.5f * w, b));
            sd.poly.m_vertices.add(new Vec2(-0.5f * s, 0.0f));
            sd.density = 5.0f;

            BodyDescription bd = new BodyDescription();
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
