package testbed.tests;

import java.util.Random;

import processing.core.PApplet;
import testbed.PTest;
import collision.ShapeDescription;
import collision.ShapeType;

import common.Vec2;

import dynamics.Body;
import dynamics.BodyDescription;
import dynamics.World;

public class PolyShapes extends PTest {

    int bodyIndex;

    Body bodies[];

    ShapeDescription sds[];

    World world;

    public PolyShapes() {
        super("PolyShapes");

        bodies = new Body[256];
        sds = new ShapeDescription[4];
    }

    @Override
    public void go(World world) {
        this.world = world;
        // Ground body
        {
            ShapeDescription sd = new ShapeDescription(ShapeType.BOX_SHAPE);
            sd.box.m_extents = new Vec2(50.0f, 10.0f);
            sd.friction = 0.3f;

            BodyDescription bd = new BodyDescription();
            bd.position = new Vec2(0.0f, -10.0f);
            bd.addShape(sd);
            world.CreateBody(bd);
        }

        sds[0] = new ShapeDescription(ShapeType.POLY_SHAPE);
        // sds[0].poly.m_vertexCount = 3;
        sds[0].poly.m_vertices.add(new Vec2(-0.5f, 0.0f));
        sds[0].poly.m_vertices.add(new Vec2(0.5f, 0.0f));
        sds[0].poly.m_vertices.add(new Vec2(0.0f, 1.5f));
        sds[0].density = 1.0f;
        sds[0].friction = 0.3f;

        sds[1] = new ShapeDescription(ShapeType.POLY_SHAPE);
        // sds[1].poly.m_vertexCount = 3;
        sds[1].poly.m_vertices.add(new Vec2(-0.1f, 0.0f));
        sds[1].poly.m_vertices.add(new Vec2(0.1f, 0.0f));
        sds[1].poly.m_vertices.add(new Vec2(0.0f, 1.5f));
        sds[1].density = 1.0f;
        sds[1].friction = 0.3f;

        sds[2] = new ShapeDescription(ShapeType.POLY_SHAPE);
        // sds[2].poly.m_vertexCount = 8;
        float w = 1.0f;
        float b = w / (2.0f + (float) Math.sqrt(2.0f));
        float s = (float) Math.sqrt(2.0f) * b;
        sds[2].poly.m_vertices.add(new Vec2(0.5f * s, 0.0f));
        sds[2].poly.m_vertices.add(new Vec2(0.5f * w, b));
        sds[2].poly.m_vertices.add(new Vec2(0.5f * w, b + s));
        sds[2].poly.m_vertices.add(new Vec2(0.5f * s, w));
        sds[2].poly.m_vertices.add(new Vec2(-0.5f * s, w));
        sds[2].poly.m_vertices.add(new Vec2(-0.5f * w, b + s));
        sds[2].poly.m_vertices.add(new Vec2(-0.5f * w, b));
        sds[2].poly.m_vertices.add(new Vec2(-0.5f * s, 0.0f));
        sds[2].density = 1.0f;
        sds[2].friction = 0.3f;

        sds[3] = new ShapeDescription(ShapeType.POLY_SHAPE);
        // sds[3].poly.m_vertexCount = 4;
        sds[3].poly.m_vertices.add(new Vec2(-0.5f, 0.0f));
        sds[3].poly.m_vertices.add(new Vec2(0.5f, 0.0f));
        sds[3].poly.m_vertices.add(new Vec2(0.5f, 1.0f));
        sds[3].poly.m_vertices.add(new Vec2(-0.5f, 1.0f));
        sds[3].density = 1.0f;
        sds[3].friction = 0.3f;

        bodyIndex = 0;
        // memset(bodies, 0, sizeof(bodies));
    }

    void CreateBody(int index) {
        if (bodies[bodyIndex] != null) {
            world.DestroyBody(bodies[bodyIndex]);
            bodies[bodyIndex] = null;
        }

        Random r = new Random();

        BodyDescription bd = new BodyDescription();
        bd.addShape(sds[index]);
        float x = r.nextFloat() * 4 - 2;
        bd.position = new Vec2(x, 10.0f);
        bd.rotation = (float) (r.nextFloat() * Math.PI * 4 - Math.PI * 2);

        bodies[bodyIndex] = world.CreateBody(bd);
        bodyIndex = (bodyIndex + 1) % bodies.length;
    }

    @Override
    protected void checkKeys() {
        if (newKeyDown['1']) {
            CreateBody('1' - '1');
        }
        if (newKeyDown['2']) {
            CreateBody('2' - '1');
        }
        if (newKeyDown['3']) {
            CreateBody('3' - '1');
        }
        if (newKeyDown['4']) {
            CreateBody('4' - '1');
        }
    }

    /*
     * @Override protected void renderGUI(Graphics2D g) { super.renderGUI(g);
     * g.drawString("Press 1-4 to drop stuff", 5, m_textLine); // m_textLine +=
     * 15; }
     */

    public static void main(String[] args) {
        // new PolyShapes().start();
        PApplet.main(new String[] { "testbed.tests.PolyShapes" });
    }
}
