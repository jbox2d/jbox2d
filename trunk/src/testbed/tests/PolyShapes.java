package testbed.tests;

import java.util.Random;

import processing.core.PApplet;
import testbed.PTest;
import collision.BoxDef;
import collision.CircleDef;
import collision.PolyDef;
import collision.ShapeDef;

import common.Vec2;

import dynamics.Body;
import dynamics.BodyDef;
import dynamics.World;

public class PolyShapes extends PTest {

    int bodyIndex;

    Body bodies[];

    ShapeDef sds[];

    World world;

    public PolyShapes() {
        super("PolyShapes");

        bodies = new Body[256];
        sds = new ShapeDef[4];
    }

    @Override
    public void go(World world) {
        this.world = world;
        // Ground body
        {
            BoxDef sd = new BoxDef();
            sd.extents = new Vec2(50.0f, 10.0f);
            sd.friction = 0.3f;

            BodyDef bd = new BodyDef();
            bd.position = new Vec2(0.0f, -10.0f);
            bd.addShape(sd);
            world.createBody(bd);
        }

        sds[0] = new PolyDef();
        PolyDef poly = (PolyDef) sds[0];
        // sds[0].poly.m_vertexCount = 3;
        poly.vertices.add(new Vec2(-0.5f, 0.0f));
        poly.vertices.add(new Vec2(0.5f, 0.0f));
        poly.vertices.add(new Vec2(0.0f, 1.5f));
        poly.density = 1.0f;
        poly.friction = 0.3f;

        sds[1] = new PolyDef();
        PolyDef poly2 = (PolyDef) sds[1];
        // sds[1].poly.m_vertexCount = 3;
        poly2.vertices.add(new Vec2(-0.1f, 0.0f));
        poly2.vertices.add(new Vec2(0.1f, 0.0f));
        poly2.vertices.add(new Vec2(0.0f, 1.5f));
        poly2.density = 1.0f;
        poly2.friction = 0.3f;

        sds[2] = new CircleDef();
        CircleDef circ = (CircleDef) sds[2];
        // sds[2].poly.m_vertexCount = 8;
        circ.radius = .6f;
        circ.density = 1.0f;
        circ.friction = 0.3f;

        sds[3] = new PolyDef();
        PolyDef poly3 = (PolyDef) sds[3];
        // sds[3].poly.m_vertexCount = 4;
        poly3.vertices.add(new Vec2(-0.5f, 0.0f));
        poly3.vertices.add(new Vec2(0.5f, 0.0f));
        poly3.vertices.add(new Vec2(0.5f, 1.0f));
        poly3.vertices.add(new Vec2(-0.5f, 1.0f));
        poly3.density = 1.0f;
        poly3.friction = 0.3f;

        bodyIndex = 0;
        // memset(bodies, 0, sizeof(bodies));
    }

    void CreateBody(int index) {
        if (bodies[bodyIndex] != null) {
            world.destroyBody(bodies[bodyIndex]);
            bodies[bodyIndex] = null;
        }

        Random r = new Random();

        BodyDef bd = new BodyDef();
        bd.addShape(sds[index]);
        float x = r.nextFloat() * 4 - 2;
        bd.position = new Vec2(x, 10.0f);
        bd.rotation = (float) (r.nextFloat() * Math.PI * 4 - Math.PI * 2);

        bodies[bodyIndex] = world.createBody(bd);
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

    public static void main(String[] args) {
        // new PolyShapes().start();
        PApplet.main(new String[] { "testbed.tests.PolyShapes" });
    }
}
