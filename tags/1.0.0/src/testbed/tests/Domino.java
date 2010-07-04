package testbed.tests;

import processing.core.PApplet;
import testbed.PTest;
import collision.ShapeDescription;
import collision.ShapeType;

import common.Vec2;

import dynamics.BodyDescription;
import dynamics.World;

public class Domino extends PTest {

    public Domino() {
        super("Domino");
    }

    public void makeDomino(float x, float y) {
    }

    @Override
    public void go(World world) {
        { // Floor
            ShapeDescription sd = new ShapeDescription(ShapeType.BOX_SHAPE);
            sd.box.m_extents = new Vec2(50.0f, 10.0f);

            BodyDescription bd = new BodyDescription();
            bd.position = new Vec2(0.0f, -10.0f);
            bd.addShape(sd);
            world.CreateBody(bd);
        }

        { // Platforms
            for (int i = 0; i < 4; i++) {
                ShapeDescription sd = new ShapeDescription(ShapeType.BOX_SHAPE);
                sd.box.m_extents = new Vec2(15.0f, 0.125f);

                BodyDescription bd = new BodyDescription();
                bd.position = new Vec2(0.0f, 5f + 5f * i);
                bd.addShape(sd);
                world.CreateBody(bd);
            }
        }

        {
            ShapeDescription sd = new ShapeDescription(ShapeType.BOX_SHAPE);
            sd.box.m_extents = new Vec2(0.125f, 2f);
            sd.density = 25.0f;

            BodyDescription bd = new BodyDescription();
            bd.addShape(sd);

            float friction = .5f;
            int numPerRow = 25;

            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < numPerRow; j++) {
                    sd.friction = friction;
                    bd.position = new Vec2(-14.75f + j
                            * (29.5f / (numPerRow - 1)), 7.3f + 5f * i);
                    if (i == 2 && j == 0) {
                        bd.rotation = -0.1f;
                        bd.position.x += .1f;
                    }
                    else if (i == 3 && j == numPerRow - 1) {
                        bd.rotation = .1f;
                        bd.position.x -= .1f;
                    }
                    else
                        bd.rotation = 0f;
                    world.CreateBody(bd);
                }
            }
        }
    }

    public static void main(String[] args) {
        PApplet.main(new String[] { "testbed.tests.Domino" });
    }
}
