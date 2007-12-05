package testbed.tests;

import processing.core.PApplet;
import testbed.PTest;
import collision.*;

import common.Vec2;

import dynamics.Body;
import dynamics.BodyDef;
import dynamics.World;
import dynamics.joints.*;

public class BugTest extends PTest {
    
    public Body body;

    public BugTest() {
        super("BugTest");
    }

    public void go(World world) {
        {        

            // Define the ground box shape.
            BoxDef groundBoxDef = new BoxDef();

            // The extents are the half-widths of the box.
            groundBoxDef.extents.set(50.0f, 10.0f);

            // Set the density of the ground box to zero. This will
            // make the ground body static (fixed).
            groundBoxDef.density = 0.0f;

            // Define the ground body.
            BodyDef groundBodyDef = new BodyDef();
            groundBodyDef.position.set(0.0f, -10.0f);

            // Part of a body's def is its list of shapes.
            groundBodyDef.addShape(groundBoxDef);

            // Call the body factory which allocates memory for the ground body
            // from a pool and creates the ground box shape (also from a pool).
            // The body is also added to the world.
            world.createBody(groundBodyDef);

            // Define another box shape for our dynamic body.
            BoxDef boxDef = new BoxDef();
            boxDef.extents.set(1.0f, 1.0f);

            // Set the box density to be non-zero, so it will be dynamic.
            boxDef.density = 1.0f;

            // Override the default friction.
            boxDef.friction = 0.3f;

            // Define the dynamic body. We set its position,
            // add the box shape, and call the body factory.
            BodyDef bodyDef = new BodyDef();
            bodyDef.position.set(0.0f, 4.0f);
            bodyDef.addShape(boxDef);
            body = world.createBody(bodyDef);
         }
    }
    
    public void frame() {
        println(body.m_position.y);
    }

    /**
     * Entry point
     */
    public static void main(String[] argv) {
        PApplet.main(new String[] { "testbed.tests.BugTest" });
    }
}