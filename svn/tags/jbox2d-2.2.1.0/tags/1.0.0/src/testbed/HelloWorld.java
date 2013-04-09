package testbed;

// This is a simple example of building and running a simulation
// using Box2D. Here we create a large ground box and a small dynamic
// box.

import common.*;
import collision.*;
import dynamics.*;

public class HelloWorld {

    static public void main(String args[]) {

        // Define the size of the world. Simulation will still work
        // if bodies reach the end of the world, but it will be slower.
        AABB worldAABB = new AABB();
        worldAABB.minVertex.set(-100.0f, -100.0f);
        worldAABB.maxVertex.set(100.0f, 100.0f);

        // Define the gravity vector.
        Vec2 gravity = new Vec2(0.0f, -10.0f);

        // Do we want to let bodies sleep?
        boolean doSleep = true;

        // Construct a world object, which will hold and simulate the rigid
        // bodies.
        World world = new World(worldAABB, gravity, doSleep);

        // Define the ground box shape.
        ShapeDescription groundBoxDef = new ShapeDescription(
                ShapeType.BOX_SHAPE);

        // The extents are the half-widths of the box.
        groundBoxDef.box.m_extents.set(50.0f, 10.0f);

        // Set the density of the ground box to zero. This will
        // make the ground body static (fixed).
        groundBoxDef.density = 0.0f;

        // Define the ground body.
        BodyDescription groundBodyDef = new BodyDescription();
        groundBodyDef.position.set(0.0f, -10.0f);

        // Part of a body's def is its list of shapes.
        groundBodyDef.addShape(groundBoxDef);

        // Call the body factory which allocates memory for the ground body
        // from a pool and creates the ground box shape (also from a pool).
        // The body is also added to the world.
        world.CreateBody(groundBodyDef);

        // Define another box shape for our dynamic body.
        ShapeDescription boxDef = new ShapeDescription(ShapeType.BOX_SHAPE);
        boxDef.box.m_extents.set(1.0f, 1.0f);

        // Set the box density to be non-zero, so it will be dynamic.
        boxDef.density = 1.0f;

        // Override the default friction.
        boxDef.friction = 0.3f;

        // Define the dynamic body. We set its position,
        // add the box shape, and call the body factory.
        BodyDescription bodyDef = new BodyDescription();
        bodyDef.position.set(0.0f, 4.0f);
        bodyDef.addShape(boxDef);
        Body body = world.CreateBody(bodyDef);

        // Prepare for simulation. Typically we use a time step of 1/60 of a
        // second (60Hz) and 10 iterations. This provides a high quality
        // simulation
        // in most game scenarios.
        float timeStep = 1.0f / 60.0f;
        int iterations = 10;

        // This is our little game loop.
        for (int i = 0; i < 60; ++i) {
            // Instruct the world to perform a single step of simulation. It is
            // generally best to keep the time step and iterations fixed.
            world.Step(timeStep, iterations);

            // Now print the position and rotation of the body.
            Vec2 position = body.GetOriginPosition();
            float rotation = body.GetRotation();

            System.out
                    .println(position.x + ", " + position.y + "; " + rotation);
        }

    }
}
