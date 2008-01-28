package testbed.tests;

import processing.core.PApplet;
import processing.core.PImage;
import testbed.PTest;
import collision.*;

import common.Vec2;

import dynamics.Body;
import dynamics.BodyDef;
import dynamics.World;

//Assumes a .png file named "noise.png" has been added to the build path

public class SpriteBinding extends PTest {
    
    public Vec2[] localCoords;
    public Vec2[] texCoords;
    public PImage myImage;
    
    public Body body;

    public SpriteBinding() {
        super("SpriteBinding");
    }

    public void go(World world) {
        int numBoxes = 25; //number of boxes per row
        Body[] boxes = new Body[3*numBoxes];
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
            
            // Make them bouncy
            boxDef.restitution = 0.6f;

            // Define the dynamic body. We set its position,
            // add the box shape, and call the body factory.
            for (int i=0; i<numBoxes; ++i) {
                for (int j=0; j<3; ++j) {
                    BodyDef bodyDef = new BodyDef();
                    bodyDef.position.set(-numBoxes + i*2.5f + (j%2)*.5f, 4.0f + j*4f);
                    bodyDef.addShape(boxDef);
                    boxes[i + numBoxes*j] = world.createBody(bodyDef);
                }
            }
            
         }
        
        // Load the image from a file
        myImage = loadImage("noise.png");
        
        // Zero offset (center image on center of object)
        Vec2 localOffset = new Vec2(0f, 0f);
        
        // Scale image to fit box width
        float scale = 2.0f / myImage.width;
        
        // Zero rotation relative to box
        float rot = 0f;
        
        // Bind images to boxes
        for (int i=0; i<boxes.length; ++i) {
            bindImage(myImage, localOffset, rot, scale, boxes[i]);   
        }
        //textureMode(NORMALIZED);
    }
    

    /**
     * Entry point
     */
    public static void main(String[] argv) {
        PApplet.main(new String[] { "testbed.tests.SpriteBinding" });
    }
}