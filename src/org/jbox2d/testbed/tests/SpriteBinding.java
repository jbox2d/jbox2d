package org.jbox2d.testbed.tests;

import org.jbox2d.collision.*;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.testbed.AbstractExample;
import org.jbox2d.testbed.TestbedMain;

import processing.core.PImage;



/**
 * This example demonstrates how to bind a sprite to an image using Processing.
 * See ProcessingDebugDraw.drawImage() and AbstractExample.bindImage() for more
 * details on this - these should translate fairly easily to most OpenGL-style
 * rendering methods.
 * <BR><BR>
 * Assumes a .png file named "noise.png" has been added to the build path
 */
public class SpriteBinding extends AbstractExample {

	public Vec2[] localCoords;
    public Vec2[] texCoords;
    public PImage myImage;
    
    public Body body;
    
    public SpriteBinding(TestbedMain _parent) {
		super(_parent);
	}

    public String getName() {
    	return "Sprite Binding";
    }

    public void create() {
    	
        int numBoxes = 15; //number of boxes per row
        int numRows = 6;
        Body[] boxes = new Body[numRows*numBoxes];
        
        {        
            PolygonDef groundBoxDef = new PolygonDef();
            groundBoxDef.setAsBox(50.0f, 10.0f);
            groundBoxDef.density = 0.0f;
            BodyDef groundBodyDef = new BodyDef();
            groundBodyDef.position.set(0.0f, -10.0f);
            m_world.createStaticBody(groundBodyDef).createShape(groundBoxDef);;

            // Define another box shape for our dynamic body.
            PolygonDef boxDef = new PolygonDef();
            boxDef.setAsBox(1.0f, 1.0f);

            // Set the box density to be non-zero, so it will be dynamic.
            boxDef.density = 1.0f;

            // Override the default friction.
            boxDef.friction = 0.3f;
            
            // Make them bouncy
            boxDef.restitution = 0.3f;

            // Define the dynamic body. We set its position,
            // add the box shape, and call the body factory.
            for (int i=0; i<numBoxes; ++i) {
                for (int j=0; j<numRows; ++j) {
                    BodyDef bodyDef = new BodyDef();
                    bodyDef.position.set(-numBoxes - 3.0f + i*2.5f, 4.0f + j*5.0f);
                    boxes[i + numBoxes*j] = m_world.createDynamicBody(bodyDef);
                    boxes[i + numBoxes*j].createShape(boxDef);
                    boxes[i + numBoxes*j].setMassFromShapes();
                    boxes[i + numBoxes*j].setAngularVelocity(parent.random(-.5f,.5f));
                }
            }
            
         }
        
        // Load the image from a file
        myImage = parent.loadImage("noise.png");
        
        // Zero offset (center image on center of object)
        Vec2 localOffset = new Vec2(0.0f, 0f);
        
        // Scale image to fit box width
        // (actually, we slightly undershoot here so we can still see the drawn lines)
        float scale = 1.9f / myImage.width;
        
        // Zero rotation relative to box
        float rot = 0f;
        
        // Bind images to boxes
        for (int i=0; i<boxes.length; ++i) {
            bindImage(myImage, localOffset, rot, scale, boxes[i]);   
        }
        //textureMode(NORMALIZED);
    }
    
}