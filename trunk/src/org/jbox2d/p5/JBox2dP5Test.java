package org.jbox2d.p5;

import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.joints.PrismaticJoint;

import processing.core.PApplet;

public class JBox2dP5Test extends PApplet {
	
	public Physics physics;
	private Body body, body2;
	
	private boolean set;
	private int count = 0;
	
	public void setup() {
		size(940,480,P3D);
		frameRate(60);
		set = false;
	}
	
	public void draw() {
		background(0);
		
		// Hack to get around threading bug re: size initialization
		++count;
		if ( (!set && count > 100) ) {
			initScene();
		}
		
		if (mousePressed) {
			//Create a body
//			float x0 = mouseX;
//			float y0 = mouseY;
//			Body randomBod = physics.createCircle(x0, y0, random(5.0f,25f));
//			Vec2 vel = new Vec2(random(-100.0f,100.0f),random(-100.0f,100.0f));
//			randomBod.setLinearVelocity(vel);
//			physics.createPolygon(mouseX+10.0f, mouseY,
//								  mouseX,	    mouseY-10.0f,
//								  mouseX-10.0f, mouseY,
//								  mouseX,	    mouseY+10.0f);
			float[] xyInterleaved = {mouseX+10.0f, mouseY,
					  mouseX,	    mouseY-10.0f,
					  mouseX-10.0f, mouseY,
					  mouseX,	    mouseY+10.0f};
			physics.createPolygon(xyInterleaved);
		}
		
		if (keyPressed) {

			//physics.setCustomRenderingMethod(physics, "defaultDraw");
			
			//Reset everything
			physics.destroy();
			body = body2 = null;
			initScene();
		}
	}
	
	public void initScene() {
		physics = new Physics(this, 940, 480);
		physics.setDensity(1.0f);
		Body b1 = null;
		Body b2 = null;
		
		// Make a chain of bodies
		for (int i=0; i<10; ++i) {
			body = body2; //bookkeeping, for neighbor connection
			
			body2 = physics.createRect(100+25*i, 10, 120+25*i, 30);
			
			// Add a hanging thingy to each body, connect it
			// with a prismatic joint (like a piston)
			Body body3 = physics.createCircle(110+25*i,35,5.0f);
			PrismaticJoint pj = physics.createPrismaticJoint(body2, body3, 0.0f, 1.0f);
			pj.m_enableLimit = true;
			pj.setLimits(-3.0f, 1.0f);
			
			if (i==0) b1 = body2; // for pulley joint later
			if (i==9) b2 = body2;
			
			if (body == null) {
				// No previous body, so continue without adding joint
				body = body2;
				continue;
			}
			// Connect the neighbors
			physics.createRevoluteJoint(body, body2, 100+25*i, 20);
			
		}
		
		// Make a pulley joint
		float groundAnchorAx = 100;
		float groundAnchorAy = 150;
		float groundAnchorBx = 860;
		float groundAnchorBy = 150;
		float anchorAx = 100;
		float anchorAy = 20;
		float anchorBx = 345;
		float anchorBy = 20;
		float ratio = 1.0f;
		
		physics.createPulleyJoint(b1, b2, 
				groundAnchorAx, groundAnchorAy, 
				groundAnchorBx, groundAnchorBy, 
				anchorAx, anchorAy, 
				anchorBx, anchorBy, ratio);
		set = true;
	}
    
	/** Start PApplet as a Java program (can also be run as an applet). */
    static public void main(String args[]) {
        PApplet.main(new String[] { "org.jbox2d.p5.JBox2dP5Test" });
    }
}
