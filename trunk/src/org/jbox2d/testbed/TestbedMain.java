/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/ 
 * Box2D homepage: http://www.gphysics.com
 * 
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
package org.jbox2d.testbed;

import java.awt.event.MouseWheelEvent;
import java.awt.event.MouseWheelListener;
import java.util.Random;
import java.util.ArrayList;

import org.jbox2d.collision.AABB;
import org.jbox2d.collision.BroadPhase;
import org.jbox2d.collision.CircleShape;
import org.jbox2d.collision.Manifold;
import org.jbox2d.collision.Pair;
import org.jbox2d.collision.PolygonShape;
import org.jbox2d.collision.Proxy;
import org.jbox2d.collision.Shape;
import org.jbox2d.collision.ShapeType;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.DebugDraw;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.contacts.Contact;
import org.jbox2d.dynamics.joints.Joint;
import org.jbox2d.dynamics.joints.JointType;
import org.jbox2d.dynamics.joints.MouseJoint;
import org.jbox2d.dynamics.joints.MouseJointDef;
import org.jbox2d.dynamics.joints.PulleyJoint;
import org.jbox2d.testbed.tests.Bridge;
import org.jbox2d.testbed.tests.CCDTest;

import processing.core.PApplet;
import processing.core.PImage;

public class TestbedMain extends PApplet {
	static private ArrayList<AbstractExample> tests = new ArrayList<AbstractExample>(0);
	static private AbstractExample currentTest = null;
	static private int currentTestIndex = 0;
	
	public boolean shiftKey = false;
	
    public DebugDraw g;

    public TestbedMain() {
    	super();
    	g = new ProcessingDebugDraw(this);
    	
    	//Set up the mouse wheel listener to control zoom
    	addMouseWheelListener(new MouseWheelListener() {
            public void mouseWheelMoved(MouseWheelEvent e) {
            	if (currentTest != null) {
            		ProcessingDebugDraw d = (ProcessingDebugDraw)(currentTest.m_debugDraw);
            		int notches = e.getWheelRotation();
                	Vec2 oldCenter = d.screenToWorld(width / 2.0f, height / 2.0f);
                	if (notches < 0) {
                		d.scaleFactor = min(300f, d.scaleFactor * 1.05f);
                	}
                	else if (notches > 0) {
                		d.scaleFactor = max(.02f, d.scaleFactor / 1.05f);
                	}
                	Vec2 newCenter = d.screenToWorld(width / 2.0f, height / 2.0f);
                	d.transX -= (oldCenter.x - newCenter.x) * d.scaleFactor;
                	d.transY -= (oldCenter.y - newCenter.y) * d.scaleFactor;
            	}
            }
        });
    	
    }
    
    public void setup() {
    	size(640,480);
    	smooth();
    	AbstractExample ex0 = new CCDTest(this);
    	AbstractExample ex1 = new Bridge(this);
    	registerExample(ex0);
    	registerExample(ex1);
    }
    
    public void draw() {
    	background(0);

    	if (currentTest == null) {
    		currentTestIndex = 0;
    		currentTest = tests.get(currentTestIndex);
    	}
    	if (currentTest.needsReset) {
    		System.out.println("reset");
    		currentTest.initialize();
    	}
    	currentTest.step();
    	
    	handleCanvasDrag();

        pmousePressed = mousePressed;
    }
    
    Vec2 mouseWorld = new Vec2();
    boolean pmousePressed = false;
    
    public void handleCanvasDrag() {
    	//Handle mouse dragging stuff
        //Left mouse attaches mouse joint to object.
        //Right mouse drags canvas.
    	ProcessingDebugDraw d = (ProcessingDebugDraw)(currentTest.m_debugDraw);
		
        mouseWorld = d.screenToWorld(mouseX, mouseY);
        if (mouseButton == RIGHT) {
            if (mousePressed) {
                d.transX += mouseX - pmouseX;
                d.transY -= mouseY - pmouseY;
            }
        }
    
    }
    
    public void mousePressed() {
    	if (currentTest == null) return;
    	currentTest.mouseDown(new Vec2(mouseX,mouseY));
    }
    
    public void mouseReleased() {
    	if (currentTest == null) return;
    	currentTest.mouseUp();
    }
    
    public void mouseMoved() {
    	if (currentTest == null) return;
    	currentTest.mouseMove(new Vec2(mouseX,mouseY));
    }
    
    public void mouseDragged() {
    	mouseMoved();
    }
    
    public void keyPressed() {
    	if (keyCode == PApplet.SHIFT) {
            shiftKey = true;
        }
    	if (currentTest == null) return;
    	if (key == 'r') currentTest.needsReset = true;
    	if (key == ' ') currentTest.launchBomb();
    	if (key == 'p') {
    		currentTest.settings.singleStep = true;
    		currentTest.settings.pause = true;
    	}
    	if (key == 'c') currentTest.settings.drawContactPoints = !currentTest.settings.drawContactPoints;
    	if (key == 'b') currentTest.settings.drawAABBs = !currentTest.settings.drawAABBs;
    		
    	currentTest.keyPressed(key);
    }
    
    public void keyReleased() {
    	if (keyCode == PApplet.SHIFT) {
            shiftKey = false;
        }
    	if (currentTest == null) return;
    	currentTest.keyReleased(key);
    }
    
    static public void registerExample(AbstractExample test) {
    	tests.add(test);
    }

    static public void main(String args[]) {
        PApplet.main(new String[] { "org.jbox2d.testbed.TestbedMain" });
    }

}
