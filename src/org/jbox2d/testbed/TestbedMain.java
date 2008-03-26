/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/ 
 * Box2D homepage: http://www.box2d.org
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
import org.jbox2d.testbed.tests.Chain;
import org.jbox2d.testbed.tests.DistanceTest;

import processing.core.PApplet;
import processing.core.PImage;

public class TestbedMain extends PApplet {
	/** The examples */
	static private ArrayList<AbstractExample> tests = new ArrayList<AbstractExample>(0);
	/** Currently running example */
	static private AbstractExample currentTest = null;
	/** 
	 * Index of current example in tests array.
	 * Assumes that the array structure does not change,
	 * though it's safe to add things on to the end. 
	 */
	static private int currentTestIndex = 0;
	
	// Little bit of input stuff
	// TODO ewjordan: refactor into an input handler class
	public boolean shiftKey = false;
    Vec2 mouseWorld = new Vec2();
    boolean pmousePressed = false;
    
    // Processing handles fps pinning, but we
    // report fps on our own just to be sure.
    final static float targetFPS = 60.0f;
    final int fpsAverageCount = 100; 
    long[] nanos;
    long nanoStart; //
    double fps = targetFPS;
    
    long frameCount = 0;
    
    public DebugDraw g;

    /** Constructor - real initialization happens in setup() function */
    public TestbedMain() {
    	super();
    }
    
    public void setup() {
    	/* On newer machines especially, the default JAVA2D renderer
    	 * is slow as hell and tends to drop frames.  I have no idea
    	 * why, but for now let's use P3D and live without the smoothing...
    	 */
    	size(640,480,P3D);
    	frameRate(targetFPS);
    	g = new ProcessingDebugDraw(this);
    	//smooth();
    	
    	/* Register the examples */
    	registerExample(new Chain(this));
    	registerExample(new Bridge(this));
    	registerExample(new CCDTest(this));
    	registerExample(new DistanceTest(this));

    	//Set up the mouse wheel listener to control zooming
    	addMouseWheelListener(new MouseWheelListener() {
            public void mouseWheelMoved(MouseWheelEvent e) {
            	if (currentTest != null) {
            		ProcessingDebugDraw d = (ProcessingDebugDraw)(currentTest.m_debugDraw);
            		int notches = e.getWheelRotation();
                	Vec2 oldCenter = d.screenToWorld(width / 2.0f, height / 2.0f);
                	//Change the zoom and clamp it to reasonable values 
                	if (notches < 0) {
                		d.scaleFactor = min(300f, d.scaleFactor * 1.05f);
                	}
                	else if (notches > 0) {
                		d.scaleFactor = max(.02f, d.scaleFactor / 1.05f);
                	}
                	Vec2 newCenter = d.screenToWorld(width / 2.0f, height / 2.0f);
                	d.transX -= (oldCenter.x - newCenter.x) * d.scaleFactor;
                	d.transY -= (oldCenter.y - newCenter.y) * d.scaleFactor;
                	currentTest.cachedCamScale = d.scaleFactor;
            	}
            }
        });
    	
    	
    	/* Set up the timers for FPS reporting */
    	nanos = new long[fpsAverageCount];
    	long nanosPerFrameGuess = (long)(1000000000.0 / targetFPS);
    	nanos[fpsAverageCount-1] = System.nanoTime();
    	for (int i=fpsAverageCount-2; i>=0; --i) {
    		nanos[i] = nanos[i+1] - nanosPerFrameGuess;
    	}
    	nanoStart = System.nanoTime();
    	
    }
    
    public void draw() {
    	background(0);
    	Vec2.creationCount = 0;
 		
    	/* Make sure we've got a valid test to run and reset it if needed */
    	if (currentTest == null) {
    		currentTestIndex = 0;
    		currentTest = tests.get(currentTestIndex);
    		nanoStart = System.nanoTime();
    		frameCount = 0;
    	}
    	if (currentTest.needsReset) {
    		//System.out.println("Resetting "+currentTest.getName());
    		currentTest.initialize();
    		nanoStart = System.nanoTime();
    		frameCount = 0;
    	}
    	currentTest.m_textLine = 15;
    	g.drawString(5, currentTest.m_textLine, currentTest.getName(),AbstractExample.white);
    	currentTest.m_textLine += 30;
    	
    	/* Take our time step (drawing is done here, too) */
    	currentTest.step();
    	
    	/* If the user wants to move the canvas, do it */
    	handleCanvasDrag();

    	/* Store whether the mouse was pressed this step */
        pmousePressed = mousePressed;
        
        /* ==== Vec2 creation and FPS reporting ==== */
        g.drawString(5, currentTest.m_textLine, "Vec2 creations/frame: "+Vec2.creationCount, AbstractExample.white);
        currentTest.m_textLine += 15;
        
        for (int i=0; i<fpsAverageCount-1; ++i) {
        	nanos[i] = nanos[i+1];
        }
        nanos[fpsAverageCount-1] = System.nanoTime();
        float averagedFPS = (float) ( (fpsAverageCount-1) * 1000000000.0 / (nanos[fpsAverageCount-1]-nanos[0]));
        ++frameCount;
        float totalFPS = (float) (frameCount * 1000000000 / (1.0*(System.nanoTime()-nanoStart)));        
        g.drawString(5, currentTest.m_textLine, "Average FPS ("+fpsAverageCount+" frames): "+averagedFPS, AbstractExample.white);
        currentTest.m_textLine += 15;
        g.drawString(5, currentTest.m_textLine, "Average FPS (entire test): "+totalFPS, AbstractExample.white);
        currentTest.m_textLine += 15;
    }
    
    
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
                Vec2 v = d.screenToWorld(width*.5f,height*.5f);
                currentTest.cachedCamX = v.x;
                currentTest.cachedCamY = v.y;
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
    	if (keyCode == PApplet.RIGHT) {
    		++currentTestIndex;
    		if (currentTestIndex >= tests.size()) currentTestIndex = 0;
    		System.out.println(currentTestIndex);
    		currentTest = tests.get(currentTestIndex);
    		currentTest.needsReset = true;
    		return;
    	} else if (keyCode == PApplet.LEFT) {
    		--currentTestIndex;
    		if (currentTestIndex < 0) currentTestIndex = tests.size()-1;
    		System.out.println(currentTestIndex);
    		currentTest = tests.get(currentTestIndex);
    		currentTest.needsReset = true;
    		return;
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
