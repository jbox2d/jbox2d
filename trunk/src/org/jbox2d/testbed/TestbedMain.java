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
import java.util.ArrayList;

import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.DebugDraw;
import org.jbox2d.testbed.tests.*;

import processing.core.PApplet;

/**
 * TestbedMain is the holder PApplet for the entire testbed.
 * It has first stab at input events, and delegates them out
 * to the appropriate handlers if necessary.  It handles the
 * list of tests to run, and starts them as needed.
 * <BR><BR>For applet safety, no variables are static, because
 * in the browser a page reload does <em>not</em> reset static
 * variables (even though the applet may get restarted otherwise).
 * From a design perspective this is not ideal, but hey - this
 * is just a demo, anyways, let's not get too bent out of
 * shape over it.
 * <BR><BR>In the future, other classes should be checked to
 * make sure this odd static variable behavior is not causing
 * memory leaks in the browser.
 * 
 * @author ewjordan
 *
 */
public class TestbedMain extends PApplet {
	/** I let Eclipse generate this to shut it up about its warnings. */
	private static final long serialVersionUID = 1712524774634907635L;
	/** The list of registered examples */
	protected ArrayList<AbstractExample> tests = new ArrayList<AbstractExample>(0);
	/** Currently running example */
	protected AbstractExample currentTest = null;
	/** 
	 * Index of current example in tests array.
	 * Assumes that the array structure does not change,
	 * though it's safe to add things on to the end. 
	 */
	protected int currentTestIndex = 0;
	/** Is the options window open? */
	protected boolean handleOptions = false;
	
	// Little bit of input stuff
	// TODO ewjordan: refactor into an input handler class
	/** Is the shift key held? */
	public boolean shiftKey = false;
    
	/** Was the mouse down last frame? */
    boolean pmousePressed = false;
    
    /** Our options handler - displays GUI and sets TestSettings for currentTest. */
    public TestbedOptions options;
    
    // Processing handles fps pinning, but we
    // report fps on our own just to be sure.
    
    /** FPS that we want to achieve */
    final static float targetFPS = 60.0f;
    /** Number of frames to average over when computing real FPS */
    final int fpsAverageCount = 100; 
    /** Array of timings */
    long[] nanos;
    /** When we started the nanotimer */
    long nanoStart; //
    
    /** Number of frames since we started this example. */
    long frameCount = 0;
    
    /** Drawing handler to use. */
    public DebugDraw g;

    /** Constructor - real initialization happens in setup() function */
    public TestbedMain() {
    	super();
    }
    
    /**
     * Called once upon program initialization (by Processing).
     * Here we set up graphics, set the framerate, register
     * all the testbed examples, set up a mousewheel listener,
     * and set up the frame rate timer.
     */
    public void setup() {
    	/* On newer machines especially, the default JAVA2D renderer
    	 * is slow as hell and tends to drop frames.  I have no idea
    	 * why, but for now let's use P3D and live without the smoothing...
    	 */
    	size(640,480,P3D);
    	frameRate(targetFPS);
    	g = new ProcessingDebugDraw(this);
    	//smooth();
    	for (int i=0; i<100; ++i) {
    		this.requestFocus();
    	}
    	/* Register the examples */
    	// Simple functionality examples
    	registerExample(new BipedTest(this));
    	registerExample(new SpriteBinding(this));
    	registerExample(new Pulleys(this));
    	registerExample(new Overhang(this));
    	registerExample(new VaryingRestitution(this));
    	registerExample(new VaryingFriction(this));
    	registerExample(new MotorsAndLimits(this));
    	registerExample(new VerticalStack(this));
    	registerExample(new Domino(this));
    	registerExample(new CompoundShapes(this));
    	registerExample(new Chain(this));
    	registerExample(new Bridge(this));
    	registerExample(new Gears(this));
    	
    	// Shape drawing demo
    	registerExample(new ShapeDrawing(this));
    	
    	// Stress tests
    	registerExample(new Pyramid(this));
    	registerExample(new DominoTower(this));
    	registerExample(new Circles(this));
    	
    	// Bug tests
    	registerExample(new CCDTest(this));
    	registerExample(new DistanceTest(this));
    	//registerExample(new BugTest(this));

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
    	
    	options = new TestbedOptions(this);
    }
    
    /**
     * This is the main looping function, and is called targetFPS times per second.
     * In the testbed, Processing takes care of the timing of these calls for us,
     * but in your own game you will likely need to handle that yourself.  This function
     * also keeps detailed track of the current FPS and Vec2 creations for optimization
     * purposes.
     */
    public void draw() {

    	if (handleOptions) {
    		options.handleOptions();
    	} else{
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
    			TestSettings s = currentTest.settings; //copy settings
    			currentTest.initialize();
    			if (s != null) currentTest.settings = s;
    			nanoStart = System.nanoTime();
    			frameCount = 0;
    		}
    		currentTest.m_textLine = AbstractExample.textLineHeight;
    		g.drawString(5, currentTest.m_textLine, currentTest.getName(),AbstractExample.white);
    		currentTest.m_textLine += 2*AbstractExample.textLineHeight;

    		/* Take our time step (drawing is done here, too) */
    		currentTest.step();

    		/* If the user wants to move the canvas, do it */
    		handleCanvasDrag();


    		/* ==== Vec2 creation and FPS reporting ==== */
    		if (currentTest.settings.drawStats) {
    			g.drawString(5, currentTest.m_textLine, "Vec2 creations/frame: "+Vec2.creationCount, AbstractExample.white);
    			currentTest.m_textLine += AbstractExample.textLineHeight;
    		}

    		for (int i=0; i<fpsAverageCount-1; ++i) {
    			nanos[i] = nanos[i+1];
    		}
    		nanos[fpsAverageCount-1] = System.nanoTime();
    		float averagedFPS = (float) ( (fpsAverageCount-1) * 1000000000.0 / (nanos[fpsAverageCount-1]-nanos[0]));
    		++frameCount;
    		float totalFPS = (float) (frameCount * 1000000000 / (1.0*(System.nanoTime()-nanoStart)));        
    		if (currentTest.settings.drawStats) {
    			g.drawString(5, currentTest.m_textLine, "Average FPS ("+fpsAverageCount+" frames): "+averagedFPS, AbstractExample.white);
    			currentTest.m_textLine += AbstractExample.textLineHeight;
    			g.drawString(5, currentTest.m_textLine, "Average FPS (entire test): "+totalFPS, AbstractExample.white);
    			currentTest.m_textLine += AbstractExample.textLineHeight;
    		}
    	}

		/* Store whether the mouse was pressed this step */
		pmousePressed = mousePressed;
    }
    
    /**
     * Allows the world to be dragged with a right-click.
     */
    public void handleCanvasDrag() {
    	//Handle mouse dragging stuff
        //Left mouse attaches mouse joint to object.
        //Right mouse drags canvas.
    	ProcessingDebugDraw d = (ProcessingDebugDraw)(currentTest.m_debugDraw);
		
        //Vec2 mouseWorld = d.screenToWorld(mouseX, mouseY);
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
    
    /** Dispatch mousePressed events to the current test. */
    public void mousePressed() {
    	if (currentTest == null || handleOptions) return;
    	currentTest.mouseDown(new Vec2(mouseX,mouseY));
    }
    
    /** Dispatch mouseReleased events to the current test. */
    public void mouseReleased() {
    	if (currentTest == null || handleOptions) return;
    	currentTest.mouseUp();
    }
    
    /** Dispatch mouseMoved events to the current test. */
    public void mouseMoved() {
    	if (currentTest == null || handleOptions) return;
    	currentTest.mouseMove(new Vec2(mouseX,mouseY));
    }
    
    /** Dispatch mouseDragged events to the current test. */
    public void mouseDragged() {
    	mouseMoved();
    }
    
    /**
     * Apply keyboard shortcuts, do keypress handling, and then
     * send the key event to the current test if appropriate.
     */
    public void keyPressed() {
    	if (key=='o') {
    		handleOptions = !handleOptions;
    		if (handleOptions) options.initialize(currentTest);
    	}
    	
    	if (keyCode == PApplet.SHIFT) {
            shiftKey = true;
        }
    	if (handleOptions) return;
    	if (keyCode == PApplet.RIGHT) {
    		++currentTestIndex;
    		if (currentTestIndex >= tests.size()) currentTestIndex = 0;
    		//System.out.println(currentTestIndex);
    		currentTest = tests.get(currentTestIndex);
    		currentTest.needsReset = true;
    		return;
    	} else if (keyCode == PApplet.LEFT) {
    		--currentTestIndex;
    		if (currentTestIndex < 0) currentTestIndex = tests.size()-1;
    		//System.out.println(currentTestIndex);
    		currentTest = tests.get(currentTestIndex);
    		currentTest.needsReset = true;
    		return;
    	}
    	if (currentTest == null) return;
    	if (key == 'r') currentTest.needsReset = true;
    	if (key == ' ') currentTest.launchBomb();
    	if (key == 'p') {
    		currentTest.settings.pause = !currentTest.settings.pause;
    	}
    	if (key == '+' && currentTest.settings.pause) {
        	currentTest.settings.singleStep = true;
        }
    	if (key == 's') currentTest.settings.drawStats = !currentTest.settings.drawStats;
    	if (key == 'c') currentTest.settings.drawContactPoints = !currentTest.settings.drawContactPoints;
    	if (key == 'b') currentTest.settings.drawAABBs = !currentTest.settings.drawAABBs;
    		
    	currentTest.keyPressed(key);
    }
    
    /** Handle keyReleased events and pass them on to currentTest. */
    public void keyReleased() {
    	if (keyCode == PApplet.SHIFT) {
            shiftKey = false;
        }
    	if (currentTest == null) return;
    	currentTest.keyReleased(key);
    }
    
    /** Register an AbstractExample to the current list of examples. */
    public void registerExample(AbstractExample test) {
    	tests.add(test);
    }

    /** Start PApplet as a Java program (can also be run as an applet). */
    static public void main(String args[]) {
        PApplet.main(new String[] { "org.jbox2d.testbed.TestbedMain" });
    }

}
