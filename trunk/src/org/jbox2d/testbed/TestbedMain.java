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
import org.jbox2d.collision.BoxDef;
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
import org.jbox2d.testbed.PTest.BoundImage;
import org.jbox2d.testbed.tests.Bridge;

import processing.core.PApplet;
import processing.core.PImage;

public class TestbedMain extends PApplet {
	static private ArrayList<AbstractExample> tests = new ArrayList<AbstractExample>(0);
	static private AbstractExample currentTest = null;
	static private int currentTestIndex = 0;
	
    public DebugDraw g;

    public TestbedMain() {
    	super();
    	g = new ProcessingDebugDraw(this);
    }
    
    public void setup() {
    	size(640,480);
    	smooth();
    	AbstractExample ex0 = new Bridge(this);
    	registerExample(ex0);
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
    	if (currentTest == null) return;
    	if (key == 'r') currentTest.needsReset = true;
    	if (key == ' ') currentTest.launchBomb();
    	if (key == 'p') {
    		currentTest.settings.singleStep = true;
    		currentTest.settings.pause = true;
    	}
    }
    
    static public void registerExample(AbstractExample test) {
    	tests.add(test);
    }

    static public void main(String args[]) {
        PApplet.main(new String[] { "org.jbox2d.testbed.TestbedMain" });
    }

}
