package org.jbox2d.testbed.tests;

import org.jbox2d.collision.PolygonDef;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import org.jbox2d.testbed.AbstractExample;
import org.jbox2d.testbed.TestbedMain;

import processing.core.PApplet;

/**
 * An example of Crayon Physics-style shape creation.
 * This is a simple method of allowing user input to
 * guide shape creation.  Rather than trying to decompose
 * the user-created polygon into a shape usable by JBox2d,
 * we create each segment of the stroke as an individual
 * shape, and finalize the shape when the "crayon" is
 * lifted.
 * <BR><BR>
 * This method is inefficient, and each segment of stroke
 * must be fairly large to avoid angering the engine, but
 * it is very robust against weird user input, which makes
 * it much easier to implement.
 */
public class ShapeDrawing extends AbstractExample {
/*
 * FIXME: Sometimes when you're far from the origin, an assertion
 * in PolygonShape.java (the toiSlop one) gets triggered.  I think
 * this is because of precision issues and the fact that we are
 * creating shapes so far from the body center (which is (0,0)).
 * Should set it up to center the shape creation on the center of mass. 
 */
	
	private boolean activeMouseStroke;
	private Vec2[] mouseStroke;
	private final int mouseStrokeMaxLength = 1000;
	private int mouseStrokeLength;
	//Minimum distance (world units) the mouse must move to create a new stroke segment
	private final float minMouseStrokeChange = .2f;
	// Radius of stroke - should be at least roughly .2f for the engine to be happy
	// (.1f works with toiSlop, but angers the mass/inertia calculations for single segments)
	private float strokeWidth = .2f; 
	
	public ShapeDrawing(TestbedMain _parent) {
		super(_parent);
	}

	@Override
	public void create() {
		activeMouseStroke = false;
		mouseStroke = new Vec2[mouseStrokeMaxLength];
		for (int i=0; i<mouseStroke.length; ++i) {
			mouseStroke[i] = new Vec2();
		}
		mouseStrokeLength = 0;
		
		// Create usual ground plane
		Body ground = null;
		{
			BodyDef bd = new BodyDef();
			bd.position.set(0.0f, -10.0f);
			ground = m_world.createStaticBody(bd);

			PolygonDef sd = new PolygonDef();
			sd.setAsBox(50.0f, 10.0f);
			ground.createShape(sd);
		}
	}
	
	public void preStep() {
		if (activeMouseStroke && mouseStrokeLength > 1) {
			for (int i=0; i<mouseStrokeLength-1; ++i) {
				m_debugDraw.drawSegment(mouseStroke[i], mouseStroke[i+1], white);
			}
		}
	}
	
	
	public void beginMouseStroke(){
		activeMouseStroke = true;
		mouseStrokeLength = 0;
		addStrokeSegment();
	}
	
	public void addStrokeSegment(){
		assert(activeMouseStroke == true);
		if (mouseStrokeLength == 0){
			mouseStroke[mouseStrokeLength++].set(mouseWorld);
		} else if (mouseStrokeLength < mouseStrokeMaxLength) {
			Vec2 worldDiff = mouseStroke[mouseStrokeLength-1].sub(mouseWorld);
			float norm = worldDiff.length();
			System.out.println(norm + " vs " + minMouseStrokeChange);
			if (norm > minMouseStrokeChange) {
				System.out.println("Creating...");
				mouseStroke[mouseStrokeLength++].set(mouseWorld);
			}
		}
	}
	
	public void finalizeStroke(){
		if (mouseStrokeLength < 2) return;
		BodyDef myBodyDef = new BodyDef();
		myBodyDef.isBullet = true;
		Body myBody = m_world.createDynamicBody(myBodyDef);
		for (int i=0; i<mouseStrokeLength-1; ++i) {
			PolygonDef sd = new PolygonDef();
			sd.density = 2.0f;
			sd.friction = 0.3f;
			System.out.println(mouseStroke[i]+" "+mouseStroke[i+1]);
			createStrokeRect(mouseStroke[i],mouseStroke[i+1],strokeWidth,myBody,sd);
		}
		myBody.setMassFromShapes();
		activeMouseStroke = false;
		mouseStrokeLength = 0;
	}
	
	public void createStrokeRect(Vec2 start, Vec2 end, float radius, Body body, PolygonDef sd) {
		//The shape count check appears unimportant 
		//if (body.m_shapeCount >= Settings.maxShapesPerBody) return;
		Vec2 tangent = end.sub(start); //not normalized
		Vec2 perp = new Vec2(tangent.y,-tangent.x);
		perp.normalize();
		perp.mulLocal(radius);
		sd.vertices.add(start.add(perp));
		sd.vertices.add(end.add(perp));
		sd.vertices.add(end.sub(perp));
		sd.vertices.add(start.sub(perp));
		body.createShape(sd);
	}
	
	public void mouseDown(Vec2 p){
    	mouseScreen.set(p);
		mouseWorld.set(m_debugDraw.screenToWorld(p));
		if (parent.mouseButton == PApplet.RIGHT) return;
		if (parent.shiftKey) {
    		spawnBomb(mouseWorld);
    		return;
    	}
		System.out.println("Mouse down");
		beginMouseStroke();
	}
	
	public void mouseUp(){
		if (bombSpawning) {
        	completeBombSpawn();
        }
		if (activeMouseStroke) {
			finalizeStroke();
		}
		activeMouseStroke = false;
	}
	
	public void mouseMove(Vec2 p){
    	mouseScreen.set(p);
		mouseWorld.set(m_debugDraw.screenToWorld(p));
		if (activeMouseStroke) {
			addStrokeSegment();
		}
	}

	@Override
	public String getName() {
		return "Shape Drawing Example";
	}
	
	public String getExampleInstructions() {
		return "Use the mouse to paint.\nMouse dragging is disabled in this demo.";
	}

}
