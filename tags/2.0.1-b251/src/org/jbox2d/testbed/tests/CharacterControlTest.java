/**
 * 
 */
package org.jbox2d.testbed.tests;

import java.util.HashSet;
import java.util.Set;

import org.jbox2d.collision.shapes.PolygonDef;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.joints.RevoluteJoint;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import org.jbox2d.testbed.AbstractExample;
import org.jbox2d.testbed.TestbedMain;
import org.jbox2d.testbed.tests.character.CharacterController;
import org.jbox2d.testbed.tests.character.CharacterModel;


/**
 * @author eric
 *
 */
public class CharacterControlTest extends AbstractExample {
	CharacterModel character;
	CharacterController characterController;
	
    public CharacterControlTest(TestbedMain _parent) {
        super(_parent);
    }
    
    public String getExampleInstructions() {
    	return "[w/a/s/d] Move character\n[space] Jump";
    }

    public void create(){
    	Body ground = null;
    	{
    		PolygonDef sd = new PolygonDef();
    		sd.setAsBox(50.0f, 10.0f,new Vec2(0.0f, -10.0f),0.0f);
    		sd.userData = "Do Not Destroy Me";

    		ground = m_world.getGroundBody();
    		ground.createShape(sd);
    		
    	}
    	
    	{
    		PolygonDef pd = new PolygonDef();
    		pd.setAsBox(10.0f, 1.0f);
    		pd.density = 1.0f;
    		BodyDef bd = new BodyDef();
    		bd.position.set(0.0f,10.0f);
    		Body bod = m_world.createBody(bd);
    		bod.createShape(pd);
    		bod.setMassFromShapes();
    		
    		RevoluteJointDef rjd = new RevoluteJointDef();
    		rjd.initialize(ground, bod, bod.getWorldCenter());
    		rjd.collideConnected = false;
    		RevoluteJoint rj = (RevoluteJoint)m_world.createJoint(rjd);
    		rj.enableMotor(true);
    		rj.setMaxMotorTorque(10000f);
    		rj.setMotorSpeed(1.0f);
    	}

    	{
    		character = new CharacterModel(m_world,new Vec2(0.0f, 5.0f), new Vec2(0.0f, 5.5f), 0.31f);
    		characterController = new CharacterController();
    		character.setController(characterController);
    		characterController.setCharacter(character);
    	}
    }

    public void mouseDown(Vec2 p1) {
    	Vec2 p = new Vec2();
    	this.viewport.getScreenToWorld(p1, p);
    	if (this.parent.mousePressed && this.getStaticBodyAtPoint(p) == null) {
    		int x = Math.round(p.x);
        	int y = Math.round(p.y);
        	createGroundBlock(new Vec2(x, y), 1.0f);
        	mergeBlocks(x, y);
    	}
    }
    
    public void mouseMove(Vec2 p1) {
    	Vec2 p = new Vec2();
    	this.viewport.getScreenToWorld(p1, p);
    	if (this.parent.mousePressed && this.getStaticBodyAtPoint(p) == null) {
    		int x = Math.round(p.x);
        	int y = Math.round(p.y);
        	createGroundBlock(new Vec2(x, y), 1.0f);
        	mergeBlocks(x, y);
    	}
    }
    
    private void createGroundBlock(Vec2 center, float size) {
    	if (!inRange(center)) return;
     	PolygonDef pd = new PolygonDef();
    	pd.setAsBox(size*.5f, size*.5f, center, 0.0f);
    	m_world.refilter(m_world.getGroundBody().createShape(pd));
    	
    }
    private void createGroundBlock(Vec2 corner1, Vec2 corner2) {
    	if (!inRange(corner1) && inRange(corner2)) return;
    	PolygonDef pd = new PolygonDef();
    	Vec2 center = corner1.add(corner2).mul(0.5f);
    	float hw = .5f*(corner2.x - corner1.x);
    	float hh = .5f*(corner2.y - corner1.y);
    	pd.setAsBox(hw, hh, center, 0.0f);
    	m_world.refilter(m_world.getGroundBody().createShape(pd));
    }
    
    //This function merges neighboring tiles so that the player doesn't bounce on
    //long flat sections.  You can also use edge shapes.
    private void mergeBlocks(int x, int y) {
    	if (!inRange(new Vec2(x, y))) return;
    	int leftX = x;
    	int rightX = x;
    	Set<Shape> shapes = new HashSet<Shape>();
    	while (true) {
    		Shape shapeAtPoint = this.getStaticShapeAtPoint(new Vec2(leftX,y));
    		if (shapeAtPoint != null && shapeAtPoint.m_userData == null) {
    			shapes.add(shapeAtPoint);
    			leftX--;
    		} else {
    			if (leftX < x) leftX++;
    			break;
    		}
    	}
    	while (true) {
    		Shape shapeAtPoint = this.getStaticShapeAtPoint(new Vec2(rightX,y));
    		if (shapeAtPoint != null && shapeAtPoint.m_userData == null) {
    			shapes.add(shapeAtPoint);
    			rightX++;
    		} else {
    			if (rightX > x) rightX--;
    			break;
    		}
    	}
    	for (Shape shape:shapes) {
    		if (shape.m_userData == null) shape.getBody().destroyShape(shape);
    	}
    	createGroundBlock(new Vec2(leftX-.5f,y-.5f), new Vec2(rightX+.5f,y+.5f));
    }

    public void preStep() {
    	characterController.setLeft(keyDown['a']);
    	characterController.setRight(keyDown['d']);
    	characterController.setJump(keyDown['w']);
    	characterController.step(1.0f/super.settings.hz);
    	character.step(1.0f/super.settings.hz);
    }
    public void postStep() {
    	this.viewport.setCenter(character.getBody().getWorldCenter());
    }
    

}

