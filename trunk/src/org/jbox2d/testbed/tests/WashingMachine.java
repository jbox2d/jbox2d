package org.jbox2d.testbed.tests;

import org.jbox2d.collision.*;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.joints.*;
import org.jbox2d.testbed.AbstractExample;
import org.jbox2d.testbed.TestbedMain;


public class WashingMachine extends AbstractExample {

	public WashingMachine(TestbedMain _parent) {
		super(_parent);
	}

	@Override
	public void create() {
		//TODO
	}

	@Override
	public String getName() {
		return "Washing Machine";
	}
	
/*
    RevoluteJoint m_joint1;

    public WashingMachine() {
        super("WashingMachine");
    }

    public void go(World world) {
        
        Body ground = null;
        {
            BoxDef sd = new BoxDef();
            sd.extents = new Vec2(50.0f, 10.0f);

            BodyDef bd = new BodyDef();
            bd.position = new Vec2(0.0f, -50.0f);
            bd.addShape(sd);
            ground = world.createBody(bd);
        }

        {
            CircleDef cd = new CircleDef();

            BodyDef bd = new BodyDef();
            int numPieces = 30;
            float radius = 13f;
            for (int i=0; i<numPieces; i++) {
                cd = new CircleDef();
                cd.radius = 1.2f;
                cd.density = 5.0f;
                cd.friction = 0.5f;
                cd.restitution = 0.5f;
                float xPos = radius * (float)Math.cos(2f*Math.PI * (i / (float)(numPieces)));
                float yPos = radius * (float)Math.sin(2f*Math.PI * (i / (float)(numPieces)));
                //System.out.println(xPos+ " "+yPos);
                cd.localPosition = new Vec2(xPos,yPos);
                bd.addShape(cd);   
            }
            
            bd.position = new Vec2(0.0f,5.0f);
            Body body = world.createBody(bd);

            RevoluteJointDef rjd = new RevoluteJointDef();
            rjd.anchorPoint = body.m_position.clone();
            rjd.body1 = ground;
            rjd.body2 = body;
            rjd.motorSpeed = (float) Math.PI / 4.0f;
            rjd.motorTorque = 50000.0f;
            rjd.enableMotor = true;
            world.createJoint(rjd);
            
            
            int loadSize = 15;
            for (int i=0; i < loadSize; i++) {
                float ang = random(0f,2*(float)Math.PI);
                float rad = random(0f,.8f*radius);
                float xPos = rad * (float)Math.cos(ang);
                float yPos = bd.position.y + rad * (float)Math.sin(ang);
                Ragdoll.makeRagdoll(.3f, new Vec2(xPos,yPos), world);
            }
            
            loadSize = 30;
            for (int i=0; i<loadSize; i++) {
                BoxDef box = new BoxDef();
                BodyDef bod = new BodyDef();
                box.extents = new Vec2( random(.4f,.9f), random(.4f,.9f) );
                box.density = 5.0f;
                box.friction = 0.5f;
                box.restitution = 0.5f;
                float ang = random(0f,2*(float)Math.PI);
                float rad = random(0f,.8f*radius);
                float xPos = rad * (float)Math.cos(ang);
                float yPos = bd.position.y + rad * (float)Math.sin(ang);
                bod.addShape(box);
                bod.position = new Vec2(xPos,yPos);
                world.createBody(bod);
            }

            for (int i=0; i<loadSize; i++) {
                CircleDef circ = new CircleDef();
                BodyDef bod = new BodyDef();
                circ.radius = random(.4f,.9f);
                circ.density = 5.0f;
                circ.friction = 0.5f;
                circ.restitution = 0.5f;
                float ang = random(0f,2*(float)Math.PI);
                float rad = random(0f,.8f*radius);
                float xPos = rad * (float)Math.cos(ang);
                float yPos = bd.position.y + rad * (float)Math.sin(ang);
                bod.addShape(circ);
                bod.position = new Vec2(xPos,yPos);
                world.createBody(bod);
            }
        }
    }
    
    public void DrawJoint(Joint j) {
        //Do nothing, just don't draw the joint
    }

    @Override
    protected void checkKeys() {

    }

    public static void main(String[] argv) {
        // new MotorsAndLimits().start();
        PApplet.main(new String[] { "org.jbox2d.testbed.tests.WashingMachine" });

    }
    */
}
