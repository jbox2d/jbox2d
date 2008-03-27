/*
 * Circles - A ridiculous circle-biased stress test.
 * 
 * Zoom out a bit to see the circles falling from above.
 */

package org.jbox2d.testbed.tests;

import org.jbox2d.collision.*;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.joints.PrismaticJoint;
import org.jbox2d.dynamics.joints.PrismaticJointDef;
import org.jbox2d.dynamics.joints.RevoluteJoint;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import org.jbox2d.testbed.AbstractExample;
import org.jbox2d.testbed.TestbedMain;

import processing.core.*;

public class Circles extends AbstractExample {
	private boolean firstTime = true;
	
    public Circles(TestbedMain parent) {
        super(parent);
    }

    public void create() {
    	if (firstTime) {
			setCamera(0f, 20f, 5f);
			firstTime = false;
		}
    	
        Body ground = m_world.getGroundBody();
        Body leftWall = null;
        Body rightWall = null;
        {
            // Ground
            PolygonDef sd = new PolygonDef();
            sd.setAsBox(50.0f, 10.0f);
            sd.friction = 1.0f;
            BodyDef bd = new BodyDef();
            bd.position = new Vec2(0.0f, -10.0f);
            m_world.createStaticBody(bd).createShape(sd);
            
            
            // Walls
            sd.setAsBox(3.0f,50.0f);
            bd = new BodyDef();
            bd.position = new Vec2(53.0f,25.0f);
            rightWall = m_world.createStaticBody(bd);
            rightWall.createShape(sd);
            bd.position = new Vec2(-53.0f,25.0f);
            leftWall = m_world.createStaticBody(bd);
            leftWall.createShape(sd);
            
            // Corners 
            bd = new BodyDef();
            sd.setAsBox(20.0f,3.0f);
            bd.angle = (float)(-Math.PI/4.0);
            bd.position = new Vec2(-40f,0.0f);
            Body myBod = m_world.createStaticBody(bd);
            myBod.createShape(sd);
            bd.angle = (float)(Math.PI/4.0);
            bd.position = new Vec2(40f,0.0f);
            myBod = m_world.createStaticBody(bd);
            myBod.createShape(sd);
            
        }
        
        CircleDef cd = new CircleDef();

        BodyDef bd = new BodyDef();
        int numPieces = 5;
        float radius = 6f;
        bd.position = new Vec2(0.0f,10.0f);
        Body body = m_world.createDynamicBody(bd);
        for (int i=0; i<numPieces; i++) {
            cd = new CircleDef();
            cd.radius = 1.2f;
            cd.density = 25.0f;
            cd.friction = 0.1f;
            cd.restitution = 0.9f;
            float xPos = radius * (float)Math.cos(2f*Math.PI * (i / (float)(numPieces)));
            float yPos = radius * (float)Math.sin(2f*Math.PI * (i / (float)(numPieces)));
            cd.localPosition = new Vec2(xPos,yPos);
            body.createShape(cd);   
        }
        body.setMassFromShapes();

        RevoluteJointDef rjd = new RevoluteJointDef();
        rjd.initialize(body,ground,body.getPosition());
        rjd.motorSpeed = (float) Math.PI;
        rjd.maxMotorTorque = 1000000.0f;
        rjd.enableMotor = true;
        m_world.createJoint(rjd);
        
        {
            int loadSize = 45;

            for (int j=0; j<10; j++){
                for (int i=0; i<loadSize; i++) {
                    CircleDef circ = new CircleDef();
                    BodyDef bod = new BodyDef();
                    circ.radius = 1.0f+(i%2==0?1.0f:-1.0f)*.5f*(i/(float)loadSize);
                    circ.density = 5.0f;
                    circ.friction = 0.1f;
                    circ.restitution = 0.5f;
                    float xPos = -45f + 2*i;
                    float yPos = 50f+j;
                    bod.position = new Vec2(xPos,yPos);
                    Body myBody = m_world.createDynamicBody(bod);
                    myBody.createShape(circ);
                    myBody.setMassFromShapes();
                    
                }
            }
            
            
        }

    }

    public String getName() {
    	return "Circle Stress Test";
    }
}