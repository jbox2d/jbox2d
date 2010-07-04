/*
 * Circles - A ridiculous circle-biased stress test.
 * 
 * Zoom out a bit to see the circles falling from above.
 */

package testbed.tests;

import processing.core.*;
import testbed.PTest;
import collision.*;

import common.Vec2;

import dynamics.Body;
import dynamics.BodyDef;
import dynamics.World;
import dynamics.joints.PrismaticJoint;
import dynamics.joints.PrismaticJointDef;
import dynamics.joints.RevoluteJointDef;
import dynamics.joints.RevoluteJoint;

public class Circles extends PTest {

    public Circles() {
        super("Circles");
    }

    public void go(World world) {
        Body ground = null;
        Body leftWall = null;
        Body rightWall = null;
        {
            // Ground
            BoxDef sd = new BoxDef();
            sd.extents = new Vec2(50.0f, 10.0f);
            sd.friction = 1.0f;
            BodyDef bd = new BodyDef();
            bd.position = new Vec2(0.0f, -10.0f);
            bd.addShape(sd);
            ground = world.createBody(bd);
            
            // Walls
            sd.extents = new Vec2(3.0f,50.0f);
            bd = new BodyDef();
            bd.addShape(sd);
            bd.position = new Vec2(53.0f,25.0f);
            rightWall = world.createBody(bd);
            bd.position = new Vec2(-53.0f,25.0f);
            leftWall = world.createBody(bd);
            
            // Corners 
            bd = new BodyDef();
            sd.extents = new Vec2(20.0f,3.0f);
            bd.addShape(sd);
            bd.rotation = (float)(-Math.PI/4.0);
            bd.position = new Vec2(-40f,0.0f);
            world.createBody(bd);
            bd.rotation = (float)(Math.PI/4.0);
            bd.position = new Vec2(40f,0.0f);
            world.createBody(bd);
            
        }
        
        CircleDef cd = new CircleDef();

        BodyDef bd = new BodyDef();
        int numPieces = 5;
        float radius = 6f;
        for (int i=0; i<numPieces; i++) {
            cd = new CircleDef();
            cd.radius = 1.2f;
            cd.density = 25.0f;
            cd.friction = 0.1f;
            cd.restitution = 0.9f;
            float xPos = radius * (float)Math.cos(2f*Math.PI * (i / (float)(numPieces)));
            float yPos = radius * (float)Math.sin(2f*Math.PI * (i / (float)(numPieces)));
            cd.localPosition = new Vec2(xPos,yPos);
            bd.addShape(cd);   
        }
        
        bd.position = new Vec2(0.0f,10.0f);
        Body body = world.createBody(bd);

        RevoluteJointDef rjd = new RevoluteJointDef();
        rjd.anchorPoint = body.m_position.clone();
        rjd.body1 = ground;
        rjd.body2 = body;
        rjd.motorSpeed = (float) Math.PI;
        rjd.motorTorque = 1000000.0f;
        rjd.enableMotor = true;
        world.createJoint(rjd);
        
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
                    bod.addShape(circ);
                    bod.position = new Vec2(xPos,yPos);
                    world.createBody(bod);
                }
            }
            
            
        }

    }


    /**
     * Entry point
     */
    public static void main(String[] argv) {
        PApplet.main(new String[] { "testbed.tests.Circles" });

    }
}