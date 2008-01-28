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

public class Smasher extends PTest {
    public static final float speed = 20f;
    public static final float torque = 3500f;
    public static final float brakeTorque = 2500f;
    public static final float punchSpeed = 60f;
    public static final float punchForce = 15000f;
    public static final float punchLimit = 5f;
    public boolean punchRight = true;
    
    RevoluteJoint m_joint1;

    RevoluteJoint m_joint2;

    PrismaticJoint m_joint3;

    public Smasher() {
        super("Smasher");
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
        
        {
            // Make a pile of stuff to smash
            
            int numBoxes = 10;
            for (int i=0; i < numBoxes; ++i) {
                BodyDef bd = new BodyDef();
                BoxDef cd = new BoxDef();
                cd.extents = new Vec2( random(2.0f,2.5f), random(2.0f,2.5f) );
                cd.density = random(0.5f,1.0f);
                cd.friction = .1f;
                bd.addShape(cd);
                bd.position = new Vec2( random(30f,40f), random(1f,10f) );
                world.createBody(bd);
                bd.position = new Vec2( -random(30f,40f), random(1f,10f) );
                world.createBody(bd);
            }
            
            
        }

        {
            // Create car
            
            float carWidth = 5.0f;
            
            // Body
            
            BoxDef sd = new BoxDef();
            sd.extents = new Vec2(carWidth, 0.8f);
            sd.density = 10.0f;
            sd.friction = 0.8f;

            BodyDef bd = new BodyDef();
            bd.position = new Vec2(0.0f,5.2f);
            bd.addShape(sd);
            Body body1 = world.createBody(bd);
            setFollowed(body1);
            
            // "Puncher"
            
            sd.extents = new Vec2(carWidth*1.1f,0.3f);
            bd = new BodyDef();
            bd.addShape(sd);
            
            sd = new BoxDef();
            sd.density = 10.0f;
            sd.friction = 0.9f;
            sd.extents = new Vec2(.4f,.5f);
            sd.localPosition = new Vec2(carWidth*1.2f,0.1f);
            bd.addShape(sd);
            
            sd = new BoxDef();
            sd.density = 10.0f;
            sd.friction = 0.9f;
            sd.extents = new Vec2(.4f,.5f);
            sd.localPosition = new Vec2(-carWidth*1.2f,0.1f);
            bd.addShape(sd);
            
            bd.position = new Vec2(0.0f,6.3f);
            Body puncher = world.createBody(bd);
            
            
            // Connect puncher to body
            
            PrismaticJointDef pjd = new PrismaticJointDef();
            pjd.anchorPoint = new Vec2(0.0f, 6.3f);
            pjd.body1 = body1;
            pjd.body2 = puncher;
            pjd.axis = new Vec2(1.0f, 0.0f);
            pjd.motorSpeed = 40.0f;
            pjd.motorForce = 2000.0f;
            pjd.enableMotor = true;
            pjd.lowerTranslation = 0.0f;
            pjd.upperTranslation = 5.0f;
            pjd.enableLimit = true;
            m_joint3 = (PrismaticJoint) world.createJoint(pjd);
            
            
            // Wheels
            
            CircleDef cd = new CircleDef();
            cd.radius = 1.3f;
            cd.density = 10.0f;
            cd.friction = 1.0f;
            cd.restitution = .5f;
            
            bd = new BodyDef();
            bd.position = new Vec2(carWidth * 2.0f/3.0f,4.5f);
            bd.addShape(cd);
            Body body2 = world.createBody(bd);
            bd = new BodyDef();
            bd.position = new Vec2(-carWidth * 2.0f/3.0f,4.5f);
            bd.addShape(cd);
            Body body3 = world.createBody(bd);

            
            //Connect wheels to body
            
            RevoluteJointDef rjd = new RevoluteJointDef();
            rjd.anchorPoint = body2.m_position;
            rjd.body1 = body1;
            rjd.body2 = body2;
            m_joint1 = (RevoluteJoint) world.createJoint(rjd);
            rjd.anchorPoint = body3.m_position;
            rjd.body2 = body3;
            m_joint2 = (RevoluteJoint) world.createJoint(rjd);
        }
        
    }

    @Override
    protected void preStep() {
        
    }

    @Override
    protected void checkKeys() {
        if (newKeyDown['l']) {
            m_joint2.m_enableLimit = !m_joint2.m_enableLimit;
            m_joint3.m_enableLimit = !m_joint3.m_enableLimit;
            m_joint2.m_body1.wakeUp();
            m_joint2.m_body2.wakeUp();
            m_joint3.m_body2.wakeUp();
        }
        if (keyDown['m']) {
            m_joint3.m_lowerTranslation = -punchLimit;
            m_joint3.m_upperTranslation = punchLimit;
            punchRight = true;
            m_joint3.m_motorSpeed = punchSpeed;
            m_joint3.m_maxMotorForce = punchForce;
            m_joint3.m_enableMotor = true;
            m_joint3.m_body1.wakeUp();
            m_joint3.m_body2.wakeUp();
        } else if (keyDown['n']){
            m_joint3.m_lowerTranslation = -punchLimit;
            m_joint3.m_upperTranslation = punchLimit;
            punchRight = false;
            m_joint3.m_motorSpeed = -punchSpeed;
            m_joint3.m_maxMotorForce = punchForce;
            m_joint3.m_enableMotor = true;
            m_joint3.m_body1.wakeUp();
            m_joint3.m_body2.wakeUp();
        } else {
            m_joint3.m_lowerTranslation *= (punchRight)?0f:1f;
            m_joint3.m_upperTranslation *= (punchRight)?1f:0f;
            m_joint3.m_motorSpeed = .3f*punchSpeed * ((punchRight)?-1f:1f);
            m_joint3.m_maxMotorForce = punchForce;
            m_joint3.m_enableMotor = true;
        }
        if (keyDown['a']) {
            m_joint1.m_motorSpeed = speed;
            m_joint1.m_maxMotorTorque = torque;
            m_joint1.m_enableMotor = true;
            m_joint2.m_motorSpeed = speed;
            m_joint2.m_maxMotorTorque = torque;
            m_joint2.m_enableMotor = true;
            m_joint1.m_body1.wakeUp();
            m_joint1.m_body2.wakeUp();
            m_joint2.m_body2.wakeUp();
        } else if (keyDown['s']) {
            m_joint1.m_motorSpeed = -speed;
            m_joint1.m_maxMotorTorque = torque;
            m_joint1.m_enableMotor = true;
            m_joint2.m_motorSpeed = -speed;
            m_joint2.m_maxMotorTorque = torque;
            m_joint2.m_enableMotor = true;
            m_joint1.m_body1.wakeUp();
            m_joint1.m_body2.wakeUp();
            m_joint2.m_body2.wakeUp();
        } else if (keyDown['b']){
            m_joint1.m_enableMotor = true;
            m_joint1.m_motorSpeed = 0.0f;
            m_joint1.m_maxMotorTorque = brakeTorque;
            m_joint2.m_enableMotor = true;
            m_joint2.m_motorSpeed = 0.0f;
            m_joint2.m_maxMotorTorque = brakeTorque;
            m_joint1.m_body1.wakeUp();
            m_joint1.m_body2.wakeUp();
            m_joint2.m_body2.wakeUp();
        } else {
            m_joint1.m_enableMotor = false;
            m_joint2.m_enableMotor = false;
        }
    }

    /**
     * Entry point
     */
    public static void main(String[] argv) {
        PApplet.main(new String[] { "testbed.tests.Smasher" });

    }
}