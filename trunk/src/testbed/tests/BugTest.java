package testbed.tests;

import processing.core.PApplet;
import testbed.PTest;
import collision.*;

import common.Vec2;

import dynamics.Body;
import dynamics.BodyDef;
import dynamics.World;
import dynamics.joints.*;

public class BugTest extends PTest {

    public BugTest() {
        super("BugTest");
    }

    public void go(World world) {
        {        
            {
                BoxDef sd = new BoxDef();
                sd.extents = new Vec2(50.0f, 10.0f);
    
                BodyDef bd = new BodyDef();
                bd.position = new Vec2(0.0f, -10.0f);
                bd.addShape(sd);
                world.createBody(bd);
            }
            CircleDef sd1 = new CircleDef();
            sd1.radius = 0.5f;
            sd1.density = 2.0f;
            sd1.friction = 5.0f;

            BodyDef w1 = new BodyDef();
            w1.addShape(sd1);
            w1.position.set(-0.75f, 20.0f);
            Body b1 = m_world.createBody(w1);

            w1.position.set(0.75f, 20.0f);
            Body b2 = m_world.createBody(w1);

            w1.position.set(0.0f, 20.6f);
            Body b3 = m_world.createBody(w1);

            DistanceJointDef jointDef = new DistanceJointDef();
            jointDef.collideConnected = true;
            jointDef.body1 = b1;
            jointDef.body2 = b2;
            jointDef.anchorPoint1 = b1.getCenterPosition();//.add(new Vec2(.1f,.1f));
            jointDef.anchorPoint2 = b2.getCenterPosition();//.add(new Vec2(3f,0f));

            m_world.createJoint(jointDef);

            RevoluteJointDef revoluteDef = new RevoluteJointDef();
            revoluteDef.collideConnected = false;
            revoluteDef.body1 = b3;
            revoluteDef.body2 = b1;
            revoluteDef.anchorPoint = b1.getCenterPosition();

            m_world.createJoint(revoluteDef);

            revoluteDef.body2 = b2;
            revoluteDef.enableMotor = false;
            revoluteDef.motorSpeed = 20.0f;
            revoluteDef.motorTorque = 1000.0f;

            revoluteDef.anchorPoint = b2.getCenterPosition();

            m_world.createJoint(revoluteDef);
         }
    }

    /**
     * Entry point
     */
    public static void main(String[] argv) {
        PApplet.main(new String[] { "testbed.tests.BugTest" });
    }
}