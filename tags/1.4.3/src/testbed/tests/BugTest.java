package testbed.tests;

import processing.core.PApplet;
import processing.core.PImage;
import testbed.PTest;
import collision.*;

import common.Vec2;

import dynamics.Body;
import dynamics.BodyDef;
import dynamics.World;
import dynamics.joints.*;

public class BugTest extends PTest {
    
    public Vec2[] localCoords;
    public Vec2[] texCoords;
    public PImage myImage;
    
    public Body missile;

    public BugTest() {
        super("BugTest");
    }

    public void go(World world) {
        {        

            // Define the ground box shape.
            BoxDef groundBoxDef = new BoxDef();

            // The extents are the half-widths of the box.
            groundBoxDef.extents.set(50.0f, 10.0f);

            // Set the density of the ground box to zero. This will
            // make the ground body static (fixed).
            groundBoxDef.density = 0.0f;

            // Define the ground body.
            BodyDef groundBodyDef = new BodyDef();
            groundBodyDef.position.set(0.0f, -10.0f);

            // Part of a body's def is its list of shapes.
            groundBodyDef.addShape(groundBoxDef);

            // Call the body factory which allocates memory for the ground body
            // from a pool and creates the ground box shape (also from a pool).
            // The body is also added to the world.
            world.createBody(groundBodyDef);

            // Define another box shape for our dynamic body.
            BoxDef boxDef = new BoxDef();
            boxDef.extents.set(1.0f, 5.0f);

            // Set the box density to be non-zero, so it will be dynamic.
            boxDef.density = 1.0f;

            // Override the default friction.
            boxDef.friction = 25.3f;
            boxDef.restitution = 0.0f;
            

            // Define the dynamic body. We set its position,
            // add the box shape, and call the body factory.
            BodyDef bodyDef = new BodyDef();
            bodyDef.position.set(0.0f, 4.0f);
            bodyDef.addShape(boxDef);
            missile = world.createBody(bodyDef);
            
         }
        
        //textureMode(NORMALIZED);
    }
    
    protected void preStep() {
        
           float velocityAngle;
           Vec2 velocity;
           float torque;
           float torqueFactor = -100.1f;
           float torqueDamping = -torqueFactor;
           
           velocity = missile.getLinearVelocity();
           float absVel = velocity.length();
           
           velocityAngle = (float) (Math.atan2( velocity.y, velocity.x ) - (Math.PI / 2));
           float angVelDelta =  missile.m_rotation - velocityAngle;
           
           //Constrain to -PI -> PI
           while (angVelDelta > Math.PI){
               angVelDelta -= 2*Math.PI;
           }
           while (angVelDelta < -Math.PI){
               angVelDelta += 2*Math.PI;
           }
           
           torque = absVel* torqueFactor * (angVelDelta) - absVel*missile.m_angularVelocity * torqueDamping;
           missile.applyTorque(torque);
           //missile.m_rotation = velocityAngle;
        
    }

    /**
     * Entry point
     */
    public static void main(String[] argv) {
        PApplet.main(new String[] { "testbed.tests.BugTest" });
    }
}