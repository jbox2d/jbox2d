package testbed.tests;

import processing.core.PApplet;
import testbed.PTest;
import collision.*;

import common.Vec2;

import dynamics.Body;
import dynamics.BodyDef;
import dynamics.World;
import dynamics.joints.*;

public class Ragdoll extends PTest {

    public Ragdoll() {
        super("Ragdoll");
    }

    public void go(World world) {
        
        Body ground = null;
        {
            BoxDef sd = new BoxDef();
            sd.extents = new Vec2(50.0f, 10.0f);

            BodyDef bd = new BodyDef();
            bd.position = new Vec2(0.0f, -10.0f);
            bd.addShape(sd);
            ground = world.createBody(bd);
        }
        
        makeRagdoll(1.0f,new Vec2(0.0f,12.0f),world);
        
    }
    
    public static void makeRagdoll(float size, Vec2 pos, World world) {
        float torsoWidth = 1.0f;
        float torsoHeight = 2.0f;
        float uArmWidth = .3f;
        float uArmHeight = 1.0f;
        float lArmWidth = .25f;
        float lArmHeight = 1.0f;
        float uLegWidth = 0.45f;
        float uLegHeight = 1.2f;
        float lLegWidth = 0.38f;
        float lLegHeight = 1.0f;
        float headRadius = 0.9f;
        float neckLength = 1.0f;
        
        float density = 5.0f;
        float restitution = 0.5f;
        float friction = 0.5f;
        
        //Torso
        BoxDef box = new BoxDef();
        box.extents = new Vec2(torsoWidth*size,torsoHeight*size);
        box.density = density; box.restitution = restitution; box.friction = friction;
        BodyDef torsoDef = new BodyDef();
        torsoDef.position = pos.clone();
        torsoDef.addShape(box);
        Body torso = world.createBody(torsoDef);
        
        //Arms
        box = new BoxDef();
        box.extents = new Vec2(uArmWidth*size,uArmHeight*size);
        box.density = density; box.restitution = restitution; box.friction = friction;
        BodyDef armDef = new BodyDef();
        armDef.position = pos.add(new Vec2((torsoWidth+uArmWidth)*size,size*(torsoHeight-uArmHeight)));
        armDef.addShape(box);
        Body ruArm = world.createBody(armDef);
        armDef.position = pos.add(new Vec2(-(torsoWidth+uArmWidth)*size,size*(torsoHeight-uArmHeight)));
        Body luArm = world.createBody(armDef);
        box = new BoxDef();
        box.extents = new Vec2(lArmWidth*size,lArmHeight*size);
        box.density = density; box.restitution = restitution; box.friction = friction;
        armDef = new BodyDef();
        armDef.position = pos.add(new Vec2((torsoWidth+uArmWidth)*size,size*(torsoHeight-2*uArmHeight-lArmHeight)));
        armDef.addShape(box);
        Body rlArm = world.createBody(armDef);
        armDef.position = pos.add(new Vec2(-(torsoWidth+uArmWidth)*size,size*(torsoHeight-2*uArmHeight-lArmHeight)));
        Body llArm = world.createBody(armDef);
        
        //Head
        CircleDef circle = new CircleDef();
        circle.radius = size*headRadius;
        circle.density = density; circle.restitution = restitution; circle.friction = friction;
        BodyDef headDef = new BodyDef();
        headDef.position = pos.add(new Vec2(0f,(torsoHeight + neckLength)*size));
        headDef.addShape(circle);
        Body head = world.createBody(headDef);
        
        //Legs
        box = new BoxDef();
        box.extents = new Vec2(uLegWidth*size,uLegHeight*size);
        box.density = density; box.restitution = restitution; box.friction = friction;
        BodyDef legDef = new BodyDef();
        legDef.position = pos.add(new Vec2((torsoWidth-uLegWidth)*size,size*(-torsoHeight-uLegHeight)));
        legDef.addShape(box);
        Body ruLeg = world.createBody(legDef);
        legDef.position = pos.add(new Vec2(-(torsoWidth-uLegWidth)*size,size*(-torsoHeight-uLegHeight)));
        Body luLeg = world.createBody(legDef);
        box = new BoxDef();
        box.extents = new Vec2(lArmWidth*size,lArmHeight*size);
        box.density = density; box.restitution = restitution; box.friction = friction;
        legDef = new BodyDef();
        legDef.position = pos.add(new Vec2((torsoWidth-uLegWidth)*size,size*(-torsoHeight-2*uLegHeight-lLegHeight)));
        legDef.addShape(box);
        Body rlLeg = world.createBody(legDef);
        legDef.position = pos.add(new Vec2(-(torsoWidth-uLegWidth)*size,size*(-torsoHeight-2*uLegHeight-lLegHeight)));
        Body llLeg = world.createBody(legDef);
        
        
        //Connect head to torso...
        RevoluteJointDef rjd = new RevoluteJointDef();
        rjd.anchorPoint = head.m_position.add(new Vec2(0f,-neckLength*size));
        rjd.body1 = head;
        rjd.body2 = torso;
        rjd.collideConnected = true;
        world.createJoint(rjd);
        
        //...torso to arms...
        rjd.anchorPoint = torso.m_position.add(new Vec2(size*(torsoWidth+uArmWidth),size*(torsoHeight)));
        rjd.body1 = torso;
        rjd.body2 = ruArm;
        world.createJoint(rjd);
        rjd.anchorPoint = torso.m_position.add(new Vec2(-size*(torsoWidth+uArmWidth),size*(torsoHeight)));
        rjd.body1 = torso;
        rjd.body2 = luArm;
        world.createJoint(rjd);
        
        //...upper arms to lower arms...
        rjd.anchorPoint = ruArm.m_position.add(new Vec2(0f,-uArmHeight*size));
        rjd.body1 = ruArm;
        rjd.body2 = rlArm;
        rjd.enableLimit = true;
        rjd.upperAngle = 1.5f;
        rjd.lowerAngle = -1.5f;
        rjd.collideConnected = false;
        world.createJoint(rjd);
        rjd.anchorPoint = luArm.m_position.add(new Vec2(0f,-uArmHeight*size));
        rjd.body1 = luArm;
        rjd.body2 = llArm;
        rjd.collideConnected = false;
        world.createJoint(rjd); 
        
        //...torso to legs...
        rjd.anchorPoint = ruLeg.m_position.add(new Vec2(0f,uLegHeight*size));
        rjd.body1 = torso;
        rjd.body2 = ruLeg;
        rjd.enableLimit = true;
        rjd.upperAngle = 1.5f;
        rjd.lowerAngle = -1.5f;
        rjd.collideConnected = false;
        world.createJoint(rjd);
        rjd.anchorPoint = luLeg.m_position.add(new Vec2(0f,uLegHeight*size));
        rjd.body1 = torso;
        rjd.body2 = luLeg;
        rjd.collideConnected = false;
        world.createJoint(rjd);        
        
        //...upper legs to lower legs...
        rjd.anchorPoint = ruLeg.m_position.add(new Vec2(0f,-uLegHeight*size));
        rjd.body1 = ruLeg;
        rjd.body2 = rlLeg;
        rjd.enableLimit = true;
        rjd.upperAngle = 1.5f;
        rjd.lowerAngle = -1.5f;
        rjd.collideConnected = false;
        world.createJoint(rjd);
        rjd.anchorPoint = luLeg.m_position.add(new Vec2(0f,-uLegHeight*size));
        rjd.body1 = luLeg;
        rjd.body2 = llLeg;
        rjd.collideConnected = false;
        world.createJoint(rjd); 
    }
 

    @Override
    protected void checkKeys() {

    }

    /**
     * Entry point
     */
    public static void main(String[] argv) {
        // new MotorsAndLimits().start();
        PApplet.main(new String[] { "testbed.tests.Ragdoll" });

    }
}