/*******************************************************************************
 * Copyright (c) 2013, Daniel Murphy
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 	* Redistributions of source code must retain the above copyright notice,
 * 	  this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright notice,
 * 	  this list of conditions and the following disclaimer in the documentation
 * 	  and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
package org.jbox2d.serialization.pb;

import java.io.IOException;
import java.io.InputStream;
import java.util.HashMap;
import java.util.Map;

import org.box2d.proto.Box2D.PbBody;
import org.box2d.proto.Box2D.PbFixture;
import org.box2d.proto.Box2D.PbJoint;
import org.box2d.proto.Box2D.PbJointType;
import org.box2d.proto.Box2D.PbShape;
import org.box2d.proto.Box2D.PbVec2;
import org.box2d.proto.Box2D.PbWorld;
import org.jbox2d.collision.shapes.ChainShape;
import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.joints.ConstantVolumeJointDef;
import org.jbox2d.dynamics.joints.DistanceJoint;
import org.jbox2d.dynamics.joints.DistanceJointDef;
import org.jbox2d.dynamics.joints.FrictionJointDef;
import org.jbox2d.dynamics.joints.GearJointDef;
import org.jbox2d.dynamics.joints.Joint;
import org.jbox2d.dynamics.joints.JointDef;
import org.jbox2d.dynamics.joints.MouseJointDef;
import org.jbox2d.dynamics.joints.PrismaticJointDef;
import org.jbox2d.dynamics.joints.PulleyJointDef;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import org.jbox2d.dynamics.joints.RopeJointDef;
import org.jbox2d.dynamics.joints.WeldJointDef;
import org.jbox2d.dynamics.joints.WheelJointDef;
import org.jbox2d.serialization.JbDeserializer;
import org.jbox2d.serialization.UnsupportedListener;
import org.jbox2d.serialization.UnsupportedObjectException;
import org.jbox2d.serialization.UnsupportedObjectException.Type;

public class PbDeserializer implements JbDeserializer {

  private ObjectListener listener = null;
  private UnsupportedListener ulistener = null;

  public PbDeserializer() {}

  public PbDeserializer(UnsupportedListener argListener) {
    ulistener = argListener;
  }

  public PbDeserializer(ObjectListener argObjectListner) {
    listener = argObjectListner;
  }

  public PbDeserializer(UnsupportedListener argListener, ObjectListener argObjectListner) {
    ulistener = argListener;
    listener = argObjectListner;
  }

  @Override
  public void setObjectListener(ObjectListener argListener) {
    listener = argListener;
  }

  @Override
  public void setUnsupportedListener(UnsupportedListener argListener) {
    ulistener = argListener;
  }

  private boolean isIndependentJoint(PbJointType argType) {
    return argType != PbJointType.GEAR && argType != PbJointType.CONSTANT_VOLUME;
  }

  @Override
  public World deserializeWorld(InputStream argInput) throws IOException {
    PbWorld world = PbWorld.parseFrom(argInput);
    return deserializeWorld(world);
  }

  public World deserializeWorld(PbWorld pbWorld) {
    World world = new World(pbToVec(pbWorld.getGravity()));

    world.setAutoClearForces(pbWorld.getAutoClearForces());
    world.setContinuousPhysics(pbWorld.getContinuousPhysics());
    world.setWarmStarting(pbWorld.getWarmStarting());
    world.setSubStepping(pbWorld.getSubStepping());

    HashMap<Integer, Body> bodyMap = new HashMap<Integer, Body>();
    HashMap<Integer, Joint> jointMap = new HashMap<Integer, Joint>();

    for (int i = 0; i < pbWorld.getBodiesCount(); i++) {
      PbBody pbBody = pbWorld.getBodies(i);
      Body body = deserializeBody(world, pbBody);
      bodyMap.put(i, body);
    }

    // first pass, indep joints
    int cnt = 0;
    for (int i = 0; i < pbWorld.getJointsCount(); i++) {
      PbJoint pbJoint = pbWorld.getJoints(i);
      if (isIndependentJoint(pbJoint.getType())) {
        Joint joint = deserializeJoint(world, pbJoint, bodyMap, jointMap);
        jointMap.put(cnt, joint);
        cnt++;
      }
    }

    // second pass, dep joints
    for (int i = 0; i < pbWorld.getJointsCount(); i++) {
      PbJoint pbJoint = pbWorld.getJoints(i);
      if (!isIndependentJoint(pbJoint.getType())) {
        Joint joint = deserializeJoint(world, pbJoint, bodyMap, jointMap);
        jointMap.put(cnt, joint);
        cnt++;
      }
    }

    if (listener != null && pbWorld.hasTag()) {
      listener.processWorld(world, pbWorld.getTag());
    }
    return world;
  }

  @Override
  public Body deserializeBody(World argWorld, InputStream argInput) throws IOException {
    PbBody body = PbBody.parseFrom(argInput);
    return deserializeBody(argWorld, body);
  }

  public Body deserializeBody(World argWorld, PbBody argBody) {
    PbBody b = argBody;

    BodyDef bd = new BodyDef();
    bd.position.set(pbToVec(b.getPosition()));
    bd.angle = b.getAngle();
    bd.linearDamping = b.getLinearDamping();
    bd.angularDamping = b.getAngularDamping();
    bd.gravityScale = b.getGravityScale();
    // velocities are populated after fixture addition
    
    bd.bullet = b.getBullet();
    bd.allowSleep = b.getAllowSleep();
    bd.awake = b.getAwake();
    bd.active = b.getActive();
    bd.fixedRotation = b.getFixedRotation();

    switch (b.getType()) {
      case DYNAMIC:
        bd.type = BodyType.DYNAMIC;
        break;
      case KINEMATIC:
        bd.type = BodyType.KINEMATIC;
        break;
      case STATIC:
        bd.type = BodyType.STATIC;
        break;
      default:
        UnsupportedObjectException e =
            new UnsupportedObjectException("Unknown body type: " + argBody.getType(), Type.BODY);
        if (ulistener == null || ulistener.isUnsupported(e)) {
          throw e;
        }
        return null;
    }

    Body body = argWorld.createBody(bd);

    for (int i = 0; i < b.getFixturesCount(); i++) {
      deserializeFixture(body, b.getFixtures(i));
    }
    
    // adding fixtures can change this, so we put this here and set it directly in the body
    body.m_linearVelocity.set(pbToVec(b.getLinearVelocity()));
    body.m_angularVelocity = b.getAngularVelocity();

    if (listener != null && b.hasTag()) {
      listener.processBody(body, b.getTag());
    }
    return body;
  }

  @Override
  public Fixture deserializeFixture(Body argBody, InputStream argInput) throws IOException {
    PbFixture fixture = PbFixture.parseFrom(argInput);
    return deserializeFixture(argBody, fixture);
  }

  public Fixture deserializeFixture(Body argBody, PbFixture argFixture) {
    PbFixture f = argFixture;

    FixtureDef fd = new FixtureDef();
    fd.density = f.getDensity();
    fd.filter.categoryBits = f.getFilter().getCategoryBits();
    fd.filter.groupIndex = f.getFilter().getGroupIndex();
    fd.filter.maskBits = f.getFilter().getMaskBits();
    fd.friction = f.getFriction();
    fd.isSensor = f.getSensor();
    fd.restitution = f.getRestitution();
    fd.shape = deserializeShape(f.getShape());

    Fixture fixture = argBody.createFixture(fd);
    if (listener != null && f.hasTag()) {
      listener.processFixture(fixture, f.getTag());
    }
    return fixture;
  }

  @Override
  public Shape deserializeShape(InputStream argInput) throws IOException {
    PbShape s = PbShape.parseFrom(argInput);
    return deserializeShape(s);
  }

  public Shape deserializeShape(PbShape argShape) {
    PbShape s = argShape;

    Shape shape = null;
    switch (s.getType()) {
      case CIRCLE:
        CircleShape c = new CircleShape();
        c.m_p.set(pbToVec(s.getCenter()));
        shape = c;
        break;
      case POLYGON:
        PolygonShape p = new PolygonShape();
        p.m_centroid.set(pbToVec(s.getCentroid()));
        p.m_count = s.getPointsCount();
        for (int i = 0; i < p.m_count; i++) {
          p.m_vertices[i].set(pbToVec(s.getPoints(i)));
          p.m_normals[i].set(pbToVec(s.getNormals(i)));
        }
        shape = p;
        break;
      case EDGE:
        EdgeShape edge = new EdgeShape();
        edge.m_vertex0.set(pbToVec(s.getV0()));
        edge.m_vertex1.set(pbToVec(s.getV1()));
        edge.m_vertex2.set(pbToVec(s.getV2()));
        edge.m_vertex3.set(pbToVec(s.getV3()));
        edge.m_hasVertex0 = s.getHas0();
        edge.m_hasVertex3 = s.getHas3();
        shape = edge;
        break;
      case CHAIN: {
        ChainShape chain = new ChainShape();
        chain.m_count = s.getPointsCount();
        chain.m_vertices = new Vec2[chain.m_count];
        for (int i = 0; i < chain.m_count; i++) {
          chain.m_vertices[i] = new Vec2(pbToVec(s.getPoints(i)));
        }
        chain.m_hasPrevVertex = s.getHas0();
        chain.m_hasNextVertex = s.getHas3();
        chain.m_prevVertex.set(pbToVec(s.getPrev()));
        chain.m_nextVertex.set(pbToVec(s.getNext()));
        shape = chain;
        break;
      }
      default: {
        UnsupportedObjectException e =
            new UnsupportedObjectException("Unknown shape type: " + s.getType(), Type.SHAPE);
        if (ulistener == null || ulistener.isUnsupported(e)) {
          throw e;
        }
        return null;
      }
    }
    shape.m_radius = s.getRadius();

    if (listener != null && s.hasTag()) {
      listener.processShape(shape, s.getTag());
    }
    return shape;
  }

  @Override
  public Joint deserializeJoint(World argWorld, InputStream argInput,
      Map<Integer, Body> argBodyMap, Map<Integer, Joint> jointMap) throws IOException {
    PbJoint joint = PbJoint.parseFrom(argInput);
    return deserializeJoint(argWorld, joint, argBodyMap, jointMap);
  }

  public Joint deserializeJoint(World argWorld, PbJoint joint, Map<Integer, Body> argBodyMap,
      Map<Integer, Joint> jointMap) {
    JointDef jd = null;

    switch (joint.getType()) {
      case PRISMATIC: {
        PrismaticJointDef def = new PrismaticJointDef();
        jd = def;
        def.enableLimit = joint.getEnableLimit();
        def.enableMotor = joint.getEnableMotor();
        def.localAnchorA.set(pbToVec(joint.getLocalAnchorA()));
        def.localAnchorB.set(pbToVec(joint.getLocalAnchorB()));
        def.localAxisA.set(pbToVec(joint.getLocalAxisA()));
        def.lowerTranslation = joint.getLowerLimit();
        def.maxMotorForce = joint.getMaxMotorForce();
        def.motorSpeed = joint.getMotorSpeed();
        def.referenceAngle = joint.getRefAngle();
        def.upperTranslation = joint.getUpperLimit();
        break;
      }
      case REVOLUTE: {
        RevoluteJointDef def = new RevoluteJointDef();
        jd = def;
        def.enableLimit = joint.getEnableLimit();
        def.enableMotor = joint.getEnableMotor();
        def.localAnchorA.set(pbToVec(joint.getLocalAnchorA()));
        def.localAnchorB.set(pbToVec(joint.getLocalAnchorB()));
        def.lowerAngle = joint.getLowerLimit();
        def.maxMotorTorque = joint.getMaxMotorTorque();
        def.motorSpeed = joint.getMotorSpeed();
        def.referenceAngle = joint.getRefAngle();
        def.upperAngle = joint.getUpperLimit();
        break;
      }
      case DISTANCE: {
        DistanceJointDef def = new DistanceJointDef();
        jd = def;
        def.localAnchorA.set(pbToVec(joint.getLocalAnchorA()));
        def.localAnchorB.set(pbToVec(joint.getLocalAnchorB()));
        def.dampingRatio = joint.getDampingRatio();
        def.frequencyHz = joint.getFrequency();
        def.length = joint.getLength();
        break;
      }
      case PULLEY: {
        PulleyJointDef def = new PulleyJointDef();
        jd = def;
        def.localAnchorA.set(pbToVec(joint.getLocalAnchorA()));
        def.localAnchorB.set(pbToVec(joint.getLocalAnchorB()));
        def.groundAnchorA.set(pbToVec(joint.getGroundAnchorA()));
        def.groundAnchorB.set(pbToVec(joint.getGroundAnchorB()));
        def.lengthA = joint.getLengthA();
        def.lengthB = joint.getLengthB();
        def.ratio = joint.getRatio();
        break;
      }
      case MOUSE: {
        MouseJointDef def = new MouseJointDef();
        jd = def;
        def.dampingRatio = joint.getDampingRatio();
        def.frequencyHz = joint.getFrequency();
        def.maxForce = joint.getMaxForce();
        def.target.set(pbToVec(joint.getTarget()));
        break;
      }
      case GEAR: {
        GearJointDef def = new GearJointDef();
        jd = def;
        if (!jointMap.containsKey(joint.getJoint1())) {
          throw new IllegalArgumentException("Index " + joint.getJoint1()
              + " is not present in the joint map.");
        }
        def.joint1 = jointMap.get(joint.getJoint1());
        if (!jointMap.containsKey(joint.getJoint2())) {
          throw new IllegalArgumentException("Index " + joint.getJoint2()
              + " is not present in the joint map.");
        }
        def.joint2 = jointMap.get(joint.getJoint2());
        def.ratio = joint.getRatio();
        break;
      }
      case WHEEL: {
        WheelJointDef def = new WheelJointDef();
        jd = def;
        def.localAnchorA.set(pbToVec(joint.getLocalAnchorA()));
        def.localAnchorB.set(pbToVec(joint.getLocalAnchorB()));
        def.localAxisA.set(pbToVec(joint.getLocalAxisA()));
        def.enableMotor = joint.getEnableMotor();
        def.maxMotorTorque = joint.getMaxMotorTorque();
        def.motorSpeed = joint.getMotorSpeed();
        def.frequencyHz = joint.getFrequency();
        def.dampingRatio = joint.getDampingRatio();
        break;
      }
      case WELD: {
        WeldJointDef def = new WeldJointDef();
        jd = def;
        def.localAnchorA.set(pbToVec(joint.getLocalAnchorA()));
        def.localAnchorB.set(pbToVec(joint.getLocalAnchorB()));
        def.referenceAngle = joint.getRefAngle();
        def.frequencyHz = joint.getFrequency();
        def.dampingRatio = joint.getDampingRatio();
        break;
      }
      case FRICTION: {
        FrictionJointDef def = new FrictionJointDef();
        jd = def;
        def.localAnchorA.set(pbToVec(joint.getLocalAnchorA()));
        def.localAnchorB.set(pbToVec(joint.getLocalAnchorB()));
        def.maxForce = joint.getMaxForce();
        def.maxTorque = joint.getMaxTorque();
        break;
      }
      case ROPE: {
        RopeJointDef def = new RopeJointDef();
        jd = def;
        def.localAnchorA.set(pbToVec(joint.getLocalAnchorA()));
        def.localAnchorB.set(pbToVec(joint.getLocalAnchorB()));
        def.maxLength = joint.getMaxLength();
        return null;
      }
      case CONSTANT_VOLUME: {
        ConstantVolumeJointDef def = new ConstantVolumeJointDef();
        jd = def;
        def.dampingRatio = joint.getDampingRatio();
        def.frequencyHz = joint.getFrequency();
        if (joint.getBodiesCount() != joint.getJointsCount()) {
          throw new IllegalArgumentException(
              "Constant volume joint must have bodies and joints defined");
        }
        for (int i = 0; i < joint.getBodiesCount(); i++) {
          int body = joint.getBodies(i);
          if (!argBodyMap.containsKey(body)) {
            throw new IllegalArgumentException("Index " + body + " is not present in the body map");
          }
          int jointIndex = joint.getJoints(i);
          if (!jointMap.containsKey(jointIndex)) {
            throw new IllegalArgumentException("Index " + jointIndex
                + " is not present in the joint map");
          }
          Joint djoint = jointMap.get(jointIndex);
          if (!(djoint instanceof DistanceJoint)) {
            throw new IllegalArgumentException(
                "Joints for constant volume joint must be distance joints");
          }
          def.addBodyAndJoint(argBodyMap.get(body), (DistanceJoint) djoint);
        }
        break;
      }
      case LINE: {
        UnsupportedObjectException e =
            new UnsupportedObjectException("Line joint no longer supported.", Type.JOINT);
        if (ulistener == null || ulistener.isUnsupported(e)) {
          throw e;
        }
        return null;
      }
      default: {
        UnsupportedObjectException e =
            new UnsupportedObjectException("Unknown joint type: " + joint.getType(), Type.JOINT);
        if (ulistener == null || ulistener.isUnsupported(e)) {
          throw e;
        }
        return null;
      }
    }

    jd.collideConnected = joint.getCollideConnected();

    if (!argBodyMap.containsKey(joint.getBodyA())) {
      throw new IllegalArgumentException("Index " + joint.getBodyA()
          + " is not present in the body map");
    }
    jd.bodyA = argBodyMap.get(joint.getBodyA());

    if (!argBodyMap.containsKey(joint.getBodyB())) {
      throw new IllegalArgumentException("Index " + joint.getBodyB()
          + " is not present in the body map");
    }
    jd.bodyB = argBodyMap.get(joint.getBodyB());

    Joint realJoint = argWorld.createJoint(jd);
    if (listener != null && joint.hasTag()) {
      listener.processJoint(realJoint, joint.getTag());
    }

    return realJoint;
  }

  private Vec2 pbToVec(PbVec2 argVec) {
    return new Vec2(argVec.getX(), argVec.getY());
  }

}
