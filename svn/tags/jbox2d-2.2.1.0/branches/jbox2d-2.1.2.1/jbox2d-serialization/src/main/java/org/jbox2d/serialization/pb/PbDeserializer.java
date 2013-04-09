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
import org.jbox2d.collision.shapes.CircleShape;
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
import org.jbox2d.dynamics.joints.LineJointDef;
import org.jbox2d.dynamics.joints.MouseJointDef;
import org.jbox2d.dynamics.joints.PrismaticJointDef;
import org.jbox2d.dynamics.joints.PulleyJointDef;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import org.jbox2d.dynamics.joints.WeldJointDef;
import org.jbox2d.serialization.JbDeserializer;
import org.jbox2d.serialization.UnsupportedListener;
import org.jbox2d.serialization.UnsupportedObjectException;
import org.jbox2d.serialization.UnsupportedObjectException.Type;

public class PbDeserializer implements JbDeserializer {

  private ObjectListener listener = null;
  private UnsupportedListener ulistener = null;

  public PbDeserializer() {
  }

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

  public World deserializeWorld(PbWorld argWorld) {
    World world = new World(pbToVec(argWorld.getGravity()), argWorld.getAllowSleep());

    world.setAutoClearForces(argWorld.getAutoClearForces());
    world.setContinuousPhysics(argWorld.getContinuousPhysics());
    world.setWarmStarting(argWorld.getWarmStarting());

    HashMap<Integer, Body> bodyMap = new HashMap<Integer, Body>();
    HashMap<Integer, Joint> jointMap = new HashMap<Integer, Joint>();

    for (int i = 0; i < argWorld.getBodiesCount(); i++) {
      PbBody pbBody = argWorld.getBodies(i);
      Body body = deserializeBody(world, pbBody);
      bodyMap.put(i, body);
    }

    // first pass, indep joints
    int cnt = 0;
    for (int i = 0; i < argWorld.getJointsCount(); i++) {
      PbJoint pbJoint = argWorld.getJoints(i);
      if (isIndependentJoint(pbJoint.getType())) {
        Joint joint = deserializeJoint(world, pbJoint, bodyMap, jointMap);
        jointMap.put(cnt, joint);
        cnt++;
      }
    }

    // second pass, dep joints
    for (int i = 0; i < argWorld.getJointsCount(); i++) {
      PbJoint pbJoint = argWorld.getJoints(i);
      if (!isIndependentJoint(pbJoint.getType())) {
        Joint joint = deserializeJoint(world, pbJoint, bodyMap, jointMap);
        jointMap.put(cnt, joint);
        cnt++;
      }
    }

    if (listener != null && argWorld.hasTag()) {
      listener.processWorld(world, argWorld.getTag());
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
    bd.active = b.getActive();
    bd.allowSleep = b.getAllowSleep();
    bd.angle = b.getAngle();
    bd.angularDamping = b.getAngularDamping();
    bd.angularVelocity = b.getAngularVelocity();
    bd.awake = b.getAwake();
    bd.bullet = b.getBullet();
    bd.fixedRotation = b.getFixedRotation();
    bd.linearDamping = b.getLinearDamping();
    bd.linearVelocity.set(pbToVec(b.getLinearVelocity()));
    bd.position.set(pbToVec(b.getPosition()));
    
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
        UnsupportedObjectException e = new UnsupportedObjectException("Unknown body type: "
            + argBody.getType(), Type.BODY);
        if (ulistener == null || ulistener.isUnsupported(e)) {
          throw e;
        }
        return null;
    }

    Body body = argWorld.createBody(bd);

    for (int i = 0; i < b.getFixturesCount(); i++) {
      deserializeFixture(body, b.getFixtures(i));
    }

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
    if (listener != null) {
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
        c.m_radius = s.getRadius();
        shape = c;
        break;
      case POLYGON:
        PolygonShape p = new PolygonShape();
        p.m_centroid.set(pbToVec(s.getCentroid()));
        p.m_radius = s.getRadius();
        p.m_vertexCount = s.getPointsCount();
        for (int i = 0; i < p.m_vertexCount; i++) {
          p.m_vertices[i].set(pbToVec(s.getPoints(i)));
          p.m_normals[i].set(pbToVec(s.getNormals(i)));
        }
        shape = p;
        break;
      case EDGE:
      case LOOP: {
        UnsupportedObjectException e = new UnsupportedObjectException(
            "Edge and loop shapes are not yet supported", Type.SHAPE);
        if (ulistener == null || ulistener.isUnsupported(e)) {
          throw e;
        }
        return null;
      }
      default: {
        UnsupportedObjectException e = new UnsupportedObjectException("Unknown shape type: "
            + s.getType(), Type.SHAPE);
        if (ulistener == null || ulistener.isUnsupported(e)) {
          throw e;
        }
        return null;
      }
    }

    if (listener != null) {
      listener.processShape(shape, s.getTag());
    }
    return shape;
  }

  @Override
  public Joint deserializeJoint(World argWorld, InputStream argInput,
      Map<Integer, Body> argBodyMap, Map<Integer, Joint> argJointMap) throws IOException {
    PbJoint joint = PbJoint.parseFrom(argInput);
    return deserializeJoint(argWorld, joint, argBodyMap, argJointMap);
  }

  public Joint deserializeJoint(World argWorld, PbJoint argJoint, Map<Integer, Body> argBodyMap,
      Map<Integer, Joint> argJointMap) {
    JointDef jd = null;

    switch (argJoint.getType()) {
      case PRISMATIC: {
        PrismaticJointDef def = new PrismaticJointDef();
        jd = def;
        def.enableLimit = argJoint.getEnableLimit();
        def.enableMotor = argJoint.getEnableMotor();
        def.localAnchorA.set(pbToVec(argJoint.getLocalAnchorA()));
        def.localAnchorB.set(pbToVec(argJoint.getLocalAnchorB()));
        def.localAxis1.set(pbToVec(argJoint.getLocalAxisA()));
        def.lowerTranslation = argJoint.getLowerLimit();
        def.maxMotorForce = argJoint.getMaxMotorForce();
        def.motorSpeed = argJoint.getMotorSpeed();
        def.referenceAngle = argJoint.getRefAngle();
        def.upperTranslation = argJoint.getUpperLimit();
        break;
      }
      case REVOLUTE: {
        RevoluteJointDef def = new RevoluteJointDef();
        jd = def;
        def.enableLimit = argJoint.getEnableLimit();
        def.enableMotor = argJoint.getEnableMotor();
        def.localAnchorA.set(pbToVec(argJoint.getLocalAnchorA()));
        def.localAnchorB.set(pbToVec(argJoint.getLocalAnchorB()));
        def.lowerAngle = argJoint.getLowerLimit();
        def.maxMotorTorque = argJoint.getMaxMotorTorque();
        def.motorSpeed = argJoint.getMotorSpeed();
        def.referenceAngle = argJoint.getRefAngle();
        def.upperAngle = argJoint.getUpperLimit();
        break;
      }
      case DISTANCE: {
        DistanceJointDef def = new DistanceJointDef();
        jd = def;
        def.localAnchorA.set(pbToVec(argJoint.getLocalAnchorA()));
        def.localAnchorB.set(pbToVec(argJoint.getLocalAnchorB()));
        def.dampingRatio = argJoint.getDampingRatio();
        def.frequencyHz = argJoint.getFrequency();
        def.length = argJoint.getLength();
        break;
      }
      case PULLEY: {
        PulleyJointDef def = new PulleyJointDef();
        jd = def;
        def.localAnchorA.set(pbToVec(argJoint.getLocalAnchorA()));
        def.localAnchorB.set(pbToVec(argJoint.getLocalAnchorB()));
        def.groundAnchorA.set(pbToVec(argJoint.getGroundAnchorA()));
        def.groundAnchorB.set(pbToVec(argJoint.getGroundAnchorB()));
        def.lengthA = argJoint.getLengthA();
        def.lengthB = argJoint.getLengthB();
        def.maxLengthA = argJoint.getMaxLengthA();
        def.maxLengthB = argJoint.getMaxLengthB();
        def.ratio = argJoint.getRatio();
        break;
      }
      case MOUSE: {
        MouseJointDef def = new MouseJointDef();
        jd = def;
        def.dampingRatio = argJoint.getDampingRatio();
        def.frequencyHz = argJoint.getFrequency();
        def.maxForce = argJoint.getMaxForce();
        def.target.set(pbToVec(argJoint.getTarget()));
        break;
      }
      case GEAR: {
        GearJointDef def = new GearJointDef();
        jd = def;
        if (!argJointMap.containsKey(argJoint.getJoint1())) {
          throw new IllegalArgumentException("Index " + argJoint.getJoint1()
              + " is not present in the joint map.");
        }
        def.joint1 = argJointMap.get(argJoint.getJoint1());
        if (!argJointMap.containsKey(argJoint.getJoint2())) {
          throw new IllegalArgumentException("Index " + argJoint.getJoint2()
              + " is not present in the joint map.");
        }
        def.joint2 = argJointMap.get(argJoint.getJoint2());
        def.ratio = argJoint.getRatio();
        break;
      }
      case WHEEL: {
        UnsupportedObjectException e = new UnsupportedObjectException(
            "Wheel joint not supported yet.", Type.JOINT);
        if (ulistener == null || ulistener.isUnsupported(e)) {
          throw e;
        }
        return null;
      }
      case WELD: {
        WeldJointDef def = new WeldJointDef();
        jd = def;
        def.localAnchorA.set(pbToVec(argJoint.getLocalAnchorA()));
        def.localAnchorB.set(pbToVec(argJoint.getLocalAnchorB()));
        def.referenceAngle = argJoint.getRefAngle();
        break;
      }
      case FRICTION: {
        FrictionJointDef def = new FrictionJointDef();
        jd = def;
        def.localAnchorA.set(pbToVec(argJoint.getLocalAnchorA()));
        def.localAnchorB.set(pbToVec(argJoint.getLocalAnchorB()));
        def.maxForce = argJoint.getMaxForce();
        def.maxTorque = argJoint.getMaxTorque();
        break;
      }
      case ROPE: {
        UnsupportedObjectException e = new UnsupportedObjectException(
            "Rope joint not supported yet.", Type.JOINT);
        if (ulistener == null || ulistener.isUnsupported(e)) {
          throw e;
        }
        return null;
      }
      case CONSTANT_VOLUME: {
        ConstantVolumeJointDef def = new ConstantVolumeJointDef();
        jd = def;
        def.dampingRatio = argJoint.getDampingRatio();
        def.frequencyHz = argJoint.getFrequency();
        if (argJoint.getBodiesCount() != argJoint.getJointsCount()) {
          throw new IllegalArgumentException(
              "Constant volume joint must have bodies and joints defined");
        }
        for (int i = 0; i < argJoint.getBodiesCount(); i++) {
          int body = argJoint.getBodies(i);
          if (!argBodyMap.containsKey(body)) {
            throw new IllegalArgumentException("Index " + body + " is not present in the body map");
          }
          int joint = argJoint.getJoints(i);
          if (!argJointMap.containsKey(joint)) {
            throw new IllegalArgumentException("Index " + joint
                + " is not present in the joint map");
          }
          Joint djoint = argJointMap.get(joint);
          if (!(djoint instanceof DistanceJoint)) {
            throw new IllegalArgumentException(
                "Joints for constant volume joint must be distance joints");
          }
          def.addBodyAndJoint(argBodyMap.get(body), (DistanceJoint) djoint);
        }
        break;
      }
      case LINE: {
        LineJointDef def = new LineJointDef();
        jd = def;
        def.localAnchorA.set(pbToVec(argJoint.getLocalAnchorA()));
        def.localAnchorB.set(pbToVec(argJoint.getLocalAnchorB()));
        def.localAxisA.set(pbToVec(argJoint.getLocalAxisA()));
        def.enableLimit = argJoint.getEnableLimit();
        def.enableMotor = argJoint.getEnableMotor();
        def.lowerTranslation = argJoint.getLowerLimit();
        def.upperTranslation = argJoint.getUpperLimit();
        def.maxMotorForce = argJoint.getMaxForce();
        def.motorSpeed = argJoint.getMotorSpeed();
        break;
      }
      default: {
        UnsupportedObjectException e = new UnsupportedObjectException("Unknown joint type: "
            + argJoint.getType(), Type.JOINT);
        if (ulistener == null || ulistener.isUnsupported(e)) {
          throw e;
        }
        return null;
      }
    }

    jd.collideConnected = argJoint.getCollideConnected();

    if (!argBodyMap.containsKey(argJoint.getBodyA())) {
      throw new IllegalArgumentException("Index " + argJoint.getBodyA()
          + " is not present in the body map");
    }
    jd.bodyA = argBodyMap.get(argJoint.getBodyA());

    if (!argBodyMap.containsKey(argJoint.getBodyB())) {
      throw new IllegalArgumentException("Index " + argJoint.getBodyB()
          + " is not present in the body map");
    }
    jd.bodyB = argBodyMap.get(argJoint.getBodyB());

    Joint joint = argWorld.createJoint(jd);
    if (listener != null && argJoint.hasTag()) {
      listener.processJoint(joint, argJoint.getTag());
    }

    return joint;
  }

  private Vec2 pbToVec(PbVec2 argVec) {
    return new Vec2(argVec.getX(), argVec.getY());
  }

}
