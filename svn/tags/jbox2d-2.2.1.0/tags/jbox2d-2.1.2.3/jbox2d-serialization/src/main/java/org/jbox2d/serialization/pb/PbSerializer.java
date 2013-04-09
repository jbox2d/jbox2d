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
import java.io.OutputStream;
import java.util.HashMap;
import java.util.Map;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.Filter;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.joints.ConstantVolumeJoint;
import org.jbox2d.dynamics.joints.DistanceJoint;
import org.jbox2d.dynamics.joints.FrictionJoint;
import org.jbox2d.dynamics.joints.GearJoint;
import org.jbox2d.dynamics.joints.Joint;
import org.jbox2d.dynamics.joints.LineJoint;
import org.jbox2d.dynamics.joints.MouseJoint;
import org.jbox2d.dynamics.joints.PrismaticJoint;
import org.jbox2d.dynamics.joints.PulleyJoint;
import org.jbox2d.dynamics.joints.RevoluteJoint;
import org.box2d.proto.Box2D.PbBody;
import org.box2d.proto.Box2D.PbBodyType;
import org.box2d.proto.Box2D.PbFilter;
import org.box2d.proto.Box2D.PbFixture;
import org.box2d.proto.Box2D.PbJoint;
import org.box2d.proto.Box2D.PbJointType;
import org.box2d.proto.Box2D.PbShape;
import org.box2d.proto.Box2D.PbShapeType;
import org.box2d.proto.Box2D.PbVec2;
import org.box2d.proto.Box2D.PbWorld;
import org.jbox2d.serialization.JbSerializer;
import org.jbox2d.serialization.SerializationHelper;
import org.jbox2d.serialization.SerializationResult;
import org.jbox2d.serialization.UnsupportedListener;
import org.jbox2d.serialization.UnsupportedObjectException;
import org.jbox2d.serialization.UnsupportedObjectException.Type;

/**
 * Protobuffer serializer implementation.
 * 
 * @author Daniel
 * 
 */
public class PbSerializer implements JbSerializer {

  private ObjectSigner signer = null;
  private UnsupportedListener listener = null;

  public PbSerializer() {
  }

  public PbSerializer(UnsupportedListener argListener) {
    listener = argListener;
  }

  public PbSerializer(ObjectSigner argSigner) {
    signer = argSigner;
  }

  public PbSerializer(UnsupportedListener argListener, ObjectSigner argSigner) {
    listener = argListener;
    signer = argSigner;
  }

  @Override
  public void setObjectSigner(ObjectSigner argSigner) {
    signer = argSigner;
  }

  @Override
  public void setUnsupportedListener(UnsupportedListener argListener) {
    listener = argListener;
  }

  @Override
  public SerializationResult serialize(World argWorld) {
    final PbWorld world = serializeWorld(argWorld).build();
    return new SerializationResult() {
      @Override
      public void writeTo(OutputStream argOutputStream) throws IOException {
        world.writeTo(argOutputStream);
      }

      @Override
      public Object getValue() {
        return world;
      }
    };
  }

  public PbWorld.Builder serializeWorld(World argWorld) {
    final PbWorld.Builder builder = PbWorld.newBuilder();
    if (signer != null) {
      Long tag = signer.getTag(argWorld);
      if (tag != null) {
        builder.setTag(tag);
      }
    }

    builder.setGravity(vecToPb(argWorld.getGravity()));
    builder.setAllowSleep(argWorld.isAllowSleep());
    builder.setContinuousPhysics(argWorld.isContinuousPhysics());
    builder.setWarmStarting(argWorld.isWarmStarting());
    // TODO(dmurph): substepping

    Body cbody = argWorld.getBodyList();
    int cnt = 0;
    HashMap<Body, Integer> bodies = new HashMap<Body, Integer>();

    while (cbody != null) {
      builder.addBodies(serializeBody(cbody));
      bodies.put(cbody, cnt);
      cnt++;
      cbody = cbody.m_next;
    }
    cnt = 0;
    HashMap<Joint, Integer> joints = new HashMap<Joint, Integer>();
    Joint cjoint = argWorld.getJointList();
    // first pass
    while (cjoint != null) {
      if (SerializationHelper.isIndependentJoint(cjoint.getType())) {
        builder.addJoints(serializeJoint(cjoint, bodies, joints));
        joints.put(cjoint, cnt);
        cnt++;
      }
      cjoint = cjoint.m_next;
    }

    // second pass for dependent joints
    cjoint = argWorld.getJointList();
    while (cjoint != null) {
      if (!SerializationHelper.isIndependentJoint(cjoint.getType())) {
        builder.addJoints(serializeJoint(cjoint, bodies, joints));
        joints.put(cjoint, cnt);
        cnt++;
      }
      cjoint = cjoint.m_next;
    }

    return builder;
  }

  @Override
  public SerializationResult serialize(Body argBody) {
    PbBody.Builder builder = serializeBody(argBody);
    if (builder == null) {
      return null;
    }
    final PbBody body = builder.build();
    return new SerializationResult() {
      @Override
      public void writeTo(OutputStream argOutputStream) throws IOException {
        body.writeTo(argOutputStream);
      }

      @Override
      public Object getValue() {
        return body;
      }
    };
  }

  public PbBody.Builder serializeBody(Body argBody) {
    PbBody.Builder builder = PbBody.newBuilder();
    if (signer != null) {
      Long id = signer.getTag(argBody);
      if (id != null) {
        builder.setTag(id);
      }
    }
    switch (argBody.getType()) {
      case DYNAMIC:
        builder.setType(PbBodyType.DYNAMIC);
        break;
      case KINEMATIC:
        builder.setType(PbBodyType.KINEMATIC);
        break;
      case STATIC:
        builder.setType(PbBodyType.STATIC);
        break;
      default:
        UnsupportedObjectException e = new UnsupportedObjectException("Unknown body type: "
            + argBody.getType(), Type.BODY);
        if (listener == null || listener.isUnsupported(e)) {
          throw e;
        }
        return null;
    }
    builder.setPosition(vecToPb(argBody.getPosition()));
    builder.setAngle(argBody.getAngle());
    builder.setLinearVelocity(vecToPb(argBody.getLinearVelocity()));
    builder.setAngularVelocity(argBody.getAngularVelocity());
    builder.setLinearDamping(argBody.getLinearDamping());
    builder.setAngularDamping(argBody.getAngularDamping());
    // TODO
    // builder.setGravityScale(argBody.getGravityScale());

    builder.setForce(vecToPb(argBody.m_force));
    builder.setTorque(argBody.m_torque);

    builder.setBullet(argBody.isBullet());
    builder.setAllowSleep(argBody.isSleepingAllowed());
    builder.setAwake(argBody.isAwake());
    builder.setActive(argBody.isActive());
    builder.setFixedRotation(argBody.isFixedRotation());

    builder.setMass(argBody.m_mass);
    builder.setI(argBody.m_I);

    Fixture curr = argBody.m_fixtureList;
    while (curr != null) {
      builder.addFixtures(serializeFixture(curr));
      curr = curr.m_next;
    }

    return builder;
  }

  @Override
  public SerializationResult serialize(Fixture argFixture) {
    final PbFixture fixture = serializeFixture(argFixture).build();
    return new SerializationResult() {
      @Override
      public void writeTo(OutputStream argOutputStream) throws IOException {
        fixture.writeTo(argOutputStream);
      }

      @Override
      public Object getValue() {
        return fixture;
      }
    };
  }

  public PbFixture.Builder serializeFixture(Fixture argFixture) {
    final PbFixture.Builder builder = PbFixture.newBuilder();
    if (signer != null) {
      Long tag = signer.getTag(argFixture);
      if (tag != null) {
        builder.setTag(tag);
      }
    }

    builder.setDensity(argFixture.m_density);
    builder.setFriction(argFixture.m_friction);
    builder.setRestitution(argFixture.m_restitution);
    builder.setSensor(argFixture.m_isSensor);

    builder.setShape(serializeShape(argFixture.m_shape));

    builder.setFilter(serializeFilter(argFixture.m_filter));
    return builder;
  }

  @Override
  public SerializationResult serialize(Shape argShape) {
    PbShape.Builder builder = serializeShape(argShape);
    if (builder == null) {
      return null;
    }
    // should we do lazy building?
    final PbShape shape = builder.build();
    return new SerializationResult() {
      @Override
      public void writeTo(OutputStream argOutputStream) throws IOException {
        shape.writeTo(argOutputStream);
      }

      @Override
      public Object getValue() {
        return shape;
      }
    };
  }

  public PbShape.Builder serializeShape(Shape argShape) {
    final PbShape.Builder builder = PbShape.newBuilder();
    if (signer != null) {
      Long tag = signer.getTag(argShape);
      if (tag != null) {
        builder.setTag(tag);
      }
    }
    builder.setRadius(argShape.m_radius);

    switch (argShape.m_type) {
      case CIRCLE:
        CircleShape c = (CircleShape) argShape;
        builder.setCenter(vecToPb(c.m_p));
        builder.setType(PbShapeType.CIRCLE);
        break;
      case POLYGON:
        PolygonShape p = (PolygonShape) argShape;
        builder.setCentroid(vecToPb(p.m_centroid));
        builder.setType(PbShapeType.POLYGON);
        for (int i = 0; i < p.m_vertexCount; i++) {
          builder.addPoints(vecToPb(p.m_vertices[i]));
          builder.addNormals(vecToPb(p.m_normals[i]));
        }
        break;
      default:
        UnsupportedObjectException e = new UnsupportedObjectException(
            "Currently only encodes circle and polygon shapes", Type.SHAPE);
        if (listener == null || listener.isUnsupported(e)) {
          throw e;
        }
        return null;
    }

    return builder;
  }

  @Override
  public SerializationResult serialize(Joint argJoint, Map<Body, Integer> argBodyIndexMap,
      Map<Joint, Integer> argJointIndexMap) {
    PbJoint.Builder builder = serializeJoint(argJoint, argBodyIndexMap, argJointIndexMap);
    if (builder == null) {
      return null;
    }
    final PbJoint joint = builder.build();
    return new SerializationResult() {
      @Override
      public void writeTo(OutputStream argOutputStream) throws IOException {
        joint.writeTo(argOutputStream);
      }

      @Override
      public Object getValue() {
        return joint;
      }
    };
  }

  public PbJoint.Builder serializeJoint(Joint argJoint, Map<Body, Integer> argBodyIndexMap,
      Map<Joint, Integer> argJointIndexMap) {
    final PbJoint.Builder builder = PbJoint.newBuilder();
    if (signer != null) {
      Long tag = signer.getTag(argJoint);
      if (tag != null) {
        builder.setTag(tag);
      }
    }
    Body bA = argJoint.m_bodyA;
    Body bB = argJoint.m_bodyB;

    if (!argBodyIndexMap.containsKey(bA)) {
      throw new IllegalArgumentException("Body " + bA + " is not present in the index map");
    }
    builder.setBodyA(argBodyIndexMap.get(bA));

    if (!argBodyIndexMap.containsKey(bB)) {
      throw new IllegalArgumentException("Body " + bB + " is not present in the index map");
    }
    builder.setBodyB(argBodyIndexMap.get(bB));

    builder.setCollideConnected(argJoint.m_collideConnected);

    switch (argJoint.m_type) {
      case REVOLUTE: {
        RevoluteJoint j = (RevoluteJoint) argJoint;
        builder.setType(PbJointType.REVOLUTE);
        builder.setRefAngle(j.m_referenceAngle);
        builder.setEnableLimit(j.m_enableLimit);
        builder.setLowerLimit(j.m_lowerAngle);
        builder.setUpperLimit(j.m_upperAngle);
        builder.setEnableMotor(j.m_enableMotor);
        builder.setMotorSpeed(j.m_motorSpeed);
        builder.setMaxMotorTorque(j.m_maxMotorTorque);
        builder.setLocalAnchorA(vecToPb(j.m_localAnchor1));
        builder.setLocalAnchorB(vecToPb(j.m_localAnchor2));
        break;
      }
      case PRISMATIC: {
        PrismaticJoint j = (PrismaticJoint) argJoint;
        builder.setType(PbJointType.PRISMATIC);
        builder.setRefAngle(j.m_refAngle);
        builder.setEnableLimit(j.m_enableLimit);
        builder.setLowerLimit(j.m_lowerTranslation);
        builder.setUpperLimit(j.m_upperTranslation);
        builder.setEnableMotor(j.m_enableMotor);
        builder.setMotorSpeed(j.m_motorSpeed);
        builder.setLocalAnchorA(vecToPb(j.m_localAnchor1));
        builder.setLocalAnchorB(vecToPb(j.m_localAnchor2));
        builder.setLocalAxisA(vecToPb(j.m_localXAxis1));
        builder.setMaxMotorForce(j.m_maxMotorForce);
        break;
      }
      case DISTANCE: {
        DistanceJoint j = (DistanceJoint) argJoint;
        builder.setType(PbJointType.DISTANCE);
        builder.setLocalAnchorA(vecToPb(j.m_localAnchor1));
        builder.setLocalAnchorB(vecToPb(j.m_localAnchor2));
        builder.setLength(j.m_length);
        builder.setFrequency(j.m_frequencyHz);
        builder.setDampingRatio(j.m_dampingRatio);
        break;
      }
      case PULLEY: {
        PulleyJoint j = (PulleyJoint) argJoint;
        builder.setType(PbJointType.PULLEY);
        builder.setLocalAnchorA(vecToPb(j.m_localAnchor1));
        builder.setLocalAnchorB(vecToPb(j.m_localAnchor2));
        builder.setGroundAnchorA(vecToPb(j.m_groundAnchor1));
        builder.setGroundAnchorB(vecToPb(j.m_groundAnchor2));
        builder.setLengthA(j.getOrigLength1());
        builder.setLengthB(j.getOrigLength2());
        builder.setMaxLengthA(j.getMaxLength1());
        builder.setMaxLengthB(j.getMaxLength2());
        builder.setRatio(j.getRatio());
        break;
      }
      case MOUSE: {
        MouseJoint j = (MouseJoint) argJoint;
        builder.setType(PbJointType.MOUSE);
        builder.setTarget(vecToPb(j.getTarget()));
        builder.setMaxForce(j.getMaxForce());
        builder.setFrequency(j.getFrequency());
        builder.setDampingRatio(j.getDampingRatio());
        break;
      }
      case GEAR: {
        GearJoint j = (GearJoint) argJoint;
        builder.setType(PbJointType.GEAR);
        builder.setLocalAnchorA(vecToPb(j.m_localAnchor1));
        builder.setLocalAnchorB(vecToPb(j.m_localAnchor2));
        builder.setRatio(j.getRatio());
        if (!argJointIndexMap.containsKey(j.getJoint1())) {
          throw new IllegalArgumentException("Joint 1 not in map");
        }
        int j1 = argJointIndexMap.get(j.getJoint1());
        if (!argJointIndexMap.containsKey(j.getJoint2())) {
          throw new IllegalArgumentException("Joint 2 not in map");
        }
        int j2 = argJointIndexMap.get(j.getJoint2());

        builder.setJoint1(j1);
        builder.setJoint2(j2);
        break;
      }
      case FRICTION: {
        FrictionJoint j = (FrictionJoint) argJoint;
        builder.setType(PbJointType.FRICTION);
        builder.setLocalAnchorA(vecToPb(j.getLocalAnchorA()));
        builder.setLocalAnchorB(vecToPb(j.getLocalAnchorB()));
        builder.setMaxForce(j.getMaxForce());
        builder.setMaxTorque(j.getMaxTorque());
        break;
      }
      case CONSTANT_VOLUME: {
        ConstantVolumeJoint j = (ConstantVolumeJoint) argJoint;
        builder.setType(PbJointType.CONSTANT_VOLUME);
        builder.setFrequency(j.frequencyHz);
        builder.setDampingRatio(j.dampingRatio);
        builder.setTargetVolume(j.targetVolume);

        for (int i = 0; i < j.bodies.length; i++) {
          Body b = j.bodies[i];
          DistanceJoint djoint = j.getJoints()[i];
          if (!argBodyIndexMap.containsKey(b)) {
            throw new IllegalArgumentException("Body " + b + " is not present in the index map");
          }
          builder.addBodies(argBodyIndexMap.get(b));
          if (!argJointIndexMap.containsKey(djoint)) {
            throw new IllegalArgumentException("Joint " + djoint
                + " is not present in the index map");
          }
          builder.addJoints(argJointIndexMap.get(djoint));
        }
        break;
      }
      case LINE: {
        LineJoint j = (LineJoint) argJoint;
        builder.setType(PbJointType.LINE);
        builder.setLocalAnchorA(vecToPb(j.m_localAnchor1));
        builder.setLocalAnchorB(vecToPb(j.m_localAnchor2));
        builder.setLocalAxisA(vecToPb(j.m_localXAxis1));
        builder.setLowerLimit(j.getLowerLimit());
        builder.setUpperLimit(j.getUpperLimit());
        builder.setMaxMotorForce(j.getMaxMotorForce());
        builder.setMotorSpeed(j.getMotorSpeed());
        builder.setEnableLimit(j.isLimitEnabled());
        builder.setEnableMotor(j.isMotorEnabled());
        break;
      }
      default:
        UnsupportedObjectException e = new UnsupportedObjectException("Unknown joint type: "
            + argJoint.getType(), Type.JOINT);
        if (listener == null || listener.isUnsupported(e)) {
          throw e;
        }
        return null;
    }
    return builder;
  }

  public PbFilter.Builder serializeFilter(Filter argFilter) {
    PbFilter.Builder builder = PbFilter.newBuilder();
    builder.setCategoryBits(argFilter.categoryBits);
    builder.setGroupIndex(argFilter.groupIndex);
    builder.setMaskBits(argFilter.maskBits);

    return builder;
  }

  private PbVec2 vecToPb(Vec2 argVec) {
    if (argVec == null) {
      return null;
    }
    return PbVec2.newBuilder().setX(argVec.x).setY(argVec.y).build();
  }
}
