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
import org.jbox2d.dynamics.joints.DistanceJoint;
import org.jbox2d.dynamics.joints.FrictionJoint;
import org.jbox2d.dynamics.joints.GearJoint;
import org.jbox2d.dynamics.joints.Joint;
import org.jbox2d.dynamics.joints.MouseJoint;
import org.jbox2d.dynamics.joints.PrismaticJoint;
import org.jbox2d.dynamics.joints.PulleyJoint;
import org.jbox2d.dynamics.joints.RevoluteJoint;
import org.jbox2d.proto.Box2D.PbBody;
import org.jbox2d.proto.Box2D.PbBodyType;
import org.jbox2d.proto.Box2D.PbFilter;
import org.jbox2d.proto.Box2D.PbFixture;
import org.jbox2d.proto.Box2D.PbJoint;
import org.jbox2d.proto.Box2D.PbJointType;
import org.jbox2d.proto.Box2D.PbShape;
import org.jbox2d.proto.Box2D.PbShapeType;
import org.jbox2d.proto.Box2D.PbVec2;
import org.jbox2d.proto.Box2D.PbWorld;
import org.jbox2d.serialization.JbSerializer;
import org.jbox2d.serialization.SerializationHelper;
import org.jbox2d.serialization.SerializationResult;

/**
 * Protobuffer serializer implementation.
 * @author Daniel
 *
 */
public class PbSerializer implements JbSerializer {
	
	private ObjectSigner signer = null;
	
	public PbSerializer(){}
	
	public PbSerializer(ObjectSigner argSigner){
		signer = argSigner;
	}

	@Override
	public void setObjectSigner(ObjectSigner argSigner) {
		signer = argSigner;
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
		if(signer != null){
			Integer id = signer.getId(argWorld);
			if(id != null){
				builder.setId(id);
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
		
		while(cbody != null){
			builder.addBodies(serializeBody(cbody));
			bodies.put(cbody, cnt);
			cnt++;
			cbody = cbody.m_next;
		}
		cnt = 0;
		HashMap<Joint, Integer> joints = new HashMap<Joint, Integer>();
		Joint cjoint = argWorld.getJointList();
		// first pass
		while(cjoint != null){
			if(SerializationHelper.isIndependentJoint(cjoint.getType())){
				builder.addJoints(serializeIndepJoint(
						cjoint, bodies));
				joints.put(cjoint, cnt);
				cnt++;
			}
			cjoint = cjoint.m_next;
		}
		
		// second pass
		cjoint = argWorld.getJointList();
		while(cjoint != null){
			if(!SerializationHelper.isIndependentJoint(cjoint.getType())){
				builder.addJoints(serializeDependJoint(
						cjoint, bodies, joints));
				cnt++;
			}
			cjoint = cjoint.m_next;
		}
		
		return builder;
	}

	@Override
	public SerializationResult serialize(Body argBody) {
		final PbBody body = serializeBody(argBody).build();
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
		if(signer != null){
			Integer id = signer.getId(argBody);
			if(id != null){
				builder.setId(id);
			}
		}
		switch(argBody.getType()){
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
				throw new IllegalArgumentException(
						"Unknown body type: "+argBody.getType());
		}
		builder.setPosition(vecToPb(argBody.getPosition()));
		builder.setAngle(argBody.getAngle());
		builder.setLinearVelocity(vecToPb(argBody.getLinearVelocity()));
		builder.setAngularVelocity(argBody.getAngularVelocity());
		builder.setLinearDamping(argBody.getLinearDamping());
		builder.setAngularDamping(argBody.getAngularDamping());
		//TODO
		//builder.setGravityScale(argBody.getGravityScale());
		builder.setBullet(argBody.isBullet());
		builder.setAllowSleep(argBody.isSleepingAllowed());
		builder.setAwake(argBody.isAwake());
		builder.setActive(argBody.isActive());
		builder.setFixedRotation(argBody.isFixedRotation());
		
		builder.setMass(argBody.m_mass);
		builder.setI(argBody.m_I);
		
		Fixture curr = argBody.m_fixtureList;
		while(curr != null){
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
		if(signer != null){
			Integer id = signer.getId(argFixture);
			if(id != null){
				builder.setId(id);
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
		// should we do lazy building?
		final PbShape shape = serializeShape(argShape).build();
		return new SerializationResult() {
			@Override
			public void writeTo(OutputStream argOutputStream) throws IOException{
				shape.writeTo(argOutputStream);
			}
			@Override
			public Object getValue() {
				return shape;
			}
		};
	}
	
	public PbShape.Builder serializeShape(Shape argShape){
		final PbShape.Builder builder = PbShape.newBuilder();
		builder.setRadius(argShape.m_radius);
		
		switch(argShape.m_type){
			case CIRCLE:
				CircleShape c = (CircleShape) argShape;
				builder.setCenter(vecToPb(c.m_p));
				builder.setType(PbShapeType.CIRCLE);
				break;
			case POLYGON:
				PolygonShape p = (PolygonShape) argShape;
				builder.setCentroid(vecToPb(p.m_centroid));
				builder.setType(PbShapeType.POLYGON);
				for(int i=0; i<p.m_vertexCount; i++){
					builder.addPoints(vecToPb(p.m_vertices[i]));
					builder.addNormals(vecToPb(p.m_normals[i]));
				}
				break;
			default:
				throw new IllegalArgumentException(
						"Currently only encodes circle and polygon shapes");
		}
		
		return builder;
	}
	
	@Override
	public SerializationResult serialize(Joint argJoint,
			Map<Body, Integer> argBodyIndexMap) {
		final PbJoint joint = serializeIndepJoint(argJoint, argBodyIndexMap).build();
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
	
	@Override
	public SerializationResult serialize(Joint argJoint,
			Map<Body, Integer> argBodyIndexMap,
			Map<Joint, Integer> argJointIndexMap) {
		final PbJoint joint = serializeDependJoint(argJoint,
				argBodyIndexMap, argJointIndexMap).build();
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
	
	public PbJoint.Builder serializeDependJoint(Joint argJoint,
			Map<Body, Integer> argBodyIndexMap,
			Map<Joint, Integer> argJointIndexMap){
		final PbJoint.Builder builder = PbJoint.newBuilder();
		if(signer != null){
			Integer id = signer.getId(argJoint);
			if(id != null){
				builder.setId(id);
			}
		}
		Body bA = argJoint.m_bodyA;
		Body bB = argJoint.m_bodyB;
		
		if(bA != null){
			if(!argBodyIndexMap.containsKey(bA)){
				throw new IllegalArgumentException("Body " + bA +
						" is not present in the index map");
			}
			builder.setBodyA(argBodyIndexMap.get(bA));
		}
		
		if(bB != null){
			if(!argBodyIndexMap.containsKey(bB)){
				throw new IllegalArgumentException("Body " + bB +
						" is not present in the index map");
			}
			builder.setBodyB(argBodyIndexMap.get(bB));
		}
		
		builder.setCollideConnected(argJoint.m_collideConnected);
		
		switch(argJoint.getType()){
			case GEAR:{
				GearJoint j = (GearJoint) argJoint;
				builder.setType(PbJointType.GEAR);
				builder.setAnchorA(vecToPb(j.m_localAnchor1));
				builder.setAnchorB(vecToPb(j.m_localAnchor2));
				builder.setRatio(j.getRatio());
				if(!argJointIndexMap.containsKey(j.getJoint1())){
					throw new IllegalArgumentException("Joint 1 not in map");
				}
				int j1 = argJointIndexMap.get(j.getJoint1());
				if(!argJointIndexMap.containsKey(j.getJoint2())){
					throw new IllegalArgumentException("Joint 2 not in map");
				}
				int j2 = argJointIndexMap.get(j.getJoint2());
				
				builder.setJoint1(j1);
				builder.setJoint2(j2);
			} break;
			default:
				throw new IllegalArgumentException(
						"Unknown dependant joint type: " + argJoint.getType());
		}
		
		return builder;
	}

	public PbJoint.Builder serializeIndepJoint(Joint argJoint,
			Map<Body, Integer> argBodyIndexMap) {
		final PbJoint.Builder builder = PbJoint.newBuilder();
		if(signer != null){
			Integer id = signer.getId(argJoint);
			if(id != null){
				builder.setId(id);
			}
		}
		Body bA = argJoint.m_bodyA;
		Body bB = argJoint.m_bodyB;
		
		if(bA != null){
			if(!argBodyIndexMap.containsKey(bA)){
				throw new IllegalArgumentException("Body " + bA +
						" is not present in the index map");
			}
			builder.setBodyA(argBodyIndexMap.get(bA));
		}
		
		if(bB != null){
			if(!argBodyIndexMap.containsKey(bB)){
				throw new IllegalArgumentException("Body " + bB +
						" is not present in the index map");
			}
			builder.setBodyB(argBodyIndexMap.get(bB));
		}
		
		builder.setCollideConnected(argJoint.m_collideConnected);
		
		switch(argJoint.m_type){
			case REVOLUTE:{
				RevoluteJoint j = (RevoluteJoint) argJoint;
				builder.setType(PbJointType.REVOLUTE);
				builder.setRefAngle(j.m_referenceAngle);
				builder.setEnableLimit(j.m_enableLimit);
				builder.setLowerLimit(j.m_lowerAngle);
				builder.setUpperLimit(j.m_upperAngle);
				builder.setEnableMotor(j.m_enableMotor);
				builder.setMotorSpeed(j.m_motorSpeed);
				builder.setMaxMotorTorque(j.m_maxMotorTorque);
				builder.setAnchorA(vecToPb(j.m_localAnchor1));
				builder.setAnchorB(vecToPb(j.m_localAnchor2));
			} break;
			case PRISMATIC: {
				PrismaticJoint j = (PrismaticJoint) argJoint;
				builder.setType(PbJointType.PRISMATIC);
				builder.setRefAngle(j.m_refAngle);
				builder.setEnableLimit(j.m_enableLimit);
				builder.setLowerLimit(j.m_lowerTranslation);
				builder.setUpperLimit(j.m_upperTranslation);
				builder.setEnableMotor(j.m_enableMotor);
				builder.setMotorSpeed(j.m_motorSpeed);
				builder.setAnchorA(vecToPb(j.m_localAnchor1));
				builder.setAnchorB(vecToPb(j.m_localAnchor2));
				builder.setLocalAxisA(vecToPb(j.m_localXAxis1));
				builder.setMaxMotorForce(j.m_maxMotorForce);
			} break;
			case DISTANCE: {
				DistanceJoint j = (DistanceJoint) argJoint;
				builder.setType(PbJointType.DISTANCE);
				builder.setAnchorA(vecToPb(j.m_localAnchor1));
				builder.setAnchorB(vecToPb(j.m_localAnchor2));
				builder.setLength(j.m_length);
				builder.setFrequency(j.m_frequencyHz);
				builder.setDampingRatio(j.m_dampingRatio);
			} break;
			case PULLEY: {
				PulleyJoint j = (PulleyJoint) argJoint;
				builder.setType(PbJointType.PULLEY);
				builder.setAnchorA(vecToPb(j.m_localAnchor1));
				builder.setAnchorB(vecToPb(j.m_localAnchor2));
				builder.setGroundAnchorA(vecToPb(j.m_groundAnchor1));
				builder.setGroundAnchorB(vecToPb(j.m_groundAnchor2));
				builder.setLengthA(j.getOrigLength1());
				builder.setLengthB(j.getOrigLength2());
				builder.setRatio(j.getRatio());
			} break;
			case MOUSE: {
				MouseJoint j = (MouseJoint) argJoint;
				builder.setType(PbJointType.MOUSE);
				builder.setTarget(vecToPb(j.getTarget()));
				builder.setMaxForce(j.getMaxForce());
				builder.setFrequency(j.getFrequency());
				builder.setDampingRatio(j.getDampingRatio());
			} break;
			case GEAR: {
				//GearJoint j = (GearJoint) argJoint;
				// dep joint
				throw new IllegalArgumentException("Gear joint not serialized in this method. " +
							"You must use dependant joint serialization method.");
			}
			case FRICTION: {
				FrictionJoint j = (FrictionJoint) argJoint;
				builder.setType(PbJointType.FRICTION);
				builder.setAnchorA(vecToPb(j.getLocalAnchorA()));
				builder.setAnchorB(vecToPb(j.getLocalAnchorB()));
				builder.setMaxForce(j.getMaxForce());
				builder.setMaxTorque(j.getMaxTorque());
			} break;
			default:
				throw new IllegalArgumentException(
						"Unknown joint type: "+argJoint.getType());
		}
		return builder;
	}
	
	public PbFilter.Builder serializeFilter(Filter argFilter){
		PbFilter.Builder builder = PbFilter.newBuilder();
		builder.setCategoryBits(argFilter.categoryBits);
		builder.setGroupIndex(argFilter.groupIndex);
		builder.setMaskBits(argFilter.maskBits);
		
		return builder;
	}
	
	private PbVec2 vecToPb(Vec2 argVec){
		if(argVec == null){
			return null;
		}
		return PbVec2.newBuilder().setX(argVec.x).setY(argVec.y).build();
	}
}
