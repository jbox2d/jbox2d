package org.jbox2d.testbed.tests;

import org.jbox2d.collision.CircleDef;
import org.jbox2d.collision.PolygonDef;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.joints.RevoluteJointDef;

/**
 * Ragdoll class thanks to darkzerox
 */
public class BipedDef {
	// Usually using a static public count variable is unsafe if we're
	// using something in an applet, because it will not be re-initialized upon
	// reload.  However, here it's only used to set the group index, so the
	// important thing is just that it's the same throughout each Biped and
	// different between multiple ones.
	static public int count = 0;
	
	// This was constant in C++, but I think scale should probably be settable
	static float k_scale = 3.0f;
	
	public BodyDef	LFootDef, RFootDef, LCalfDef, RCalfDef, LThighDef, RThighDef, 
					PelvisDef, StomachDef, ChestDef, NeckDef, HeadDef, 
					LUpperArmDef, RUpperArmDef, LForearmDef, RForearmDef, LHandDef, RHandDef;

	public PolygonDef LFootPoly, RFootPoly, LCalfPoly, RCalfPoly, LThighPoly, RThighPoly,
					  PelvisPoly, StomachPoly, ChestPoly, NeckPoly,
					  LUpperArmPoly, RUpperArmPoly, LForearmPoly, RForearmPoly, LHandPoly, RHandPoly;

	public CircleDef HeadCirc;

	public RevoluteJointDef LAnkleDef, RAnkleDef, LKneeDef, RKneeDef, LHipDef, RHipDef, 
							LowerAbsDef, UpperAbsDef, LowerNeckDef, UpperNeckDef,
							LShoulderDef, RShoulderDef, LElbowDef, RElbowDef, LWristDef, RWristDef;

	public BipedDef() {
		LFootDef = new BodyDef(); 
		RFootDef = new BodyDef(); 
		LCalfDef = new BodyDef(); 
		RCalfDef = new BodyDef(); 
		LThighDef = new BodyDef(); 
		RThighDef = new BodyDef(); 
		PelvisDef = new BodyDef(); 
		StomachDef = new BodyDef(); 
		ChestDef = new BodyDef(); 
		NeckDef = new BodyDef(); 
		HeadDef = new BodyDef(); 
		LUpperArmDef = new BodyDef(); 
		RUpperArmDef = new BodyDef(); 
		LForearmDef = new BodyDef(); 
		RForearmDef = new BodyDef(); 
		LHandDef = new BodyDef(); 
		RHandDef = new BodyDef();
		
		LFootPoly = new PolygonDef();
		RFootPoly = new PolygonDef();  
		LCalfPoly = new PolygonDef();  
		RCalfPoly = new PolygonDef();  
		LThighPoly = new PolygonDef();  
		RThighPoly = new PolygonDef(); 
		PelvisPoly = new PolygonDef();  
		StomachPoly = new PolygonDef();  
		ChestPoly = new PolygonDef();  
		NeckPoly = new PolygonDef(); 
		LUpperArmPoly = new PolygonDef();  
		RUpperArmPoly = new PolygonDef();  
		LForearmPoly = new PolygonDef();  
		RForearmPoly = new PolygonDef();  
		LHandPoly = new PolygonDef();  
		RHandPoly = new PolygonDef();
		
		HeadCirc = new CircleDef();
		
		LAnkleDef = new RevoluteJointDef();
		RAnkleDef = new RevoluteJointDef(); 
		LKneeDef = new RevoluteJointDef(); 
		RKneeDef = new RevoluteJointDef(); 
		LHipDef = new RevoluteJointDef(); 
		RHipDef = new RevoluteJointDef(); 
		LowerAbsDef = new RevoluteJointDef(); 
		UpperAbsDef = new RevoluteJointDef(); 
		LowerNeckDef = new RevoluteJointDef(); 
		UpperNeckDef = new RevoluteJointDef();
		LShoulderDef = new RevoluteJointDef(); 
		RShoulderDef = new RevoluteJointDef(); 
		LElbowDef = new RevoluteJointDef(); 
		RElbowDef = new RevoluteJointDef(); 
		LWristDef = new RevoluteJointDef(); 
		RWristDef = new RevoluteJointDef();
		
		
		
		setMotorTorque(2.0f);
		setMotorSpeed(0.0f);
		setDensity(20.0f);
		setRestitution(0.0f);
		setLinearDamping(0.0f);
		setAngularDamping(0.005f);
		setGroupIndex(--count);
		enableMotor();
		enableLimit();
		
		defaultVertices();
		defaultPositions();
		defaultJoints();

		LFootPoly.friction = RFootPoly.friction = 0.85f;
	}

	public void isFast(boolean b) {
		//B2_NOT_USED(b);
		/*
		LFootDef.isFast			= b;
		RFootDef.isFast			= b;
		LCalfDef.isFast			= b;
		RCalfDef.isFast			= b;
		LThighDef.isFast		= b;
		RThighDef.isFast		= b;
		PelvisDef.isFast		= b;
		StomachDef.isFast		= b;
		ChestDef.isFast			= b;
		NeckDef.isFast			= b;
		HeadDef.isFast			= b;
		LUpperArmDef.isFast		= b;
		RUpperArmDef.isFast		= b;
		LForearmDef.isFast		= b;
		RForearmDef.isFast		= b;
		LHandDef.isFast			= b;
		RHandDef.isFast			= b;
		*/
	}

	public void setGroupIndex(int i) {
		LFootPoly.groupIndex		= i;
		RFootPoly.groupIndex		= i;
		LCalfPoly.groupIndex		= i;
		RCalfPoly.groupIndex		= i;
		LThighPoly.groupIndex		= i;
		RThighPoly.groupIndex		= i;
		PelvisPoly.groupIndex		= i;
		StomachPoly.groupIndex		= i;
		ChestPoly.groupIndex		= i;
		NeckPoly.groupIndex			= i;
		HeadCirc.groupIndex			= i;
		LUpperArmPoly.groupIndex	= i;
		RUpperArmPoly.groupIndex	= i;
		LForearmPoly.groupIndex		= i;
		RForearmPoly.groupIndex		= i;
		LHandPoly.groupIndex		= i;
		RHandPoly.groupIndex		= i;
	}

	public void setLinearDamping(float f) {
		LFootDef.linearDamping		= f;
		RFootDef.linearDamping		= f;
		LCalfDef.linearDamping		= f;
		RCalfDef.linearDamping		= f;
		LThighDef.linearDamping		= f;
		RThighDef.linearDamping		= f;
		PelvisDef.linearDamping		= f;
		StomachDef.linearDamping	= f;
		ChestDef.linearDamping		= f;
		NeckDef.linearDamping		= f;
		HeadDef.linearDamping		= f;
		LUpperArmDef.linearDamping	= f;
		RUpperArmDef.linearDamping	= f;
		LForearmDef.linearDamping	= f;
		RForearmDef.linearDamping	= f;
		LHandDef.linearDamping		= f;
		RHandDef.linearDamping		= f;
	}

	public void setAngularDamping(float f) {
		LFootDef.angularDamping		= f;
		RFootDef.angularDamping		= f;
		LCalfDef.angularDamping		= f;
		RCalfDef.angularDamping		= f;
		LThighDef.angularDamping	= f;
		RThighDef.angularDamping	= f;
		PelvisDef.angularDamping	= f;
		StomachDef.angularDamping	= f;
		ChestDef.angularDamping		= f;
		NeckDef.angularDamping		= f;
		HeadDef.angularDamping		= f;
		LUpperArmDef.angularDamping	= f;
		RUpperArmDef.angularDamping	= f;
		LForearmDef.angularDamping	= f;
		RForearmDef.angularDamping	= f;
		LHandDef.angularDamping		= f;
		RHandDef.angularDamping		= f;
	}

	public void setMotorTorque(float f) {
		LAnkleDef.maxMotorTorque		= f;
		RAnkleDef.maxMotorTorque		= f;
		LKneeDef.maxMotorTorque		= f;
		RKneeDef.maxMotorTorque		= f;
		LHipDef.maxMotorTorque			= f;
		RHipDef.maxMotorTorque			= f;
		LowerAbsDef.maxMotorTorque		= f;
		UpperAbsDef.maxMotorTorque		= f;
		LowerNeckDef.maxMotorTorque	= f;
		UpperNeckDef.maxMotorTorque	= f;
		LShoulderDef.maxMotorTorque	= f;
		RShoulderDef.maxMotorTorque	= f;
		LElbowDef.maxMotorTorque		= f;
		RElbowDef.maxMotorTorque		= f;
		LWristDef.maxMotorTorque		= f;
		RWristDef.maxMotorTorque		= f;
	}

	public void setMotorSpeed(float f) {
		LAnkleDef.motorSpeed		= f;
		RAnkleDef.motorSpeed		= f;
		LKneeDef.motorSpeed			= f;
		RKneeDef.motorSpeed			= f;
		LHipDef.motorSpeed			= f;
		RHipDef.motorSpeed			= f;
		LowerAbsDef.motorSpeed		= f;
		UpperAbsDef.motorSpeed		= f;
		LowerNeckDef.motorSpeed		= f;
		UpperNeckDef.motorSpeed		= f;
		LShoulderDef.motorSpeed		= f;
		RShoulderDef.motorSpeed		= f;
		LElbowDef.motorSpeed		= f;
		RElbowDef.motorSpeed		= f;
		LWristDef.motorSpeed		= f;
		RWristDef.motorSpeed		= f;
	}

	public void setDensity(float f) {
		LFootPoly.density			= f;
		RFootPoly.density			= f;
		LCalfPoly.density			= f;
		RCalfPoly.density			= f;
		LThighPoly.density			= f;
		RThighPoly.density			= f;
		PelvisPoly.density			= f;
		StomachPoly.density			= f;
		ChestPoly.density			= f;
		NeckPoly.density			= f;
		HeadCirc.density			= f;
		LUpperArmPoly.density		= f;
		RUpperArmPoly.density		= f;
		LForearmPoly.density		= f;
		RForearmPoly.density		= f;
		LHandPoly.density			= f;
		RHandPoly.density			= f;
	}

	public void setRestitution(float f) {
		LFootPoly.restitution		= f;
		RFootPoly.restitution		= f;
		LCalfPoly.restitution		= f;
		RCalfPoly.restitution		= f;
		LThighPoly.restitution		= f;
		RThighPoly.restitution		= f;
		PelvisPoly.restitution		= f;
		StomachPoly.restitution		= f;
		ChestPoly.restitution		= f;
		NeckPoly.restitution		= f;
		HeadCirc.restitution		= f;
		LUpperArmPoly.restitution	= f;
		RUpperArmPoly.restitution	= f;
		LForearmPoly.restitution	= f;
		RForearmPoly.restitution	= f;
		LHandPoly.restitution		= f;
		RHandPoly.restitution		= f;
	}

	public void enableLimit() {
		setLimit(true);
	}

	public void disableLimit() {
		setLimit(false);
	}

	public void setLimit(boolean b) {
		LAnkleDef.enableLimit		= b;
		RAnkleDef.enableLimit		= b;
		LKneeDef.enableLimit		= b;
		RKneeDef.enableLimit		= b;
		LHipDef.enableLimit			= b;
		RHipDef.enableLimit			= b;
		LowerAbsDef.enableLimit		= b;
		UpperAbsDef.enableLimit		= b;
		LowerNeckDef.enableLimit	= b;
		UpperNeckDef.enableLimit	= b;
		LShoulderDef.enableLimit	= b;
		RShoulderDef.enableLimit	= b;
		LElbowDef.enableLimit		= b;
		RElbowDef.enableLimit		= b;
		LWristDef.enableLimit		= b;
		RWristDef.enableLimit		= b;
	}

	public void enableMotor() {
		setMotor(true);
	}

	public void disableMotor() {
		setMotor(false);
	}

	public void setMotor(boolean b) {
		LAnkleDef.enableMotor		= b;
		RAnkleDef.enableMotor		= b;
		LKneeDef.enableMotor		= b;
		RKneeDef.enableMotor		= b;
		LHipDef.enableMotor			= b;
		RHipDef.enableMotor			= b;
		LowerAbsDef.enableMotor		= b;
		UpperAbsDef.enableMotor		= b;
		LowerNeckDef.enableMotor	= b;
		UpperNeckDef.enableMotor	= b;
		LShoulderDef.enableMotor	= b;
		RShoulderDef.enableMotor	= b;
		LElbowDef.enableMotor		= b;
		RElbowDef.enableMotor		= b;
		LWristDef.enableMotor		= b;
		RWristDef.enableMotor		= b;
	}

	public void defaultVertices() {
		{	// feet
			LFootPoly.vertices.add(new Vec2(0.033f*k_scale, .143f*k_scale));
			LFootPoly.vertices.add(new Vec2(0.023f*k_scale, .033f*k_scale));
			LFootPoly.vertices.add(new Vec2(0.267f*k_scale, .035f*k_scale));
			LFootPoly.vertices.add(new Vec2(0.265f*k_scale, .065f*k_scale));
			LFootPoly.vertices.add(new Vec2(0.117f*k_scale, .143f*k_scale));
			RFootPoly.vertices.add(new Vec2(0.033f*k_scale, .143f*k_scale));
			RFootPoly.vertices.add(new Vec2(0.023f*k_scale, .033f*k_scale));
			RFootPoly.vertices.add(new Vec2(0.267f*k_scale, .035f*k_scale));
			RFootPoly.vertices.add(new Vec2(0.265f*k_scale, .065f*k_scale));
			RFootPoly.vertices.add(new Vec2(0.117f*k_scale, .143f*k_scale));
		}
		{	// calves
			LCalfPoly.vertices.add(new Vec2(.089f*k_scale, .016f*k_scale));
			RCalfPoly.vertices.add(new Vec2(.089f*k_scale, .016f*k_scale));
			LCalfPoly.vertices.add(new Vec2(.178f*k_scale, .016f*k_scale));
			RCalfPoly.vertices.add(new Vec2(.178f*k_scale, .016f*k_scale));
			LCalfPoly.vertices.add(new Vec2(.205f*k_scale, .417f*k_scale));
			RCalfPoly.vertices.add(new Vec2(.205f*k_scale, .417f*k_scale));
			LCalfPoly.vertices.add(new Vec2(.095f*k_scale, .417f*k_scale));
			RCalfPoly.vertices.add(new Vec2(.095f*k_scale, .417f*k_scale));
		}
		{	// thighs
			LThighPoly.vertices.add(new Vec2(.137f*k_scale, .032f*k_scale));
			RThighPoly.vertices.add(new Vec2(.137f*k_scale, .032f*k_scale));
			LThighPoly.vertices.add(new Vec2(.243f*k_scale, .032f*k_scale));
			RThighPoly.vertices.add(new Vec2(.243f*k_scale, .032f*k_scale));
			LThighPoly.vertices.add(new Vec2(.318f*k_scale, .343f*k_scale));
			RThighPoly.vertices.add(new Vec2(.318f*k_scale, .343f*k_scale));
			LThighPoly.vertices.add(new Vec2(.142f*k_scale, .343f*k_scale));
			RThighPoly.vertices.add(new Vec2(.142f*k_scale, .343f*k_scale));
		}
		{	// pelvis
			PelvisPoly.vertices.add(new Vec2(.105f*k_scale, .051f*k_scale));
			PelvisPoly.vertices.add(new Vec2(.277f*k_scale, .053f*k_scale));
			PelvisPoly.vertices.add(new Vec2(.320f*k_scale, .233f*k_scale));
			PelvisPoly.vertices.add(new Vec2(.112f*k_scale, .233f*k_scale));
			PelvisPoly.vertices.add(new Vec2(.067f*k_scale, .152f*k_scale));
		}
		{	// stomach
			StomachPoly.vertices.add(new Vec2(.088f*k_scale, .043f*k_scale));
			StomachPoly.vertices.add(new Vec2(.284f*k_scale, .043f*k_scale));
			StomachPoly.vertices.add(new Vec2(.295f*k_scale, .231f*k_scale));
			StomachPoly.vertices.add(new Vec2(.100f*k_scale, .231f*k_scale));
		}
		{	// chest
			ChestPoly.vertices.add(new Vec2(.091f*k_scale, .042f*k_scale));
			ChestPoly.vertices.add(new Vec2(.283f*k_scale, .042f*k_scale));
			ChestPoly.vertices.add(new Vec2(.177f*k_scale, .289f*k_scale));
			ChestPoly.vertices.add(new Vec2(.065f*k_scale, .289f*k_scale));
		}
		{	// head
			HeadCirc.radius = k_scale * .115f;
		}
		{	// neck
			NeckPoly.vertices.add(new Vec2(.038f*k_scale, .054f*k_scale));
			NeckPoly.vertices.add(new Vec2(.149f*k_scale, .054f*k_scale));
			NeckPoly.vertices.add(new Vec2(.154f*k_scale, .102f*k_scale));
			NeckPoly.vertices.add(new Vec2(.054f*k_scale, .113f*k_scale));
		}
		{	// upper arms
			LUpperArmPoly.vertices.add(new Vec2(.092f*k_scale, .059f*k_scale));
			LUpperArmPoly.vertices.add(new Vec2(.159f*k_scale, .059f*k_scale));
			LUpperArmPoly.vertices.add(new Vec2(.169f*k_scale, .335f*k_scale));
			LUpperArmPoly.vertices.add(new Vec2(.078f*k_scale, .335f*k_scale));
			LUpperArmPoly.vertices.add(new Vec2(.064f*k_scale, .248f*k_scale));
			RUpperArmPoly.vertices.add(new Vec2(.092f*k_scale, .059f*k_scale));
			RUpperArmPoly.vertices.add(new Vec2(.159f*k_scale, .059f*k_scale));
			RUpperArmPoly.vertices.add(new Vec2(.169f*k_scale, .335f*k_scale));
			RUpperArmPoly.vertices.add(new Vec2(.078f*k_scale, .335f*k_scale));
			RUpperArmPoly.vertices.add(new Vec2(.064f*k_scale, .248f*k_scale));
		}
		{	// forearms
			LForearmPoly.vertices.add(new Vec2(.082f*k_scale, .054f*k_scale));
			LForearmPoly.vertices.add(new Vec2(.138f*k_scale, .054f*k_scale));
			LForearmPoly.vertices.add(new Vec2(.149f*k_scale, .296f*k_scale));
			LForearmPoly.vertices.add(new Vec2(.088f*k_scale, .296f*k_scale));
			RForearmPoly.vertices.add(new Vec2(.082f*k_scale, .054f*k_scale));
			RForearmPoly.vertices.add(new Vec2(.138f*k_scale, .054f*k_scale));
			RForearmPoly.vertices.add(new Vec2(.149f*k_scale, .296f*k_scale));
			RForearmPoly.vertices.add(new Vec2(.088f*k_scale, .296f*k_scale));
		}
		{	// hands
			LHandPoly.vertices.add(new Vec2(.066f*k_scale, .031f*k_scale));
			LHandPoly.vertices.add(new Vec2(.123f*k_scale, .020f*k_scale));
			LHandPoly.vertices.add(new Vec2(.160f*k_scale, .127f*k_scale));
			LHandPoly.vertices.add(new Vec2(.127f*k_scale, .178f*k_scale));
			LHandPoly.vertices.add(new Vec2(.074f*k_scale, .178f*k_scale));
			RHandPoly.vertices.add(new Vec2(.066f*k_scale, .031f*k_scale));
			RHandPoly.vertices.add(new Vec2(.123f*k_scale, .020f*k_scale));
			RHandPoly.vertices.add(new Vec2(.160f*k_scale, .127f*k_scale));
			RHandPoly.vertices.add(new Vec2(.127f*k_scale, .178f*k_scale));
			RHandPoly.vertices.add(new Vec2(.074f*k_scale, .178f*k_scale));
		}
	}

	public void defaultJoints() {
		//b.LAnkleDef.body1		= LFoot;
		//b.LAnkleDef.body2		= LCalf;
		//b.RAnkleDef.body1		= RFoot;
		//b.RAnkleDef.body2		= RCalf;
		{	// ankles
			Vec2 anchor = new Vec2(-.045f,-.75f);
			anchor.mulLocal(k_scale);
			LAnkleDef.localAnchor1	= anchor.sub(LFootDef.position);
			RAnkleDef.localAnchor1	= anchor.sub(LFootDef.position);
			LAnkleDef.localAnchor2  = anchor.sub(LCalfDef.position);
			RAnkleDef.localAnchor2	= anchor.sub(LCalfDef.position);
			LAnkleDef.referenceAngle	= RAnkleDef.referenceAngle	= 0.0f;
			LAnkleDef.lowerAngle		= RAnkleDef.lowerAngle		= -0.523598776f;
			LAnkleDef.upperAngle		= RAnkleDef.upperAngle		= 0.523598776f;
		}

		//b.LKneeDef.body1		= LCalf;
		//b.LKneeDef.body2		= LThigh;
		//b.RKneeDef.body1		= RCalf;
		//b.RKneeDef.body2		= RThigh;
		{	// knees
			Vec2 anchor = new Vec2(-.030f,-.355f);
			anchor.mulLocal(k_scale);
			LKneeDef.localAnchor1	= anchor.sub(LCalfDef.position);
			RKneeDef.localAnchor1	= anchor.sub(LCalfDef.position);
			LKneeDef.localAnchor2	= anchor.sub(LThighDef.position);
			RKneeDef.localAnchor2	= anchor.sub(LThighDef.position);
			LKneeDef.referenceAngle	= RKneeDef.referenceAngle	= 0.0f;
			LKneeDef.lowerAngle		= RKneeDef.lowerAngle		= 0;
			LKneeDef.upperAngle		= RKneeDef.upperAngle		= 2.61799388f;
		}

		//b.LHipDef.body1			= LThigh;
		//b.LHipDef.body2			= Pelvis;
		//b.RHipDef.body1			= RThigh;
		//b.RHipDef.body2			= Pelvis;
		{	// hips
			Vec2 anchor = new Vec2(.005f,-.045f);
			anchor.mulLocal(k_scale);
			LHipDef.localAnchor1	= anchor.sub(LThighDef.position);
			RHipDef.localAnchor1	= anchor.sub(LThighDef.position);
			LHipDef.localAnchor2	= anchor.sub(PelvisDef.position);
			RHipDef.localAnchor2	= anchor.sub(PelvisDef.position);
			LHipDef.referenceAngle	= RHipDef.referenceAngle	= 0.0f;
			LHipDef.lowerAngle		= RHipDef.lowerAngle		= -2.26892803f;
			LHipDef.upperAngle		= RHipDef.upperAngle		= 0;
		}

		//b.LowerAbsDef.body1		= Pelvis;
		//b.LowerAbsDef.body2		= Stomach;
		{	// lower abs
			Vec2 anchor = new Vec2(.035f,.135f);
			anchor.mulLocal(k_scale);
			LowerAbsDef.localAnchor1	= anchor.sub(PelvisDef.position);
			LowerAbsDef.localAnchor2	= anchor.sub(StomachDef.position);
			LowerAbsDef.referenceAngle	= 0.0f;
			LowerAbsDef.lowerAngle		= -0.523598776f;
			LowerAbsDef.upperAngle		= 0.523598776f;
		}

		//b.UpperAbsDef.body1		= Stomach;
		//b.UpperAbsDef.body2		= Chest;
		{	// upper abs
			Vec2 anchor = new Vec2(.045f,.320f);
			anchor.mulLocal(k_scale);
			UpperAbsDef.localAnchor1	= anchor.sub(StomachDef.position);
			UpperAbsDef.localAnchor2	= anchor.sub(ChestDef.position);
			UpperAbsDef.referenceAngle	= 0.0f;
			UpperAbsDef.lowerAngle		= -0.523598776f;
			UpperAbsDef.upperAngle		= 0.174532925f;
		}

		//b.LowerNeckDef.body1	= Chest;
		//b.LowerNeckDef.body2	= Neck;
		{	// lower neck
			Vec2 anchor = new Vec2(-.015f,.575f);
			anchor.mulLocal(k_scale);
			LowerNeckDef.localAnchor1	= anchor.sub(ChestDef.position);
			LowerNeckDef.localAnchor2	= anchor.sub(NeckDef.position);
			LowerNeckDef.referenceAngle	= 0.0f;
			LowerNeckDef.lowerAngle		= -0.174532925f;
			LowerNeckDef.upperAngle		= 0.174532925f;
		}

		//b.UpperNeckDef.body1	= Chest;
		//b.UpperNeckDef.body2	= Head;
		{	// upper neck
			Vec2 anchor = new Vec2(-.005f,.630f);
			anchor.mulLocal(k_scale);
			UpperNeckDef.localAnchor1	= anchor.sub(ChestDef.position);
			UpperNeckDef.localAnchor2	= anchor.sub(HeadDef.position);
			UpperNeckDef.referenceAngle	= 0.0f;
			UpperNeckDef.lowerAngle		= -0.610865238f;
			UpperNeckDef.upperAngle		= 0.785398163f;
		}

		//b.LShoulderDef.body1	= Chest;
		//b.LShoulderDef.body2	= LUpperArm;
		//b.RShoulderDef.body1	= Chest;
		//b.RShoulderDef.body2	= RUpperArm;
		{	// shoulders
			Vec2 anchor = new Vec2(-.015f,.545f);
			anchor.mulLocal(k_scale);
			LShoulderDef.localAnchor1	= anchor.sub(ChestDef.position);
			RShoulderDef.localAnchor1	= anchor.sub(ChestDef.position);
			LShoulderDef.localAnchor2	= anchor.sub(LUpperArmDef.position);
			RShoulderDef.localAnchor2	= anchor.sub(LUpperArmDef.position);
			LShoulderDef.referenceAngle	= RShoulderDef.referenceAngle	= 0.0f;
			LShoulderDef.lowerAngle		= RShoulderDef.lowerAngle		= -1.04719755f;
			LShoulderDef.upperAngle		= RShoulderDef.upperAngle		= 3.14159265f;
		}

		//b.LElbowDef.body1		= LForearm;
		//b.LElbowDef.body2		= LUpperArm;
		//b.RElbowDef.body1		= RForearm;
		//b.RElbowDef.body2		= RUpperArm;
		{	// elbows
			Vec2 anchor = new Vec2(-.005f,.290f);
			anchor.mulLocal(k_scale);
			LElbowDef.localAnchor1		= anchor.sub(LForearmDef.position);
			RElbowDef.localAnchor1		= anchor.sub(LForearmDef.position);
			LElbowDef.localAnchor2		= anchor.sub(LUpperArmDef.position);
			RElbowDef.localAnchor2		= anchor.sub(LUpperArmDef.position);
			LElbowDef.referenceAngle	= RElbowDef.referenceAngle	= 0.0f;
			LElbowDef.lowerAngle		= RElbowDef.lowerAngle		= -2.7925268f;
			LElbowDef.upperAngle		= RElbowDef.upperAngle		= 0;
		}

		//b.LWristDef.body1		= LHand;
		//b.LWristDef.body2		= LForearm;
		//b.RWristDef.body1		= RHand;
		//b.RWristDef.body2		= RForearm;
		{	// wrists
			Vec2 anchor = new Vec2(-.010f,.045f);
			anchor.mulLocal(k_scale);
			LWristDef.localAnchor1		= anchor.sub(LHandDef.position);
			RWristDef.localAnchor1		= anchor.sub(LHandDef.position);
			LWristDef.localAnchor2		= anchor.sub(LForearmDef.position);
			RWristDef.localAnchor2		= anchor.sub(LForearmDef.position);
			LWristDef.referenceAngle	= RWristDef.referenceAngle	= 0.0f;
			LWristDef.lowerAngle		= RWristDef.lowerAngle		= -0.174532925f;
			LWristDef.upperAngle		= RWristDef.upperAngle		= 0.174532925f;
		}
	}

	public void defaultPositions() {
		LFootDef.position		= (new Vec2(-.122f,-.901f).mulLocal(k_scale));
		RFootDef.position		= (new Vec2(-.122f,-.901f).mulLocal(k_scale));
		LCalfDef.position		= (new Vec2(-.177f,-.771f).mulLocal(k_scale));
		RCalfDef.position		= (new Vec2(-.177f,-.771f).mulLocal(k_scale));
		LThighDef.position		= (new Vec2(-.217f,-.391f).mulLocal(k_scale));
		RThighDef.position		= (new Vec2(-.217f,-.391f).mulLocal(k_scale));
		LUpperArmDef.position	= (new Vec2(-.127f,.228f).mulLocal(k_scale));
		RUpperArmDef.position	= (new Vec2(-.127f,.228f).mulLocal(k_scale));
		LForearmDef.position	= (new Vec2(-.117f,-.011f).mulLocal(k_scale));
		RForearmDef.position	= (new Vec2(-.117f,-.011f).mulLocal(k_scale));
		LHandDef.position		= (new Vec2(-.112f,-.136f).mulLocal(k_scale));
		RHandDef.position		= (new Vec2(-.112f,-.136f).mulLocal(k_scale));
		PelvisDef.position		= (new Vec2(-.177f,-.101f).mulLocal(k_scale));
		StomachDef.position		= (new Vec2(-.142f,.088f).mulLocal(k_scale));
		ChestDef.position		= (new Vec2(-.132f,.282f).mulLocal(k_scale));
		NeckDef.position		= (new Vec2(-.102f,.518f).mulLocal(k_scale));
		HeadDef.position		= (new Vec2(.022f,.738f).mulLocal(k_scale));
	}

}
