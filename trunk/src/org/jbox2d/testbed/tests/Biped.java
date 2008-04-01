package org.jbox2d.testbed.tests;

import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.joints.RevoluteJoint;

/**
 * Ragdoll class thanks to darkzerox.
 */
public class Biped {

	private World m_world;

	private Body LFoot, RFoot, LCalf, RCalf, LThigh, RThigh,
				 Pelvis, Stomach, Chest, Neck, Head,
				 LUpperArm, RUpperArm, LForearm, RForearm, LHand, RHand;

	private RevoluteJoint LAnkle, RAnkle, LKnee, RKnee, LHip, RHip, 
						  LowerAbs, UpperAbs, LowerNeck, UpperNeck,
						  LShoulder, RShoulder, LElbow, RElbow, LWrist, RWrist;

	public Biped(World w, Vec2 position) {
		m_world = w;

		BipedDef def = new BipedDef();
		BodyDef bd = new BodyDef();

		// create body parts
		bd = def.LFootDef;
		bd.position.addLocal(position);
		LFoot = w.createDynamicBody(bd);
		LFoot.createShape(def.LFootPoly);
		LFoot.setMassFromShapes();

		bd = def.RFootDef;
		bd.position.addLocal(position);
		RFoot = w.createDynamicBody(bd);
		RFoot.createShape(def.RFootPoly);
		RFoot.setMassFromShapes();

		bd = def.LCalfDef;
		bd.position.addLocal(position);
		LCalf = w.createDynamicBody(bd);
		LCalf.createShape(def.LCalfPoly);
		LCalf.setMassFromShapes();

		bd = def.RCalfDef;
		bd.position.addLocal(position);
		RCalf = w.createDynamicBody(bd);
		RCalf.createShape(def.RCalfPoly);
		RCalf.setMassFromShapes();

		bd = def.LThighDef;
		bd.position.addLocal(position);
		LThigh = w.createDynamicBody(bd);
		LThigh.createShape(def.LThighPoly);
		LThigh.setMassFromShapes();

		bd = def.RThighDef;
		bd.position.addLocal(position);
		RThigh = w.createDynamicBody(bd);
		RThigh.createShape(def.RThighPoly);
		RThigh.setMassFromShapes();

		bd = def.PelvisDef;
		bd.position.addLocal(position);
		Pelvis = w.createDynamicBody(bd);
		Pelvis.createShape(def.PelvisPoly);
		Pelvis.setMassFromShapes();

		bd = def.StomachDef;
		bd.position.addLocal(position);
		Stomach = w.createDynamicBody(bd);
		Stomach.createShape(def.StomachPoly);
		Stomach.setMassFromShapes();

		bd = def.ChestDef;
		bd.position.addLocal(position);
		Chest = w.createDynamicBody(bd);
		Chest.createShape(def.ChestPoly);
		Chest.setMassFromShapes();

		bd = def.NeckDef;
		bd.position.addLocal(position);
		Neck = w.createDynamicBody(bd);
		Neck.createShape(def.NeckPoly);
		Neck.setMassFromShapes();

		bd = def.HeadDef;
		bd.position.addLocal(position);
		Head = w.createDynamicBody(bd);
		Head.createShape(def.HeadCirc);
		Head.setMassFromShapes();

		bd = def.LUpperArmDef;
		bd.position.addLocal(position);
		LUpperArm = w.createDynamicBody(bd);
		LUpperArm.createShape(def.LUpperArmPoly);
		LUpperArm.setMassFromShapes();

		bd = def.RUpperArmDef;
		bd.position.addLocal(position);
		RUpperArm = w.createDynamicBody(bd);
		RUpperArm.createShape(def.RUpperArmPoly);
		RUpperArm.setMassFromShapes();

		bd = def.LForearmDef;
		bd.position.addLocal(position);
		LForearm = w.createDynamicBody(bd);
		LForearm.createShape(def.LForearmPoly);
		LForearm.setMassFromShapes();

		bd = def.RForearmDef;
		bd.position.addLocal(position);
		RForearm = w.createDynamicBody(bd);
		RForearm.createShape(def.RForearmPoly);
		RForearm.setMassFromShapes();

		bd = def.LHandDef;
		bd.position.addLocal(position);
		LHand = w.createDynamicBody(bd);
		LHand.createShape(def.LHandPoly);
		LHand.setMassFromShapes();

		bd = def.RHandDef;
		bd.position.addLocal(position);
		RHand = w.createDynamicBody(bd);
		RHand.createShape(def.RHandPoly);
		RHand.setMassFromShapes();
		
		// link body parts
		def.LAnkleDef.body1		= LFoot;
		def.LAnkleDef.body2		= LCalf;
		def.RAnkleDef.body1		= RFoot;
		def.RAnkleDef.body2		= RCalf;
		def.LKneeDef.body1		= LCalf;
		def.LKneeDef.body2		= LThigh;
		def.RKneeDef.body1		= RCalf;
		def.RKneeDef.body2		= RThigh;
		def.LHipDef.body1		= LThigh;
		def.LHipDef.body2		= Pelvis;
		def.RHipDef.body1		= RThigh;
		def.RHipDef.body2		= Pelvis;
		def.LowerAbsDef.body1	= Pelvis;
		def.LowerAbsDef.body2	= Stomach;
		def.UpperAbsDef.body1	= Stomach;
		def.UpperAbsDef.body2	= Chest;
		def.LowerNeckDef.body1	= Chest;
		def.LowerNeckDef.body2	= Neck;
		def.UpperNeckDef.body1	= Chest;
		def.UpperNeckDef.body2	= Head;
		def.LShoulderDef.body1	= Chest;
		def.LShoulderDef.body2	= LUpperArm;
		def.RShoulderDef.body1	= Chest;
		def.RShoulderDef.body2	= RUpperArm;
		def.LElbowDef.body1		= LForearm;
		def.LElbowDef.body2		= LUpperArm;
		def.RElbowDef.body1		= RForearm;
		def.RElbowDef.body2		= RUpperArm;
		def.LWristDef.body1		= LHand;
		def.LWristDef.body2		= LForearm;
		def.RWristDef.body1		= RHand;
		def.RWristDef.body2		= RForearm;
		
		// create joints
		LAnkle		= (RevoluteJoint)w.createJoint(def.LAnkleDef);
		RAnkle		= (RevoluteJoint)w.createJoint(def.RAnkleDef);
		LKnee		= (RevoluteJoint)w.createJoint(def.LKneeDef);
		RKnee		= (RevoluteJoint)w.createJoint(def.RKneeDef);
		LHip		= (RevoluteJoint)w.createJoint(def.LHipDef);
		RHip		= (RevoluteJoint)w.createJoint(def.RHipDef);
		LowerAbs	= (RevoluteJoint)w.createJoint(def.LowerAbsDef);
		UpperAbs	= (RevoluteJoint)w.createJoint(def.UpperAbsDef);
		LowerNeck	= (RevoluteJoint)w.createJoint(def.LowerNeckDef);
		UpperNeck	= (RevoluteJoint)w.createJoint(def.UpperNeckDef);
		LShoulder	= (RevoluteJoint)w.createJoint(def.LShoulderDef);
		RShoulder	= (RevoluteJoint)w.createJoint(def.RShoulderDef);
		LElbow		= (RevoluteJoint)w.createJoint(def.LElbowDef);
		RElbow		= (RevoluteJoint)w.createJoint(def.RElbowDef);
		LWrist		= (RevoluteJoint)w.createJoint(def.LWristDef);
		RWrist		= (RevoluteJoint)w.createJoint(def.RWristDef);
	}

};


