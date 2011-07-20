package org.jbox2d.serialization;

import org.jbox2d.dynamics.joints.JointType;
import org.jbox2d.proto.JBox2D.PbJointType;

public final class SerializationHelper {

	private SerializationHelper(){}
	
	public static boolean isIndependentJoint(JointType argType){
		return argType != JointType.GEAR;
	}
	
	public static boolean isIndependentJoint(PbJointType argType){
		return argType != PbJointType.GEAR;
	}
}
