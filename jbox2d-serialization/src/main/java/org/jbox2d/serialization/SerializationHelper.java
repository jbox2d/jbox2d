package org.jbox2d.serialization;

import org.jbox2d.dynamics.joints.JointType;

public final class SerializationHelper {

	private SerializationHelper(){}
	
	public static boolean isIndependentJoint(JointType argType){
		return argType != JointType.GEAR && argType != JointType.CONSTANT_VOLUME;
	}
}
