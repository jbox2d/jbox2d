package org.jbox2d.dynamics.joints;

import org.jbox2d.dynamics.Body;

public class ConstantVolumeJointDef extends JointDef {
	Body[] bodies;
	public float frequencyHz;
	public float dampingRatio;
	//public float relaxationFactor;//1.0 is perfectly stiff (but doesn't work, unstable)
	
	public ConstantVolumeJointDef() {
		type = JointType.CONSTANT_VOLUME_JOINT;
		bodies = new Body[0];
		//relaxationFactor = 0.9f;
		collideConnected = false;
		frequencyHz = 0.0f;
		dampingRatio = 0.0f;
	}
	
	public void addBody(Body b) {
		Body[] tmp = new Body[bodies.length+1];
		System.arraycopy(bodies, 0, tmp, 0, bodies.length);
		tmp[bodies.length] = b;
		bodies = tmp;
		if (tmp.length == 1) body1 = b;
		if (tmp.length == 2) body2 = b;
	}
}
