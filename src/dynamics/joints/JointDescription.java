package dynamics.joints;

import dynamics.Body;

public class JointDescription {
	JointType type;
	Body body1;
	Body body2;

	public JointDescription() {
		type = JointType.unknownJoint;
		body1 = null;
		body2 = null;
	}

}
