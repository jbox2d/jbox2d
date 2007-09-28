package dynamics.joints;

import common.Vec2;
import dynamics.Body;

public abstract class Joint {

	JointType m_type;
	Joint m_prev;
	Joint m_next;
	JointNode m_node1;
	JointNode m_node2;
	Body m_body1;
	Body m_body2;

	boolean m_islandFlag;

	static Joint Create(JointDescription description) {
		Joint joint = null;

		if (description.type == JointType.distanceJoint) {
			joint = new DistanceJoint((DistanceJointDescription) description);
		} else if (description.type == JointType.mouseJoint) {
			joint = new MouseJoint((MouseDescription) description);
		} else if (description.type == JointType.prismaticJoint) {
			joint = new PrismaticJoint((PrismaticJointDescription) description);
		} else if (description.type == JointType.revoluteJoint) {
			joint = new RevoluteJoint((RevoluteDescription) description);
		} else {
			assert false;
		}

		return joint;
	}

	public Joint(JointDescription description) {
		m_type = description.type;
		m_prev = null;
		m_next = null;
		m_body1 = description.body1;
		m_body2 = description.body2;

	}

	abstract void PreSolve();

	abstract void SolveVelocityConstraints(float dt);

	// This returns true if the position errors are within tolerance.
	abstract boolean SolvePositionConstraints();

	abstract Vec2 GetAnchor1();

	abstract Vec2 GetAnchor2();
}
