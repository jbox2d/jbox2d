package dynamics.joints;

import common.Vec2;
import dynamics.Body;

public abstract class Joint {

    public JointType m_type;

    public Joint m_prev;

    public Joint m_next;

    public JointNode m_node1;

    public JointNode m_node2;

    public Body m_body1;

    public Body m_body2;

    public boolean m_islandFlag;

    public static Joint Create(JointDescription description) {
        Joint joint = null;

        if (description.type == JointType.distanceJoint) {
            joint = new DistanceJoint((DistanceJointDescription) description);
        }
        else if (description.type == JointType.mouseJoint) {
            joint = new MouseJoint((MouseDescription) description);
        }
        else if (description.type == JointType.prismaticJoint) {
            joint = new PrismaticJoint((PrismaticJointDescription) description);
        }
        else if (description.type == JointType.revoluteJoint) {
            joint = new RevoluteJoint((RevoluteDescription) description);
        }
        else {
            assert false;
        }

        return joint;
    }

    public Joint(JointDescription description) {
        m_type = description.type;
        m_prev = null;
        m_next = null;
        m_node1 = new JointNode();
        m_node2 = new JointNode();
        m_body1 = description.body1;
        m_body2 = description.body2;

    }

    public abstract void PreSolve();

    public abstract void SolveVelocityConstraints(float dt);

    // This returns true if the position errors are within tolerance.
    public abstract boolean SolvePositionConstraints();

    public abstract Vec2 GetAnchor1();

    public abstract Vec2 GetAnchor2();
}
