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

    public boolean m_collideConnected;

    public Object m_userData;

    public Joint(JointDef description) {
        m_type = description.type;
        m_prev = null;
        m_next = null;
        m_node1 = new JointNode();
        m_node2 = new JointNode();
        m_body1 = description.body1;
        m_body2 = description.body2;
        m_collideConnected = description.collideConnected;
        m_islandFlag = false;
        m_userData = description.userData;
    }

    // ewjordan: I've added a Destroy method because although
    // these usually just deallocate memory, it is possible that
    // Erin may alter them to do more nontrivial things, and we
    // should be prepared for this possibility.
    public static void destroy(Joint j) {
        j.destructor();
        return;
    }

    public void destructor() {
    }

    public static Joint create(JointDef description) {
        Joint joint = null;

        if (description.type == JointType.DISTANCE_JOINT) {
            joint = new DistanceJoint((DistanceJointDef) description);
        }
        else if (description.type == JointType.MOUSE_JOINT) {
            joint = new MouseJoint((MouseDef) description);
        }
        else if (description.type == JointType.PRISMATIC_JOINT) {
            joint = new PrismaticJoint((PrismaticJointDef) description);
        }
        else if (description.type == JointType.REVOLUTE_JOINT) {
            joint = new RevoluteJoint((RevoluteJointDef) description);
        }
        else if (description.type == JointType.PULLEY_JOINT) {
            joint = new PulleyJoint((PulleyJointDef) description);
        }
        else if (description.type == JointType.GEAR_JOINT) {
            joint = new GearJoint((GearJointDef) description);
        }
        else {
            assert false;
        }

        return joint;
    }

    public Body getBody1() {
        return m_body1;
    }

    public Body getBody2() {
        return m_body2;
    }

    public Joint getNext() {
        return m_next;
    }

    public Object GetUserData() {
        return m_userData;
    }

    public abstract void preSolve();

    public abstract void solveVelocityConstraints(float dt);

    // This returns true if the position errors are within tolerance.
    public abstract boolean solvePositionConstraints();

    public abstract Vec2 getAnchor1();

    public abstract Vec2 getAnchor2();
}
