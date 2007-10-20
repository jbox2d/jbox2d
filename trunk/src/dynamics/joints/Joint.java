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
    
    //ewjordan: I've added a Destroy method because although
    //these usually just deallocate memory, it is possible that
    //Erin may alter them to do more nontrivial things, and we
    //should be prepared for this possibility.
    public static void Destroy(Joint j) {
        j.Destructor();
        return;
    }
    
    public void Destructor() {
        
    }

    public static Joint Create(JointDef description) {
        Joint joint = null;

        if (description.type == JointType.distanceJoint) {
            joint = new DistanceJoint((DistanceJointDef) description);
        }
        else if (description.type == JointType.mouseJoint) {
            joint = new MouseJoint((MouseDef) description);
        }
        else if (description.type == JointType.prismaticJoint) {
            joint = new PrismaticJoint((PrismaticJointDef) description);
        }
        else if (description.type == JointType.revoluteJoint) {
            joint = new RevoluteJoint((RevoluteJointDef) description);
        }
        else if (description.type == JointType.pulleyJoint) {
            joint = new PulleyJoint((PulleyJointDef) description);
        }
        else if (description.type == JointType.gearJoint) {
            joint = new GearJoint((GearJointDef) description);
        }
        else {
            assert false;
        }

        return joint;
    }

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
    
    public Body GetBody1() {
        return m_body1;
    }
    
    public Body GetBody2() {
        return m_body2;
    }
    
    public Joint GetNext() {
        return m_next;
    }
    
    public Object GetUserData() {
        return m_userData;
    }

    public abstract void PreSolve();

    public abstract void SolveVelocityConstraints(float dt);

    // This returns true if the position errors are within tolerance.
    public abstract boolean SolvePositionConstraints();

    public abstract Vec2 GetAnchor1();

    public abstract Vec2 GetAnchor2();
}
