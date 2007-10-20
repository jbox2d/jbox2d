package dynamics.joints;

public class GearJointDef extends JointDef {
    public Joint joint1;
    public Joint joint2;
    public float ratio;

    public GearJointDef() {
        type = JointType.gearJoint;
        joint1 = null;
        joint2 = null;
        ratio = 1.0f;
    }
}
