import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.joints.JointDef;
import org.jbox2d.dynamics.joints.JointType;

/**
 * Created at 3:38:52 AM Jan 15, 2011
 */

/**
 * @author Daniel Murphy
 */
public class WeldJointDef extends JointDef {
	/**
	 * The local anchor point relative to body1's origin.
	 */
	public final Vec2 localAnchorA;

	/**
	 * The local anchor point relative to body2's origin.
	 */
	public final Vec2  localAnchorB;

	/**
	 * The body2 angle minus body1 angle in the reference state (radians).
	 */
	public float referenceAngle;
	
	public WeldJointDef(){
		type = JointType.WELD;
		localAnchorA = new Vec2();
		localAnchorB = new Vec2();
		referenceAngle = 0.0f;
	}
	
	/**
	 * Initialize the bodies, anchors, and reference angle using a world
	 * anchor point.
	 * @param bA
	 * @param bB
	 * @param anchor
	 */
	public void Initialize(Body bA, Body bB, Vec2 anchor){
		bodyA = bA;
		bodyB = bB;
		bodyA.getLocalPointToOut(anchor, localAnchorA);
		bodyB.getLocalPointToOut(anchor, localAnchorB);
		referenceAngle = bodyB.getAngle() - bodyA.getAngle();
	}
}
