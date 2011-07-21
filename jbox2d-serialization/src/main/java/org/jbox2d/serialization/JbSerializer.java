package org.jbox2d.serialization;

import java.util.Map;

import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.joints.Joint;

/**
 * Serializer for jbox2d, used to 
 * serialize any aspect of the physics world
 * @author Daniel
 *
 */
public interface JbSerializer {
	
	/**
	 * Sets the object signer for the serializer.  This allows
	 * the user to specify an 'tag' for each main physics object,
	 * which is then referenced later at deserialization for the user.
	 * @param argSigner
	 */
	public void setObjectSigner(ObjectSigner argSigner);
	
	/**
	 * Sets a listener for unsupported exception instead of 
	 * stopping the whole serialization process by throwing
	 * and exception.
	 * @param argListener
	 */
	public void setUnsupportedListener(UnsupportedListener argListener);

	public SerializationResult serialize(World argWorld);
	
	public SerializationResult serialize(Body argBody);
	
	public SerializationResult serialize(Fixture argFixture);
	
	public SerializationResult serialize(Shape argShape);
	
	/**
	 * Serializes joints.  Joints need to reference bodies
	 * and sometimes other joints.
	 * @param argJoint
	 * @param argBodyIndexMap
	 * @param argJointIndexMap
	 * @return
	 */
	public SerializationResult serialize(Joint argJoint,
			Map<Body, Integer> argBodyIndexMap,
			Map<Joint, Integer> argJointIndexMap);
	
	/**
	 * Interface that allows the serializer to
	 * look up tags for each object, which can be
	 * used later during deserializing by the developer.
	 * @author Daniel
	 */
	public static interface ObjectSigner {
		/**
		 * @param argWorld
		 * @return the tag for the world. can be null.
		 */
		public Long getTag(World argWorld);
		/**
		 * @param argBody
		 * @return the tag for the body.  can be null.
		 */
		public Long getTag(Body argBody);
		/**
		 * @param argShape
		 * @return the tag for the shape. can be null.
		 */
		public Long getTag(Shape argShape);
		/**
		 * @param argFixture
		 * @return the tag for the fixture. can be null.
		 */
		public Long getTag(Fixture argFixture);
		/**
		 * @param argJoint
		 * @return the tag for the joint. can be null.
		 */
		public Long getTag(Joint argJoint);
	}
}
