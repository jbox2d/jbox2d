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
	 * the user to specify an 'id' for each main physics object,
	 * which is then referenced later at deserialization for the user.
	 * @param argSigner
	 */
	public void setObjectSigner(ObjectSigner argSigner);

	public SerializationResult serialize(World argWorld);
	
	public SerializationResult serialize(Body argBody);
	
	public SerializationResult serialize(Fixture argFixture);
	
	public SerializationResult serialize(Shape argShape);
	
	/** 
	 * Serializes independent joints (joints that don't reference
	 * other joints)
	 * @param argJoint
	 * @param argBodyIndexMap
	 * @return
	 */
	public SerializationResult serialize(Joint argJoint,
			Map<Body, Integer> argBodyIndexMap);
	
	/**
	 * Serialized dependent joints (joints that are built on other
	 * joints)
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
	 * look up id's for each object, that can be
	 * used later by the user when deserializing
	 * @author Daniel
	 */
	public static interface ObjectSigner {
		/**
		 * @param argWorld
		 * @return the id for the world. can be null.
		 */
		public Integer getId(World argWorld);
		/**
		 * @param argBody
		 * @return the id for the body.  can be null.
		 */
		public Integer getId(Body argBody);
		/**
		 * @param argShape
		 * @return the id for the shape. can be null.
		 */
		public Integer getId(Shape argShape);
		/**
		 * @param argFixture
		 * @return the id for the fixture. can be null.
		 */
		public Integer getId(Fixture argFixture);
		/**
		 * @param argJoint
		 * @return the id for the joint. can be null.
		 */
		public Integer getId(Joint argJoint);
	}
}
