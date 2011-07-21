package org.jbox2d.serialization;

import java.io.IOException;
import java.io.InputStream;
import java.util.Map;

import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.joints.Joint;

public interface JbDeserializer {
	
	/**
	 * Sets the object listener, which allows the
	 * user to process each physics object that
	 * has a tag to do any sort of custom logic.
	 * @param argListener
	 */
	public void setObjectListener(ObjectListener argListener);
	
	/**
	 * Sets a listener for unsupported exceptions instead of 
	 * stopping the whole deserialization process by throwing
	 * and exception.
	 * @param argListener
	 */
	public void setUnsupportedListener(UnsupportedListener argListener);
	
	public World deserializeWorld(InputStream argInput) throws IOException;
	
	public Body deserializeBody(World argWorld, InputStream argInput) throws IOException;
	
	public Fixture deserializeFixture(Body argBody, InputStream argInput) throws IOException;
	
	public Shape deserializeShape(InputStream argInput) throws IOException;
	
	public Joint deserializeJoint(World argWorld, InputStream argInput, Map<Integer, 
								  Body> argBodyMap, Map<Integer, Joint> argJointMap) throws IOException;
	
	/**
	 * Called for each physics object with a tag defined.
	 * @author dmurph
	 *
	 */
	public static interface ObjectListener{
		
		public void processWorld(World argWorld, Long argTag);
		
		public void processBody(Body argBody, Long argTag);
		
		public void processFixture(Fixture argFixture, Long argTag);
		
		public void processShape(Shape argShape, Long argTag);
		
		public void processJoint(Joint argJoint, Long argTag);
	}
}
