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
	
	public void setObjectListener(ObjectListener argListener);
	
	public World deserializeWorld(InputStream argInput) throws IOException;
	
	public Body deserializeBody(World argWorld, InputStream argInput) throws IOException;
	
	public Fixture deserializeFixture(Body argBody, InputStream argInput) throws IOException;
	
	public Shape deserializeShape(InputStream argInput) throws IOException;
	
	public Joint deserializeJoint(World argWorld, InputStream argInput, Map<Integer, 
								  Body> argBodyMap, Map<Integer, Joint> argJointMap) throws IOException;
	
	/**
	 * Called during deserialization
	 * @author dmurph
	 *
	 */
	public static interface ObjectListener{
		
		public void processWorld(World argWorld, Integer argId);
		
		public void processBody(Body argBody, Integer argId);
		
		public void processFixture(Fixture argFixture, Integer argId);
		
		public void processJoint(Joint argJoint, Integer argId);
	}
}
