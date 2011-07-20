package org.jbox2d.serialization;

import java.io.InputStream;
import java.util.Map;

import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.joints.Joint;

public interface JbDeserializer {
	
	public World deserializeWorld(InputStream argInput);
	
	public Body deserializeBody(World argWorld, InputStream argInput);
	
	public Fixture deserializeFixture(World argWorld, InputStream argInput);
	
	public Fixture deserializeShape(InputStream argInput);
	
	public Joint deserializeJoint(World argWorld, InputStream argInput, Map<Integer, 
								  Body> argBodyMap, Map<Integer, Joint> argJointMap);
	
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
