package org.jbox2d.dynamics.joints;

import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.TimeStep;
import org.jbox2d.dynamics.World;

public class ConstantVolumeJoint extends Joint {
	Body[] bodies;
	float[] targetLengths;
	float targetVolume;
	//float relaxationFactor;//1.0 is perfectly stiff (but doesn't work, unstable)
	World world;
	
	Vec2[] normals;
	
	TimeStep m_step;
	
	DistanceJoint[] distanceJoints;
	
	public Body[] getBodies() {
		return bodies;
	}
	
	public ConstantVolumeJoint(ConstantVolumeJointDef def) {
		super(def);
		if (def.bodies.length <= 2) {
			throw new IllegalArgumentException("You cannot create a constant volume joint with less than three bodies.");
		}
		world = def.bodies[0].getWorld();
		bodies = def.bodies;
		//relaxationFactor = def.relaxationFactor;
		targetLengths = new float[bodies.length];
		for (int i=0; i<targetLengths.length; ++i) {
			int next = (i == targetLengths.length-1)?0:i+1;
			float dist = bodies[i].getWorldCenter().sub(bodies[next].getWorldCenter()).length();
			targetLengths[i] = dist;
		}
		targetVolume = getArea();
		
		distanceJoints = new DistanceJoint[bodies.length];
		for (int i=0; i<targetLengths.length; ++i) {
			int next = (i == targetLengths.length-1)?0:i+1;
			DistanceJointDef djd = new DistanceJointDef();
			djd.frequencyHz = def.frequencyHz;//20.0f;
			djd.dampingRatio = def.dampingRatio;//50.0f;
			djd.initialize(bodies[i], bodies[next], bodies[i].getWorldCenter(), bodies[next].getWorldCenter());
			distanceJoints[i] = (DistanceJoint)world.createJoint(djd);
		}
		
		normals = new Vec2[bodies.length];
		for (int i=0; i<normals.length; ++i) {
			normals[i] = new Vec2();
		}
		
		this.m_body1 = bodies[0];
		this.m_body2 = bodies[1];
		this.m_collideConnected = false;
	}
	
	@Override
	public void destructor() {
		for (int i=0; i<distanceJoints.length; ++i) {
			world.destroyJoint(distanceJoints[i]);
		}
	}
	
	private float getArea() {
		  float area = 0.0f;
		  area += bodies[bodies.length-1].getWorldCenter().x * bodies[0].getWorldCenter().y -
		  		  bodies[0].getWorldCenter().x * bodies[bodies.length-1].getWorldCenter().y;
		  for (int i=0; i<bodies.length-1; ++i){
		    area += bodies[i].getWorldCenter().x * bodies[i+1].getWorldCenter().y - 
		    		bodies[i+1].getWorldCenter().x * bodies[i].getWorldCenter().y;
		  }
		  area *= .5f;
		  return area;
	}
	
	/**
	 * Apply the position correction to the particles.
	 * @param step
	 */
	public boolean constrainEdges(TimeStep step) {
		  float perimeter = 0.0f;
		  for (int i=0; i<bodies.length; ++i) {
		    int next = (i==bodies.length-1)?0:i+1;
		    float dx = bodies[next].getWorldCenter().x-bodies[i].getWorldCenter().x;
		    float dy = bodies[next].getWorldCenter().y-bodies[i].getWorldCenter().y;
		    float dist = (float)Math.sqrt(dx*dx+dy*dy);
		    if (dist < Settings.EPSILON) dist = 1.0f;
		    normals[i].x = dy / dist;
		    normals[i].y = -dx / dist;
		    perimeter += dist;
		  }
		  
		  float deltaArea = targetVolume - getArea();
		  float toExtrude = 0.5f*deltaArea / perimeter; //*relaxationFactor
		  //float sumdeltax = 0.0f;
		  boolean done = true;
		  for (int i=0; i<bodies.length; ++i) {
		    int next = (i==bodies.length-1)?0:i+1;
		    Vec2 delta = new Vec2(toExtrude * (normals[i].x + normals[next].x),
		    					  toExtrude * (normals[i].y + normals[next].y));
		    //sumdeltax += dx;
		    float norm = delta.length();
		    if (norm > Settings.maxLinearCorrection) delta.mulLocal(Settings.maxLinearCorrection/norm);
		    if (norm > Settings.linearSlop) done = false;
		    bodies[next].m_sweep.c.x += delta.x;
		    bodies[next].m_sweep.c.y += delta.y;
		    bodies[next].synchronizeTransform();
		    //bodies[next].m_linearVelocity.x += delta.x * step.inv_dt;
		    //bodies[next].m_linearVelocity.y += delta.y * step.inv_dt;
		  }
		  //System.out.println(sumdeltax);
		  return done;
	}

	private float m_impulse = 0.0f;
	@Override
	public void initVelocityConstraints(TimeStep step) {
		m_step = step;
		
		Vec2[] d = new Vec2[bodies.length];
		for (int i=0; i<bodies.length; ++i) {
			int prev = (i==0)?bodies.length-1:i-1;
			int next = (i==bodies.length-1)?0:i+1;
			d[i] = bodies[next].getWorldCenter().sub(bodies[prev].getWorldCenter());
		}
		
		if (step.warmStarting) {
    		m_impulse *= step.dtRatio;
    		//float lambda = -2.0f * crossMassSum / dotMassSum;
    		//System.out.println(crossMassSum + " " +dotMassSum);
    		//lambda = MathUtils.clamp(lambda, -Settings.maxLinearCorrection, Settings.maxLinearCorrection);
    		//m_impulse = lambda;
    		for (int i=0; i<bodies.length; ++i) {
    			bodies[i].m_linearVelocity.x += bodies[i].m_invMass * d[i].y * .5f * m_impulse;
    			bodies[i].m_linearVelocity.y += bodies[i].m_invMass * -d[i].x * .5f * m_impulse;
    		}
    	} else {
    		m_impulse = 0.0f;
    	}
	}

	@Override
	public boolean solvePositionConstraints() {
		return constrainEdges(m_step);
	}

	@Override
	public void solveVelocityConstraints(TimeStep step) {
		float crossMassSum = 0.0f;
		float dotMassSum = 0.0f;
		Vec2[] d = new Vec2[bodies.length];
		for (int i=0; i<bodies.length; ++i) {
			int prev = (i==0)?bodies.length-1:i-1;
			int next = (i==bodies.length-1)?0:i+1;
			d[i] = bodies[next].getWorldCenter().sub(bodies[prev].getWorldCenter());
			dotMassSum += (d[i].lengthSquared())/bodies[i].getMass();
			crossMassSum += Vec2.cross(bodies[i].getLinearVelocity(),d[i]);
		}
		float lambda = -2.0f * crossMassSum / dotMassSum;
		//System.out.println(crossMassSum + " " +dotMassSum);
		//lambda = MathUtils.clamp(lambda, -Settings.maxLinearCorrection, Settings.maxLinearCorrection);
		m_impulse += lambda;
		//System.out.println(m_impulse);
		for (int i=0; i<bodies.length; ++i) {
			bodies[i].m_linearVelocity.x += bodies[i].m_invMass * d[i].y * .5f * lambda;
			bodies[i].m_linearVelocity.y += bodies[i].m_invMass * -d[i].x * .5f * lambda;
		}
	}

	@Override
	public Vec2 getAnchor1() {
		// TODO Auto-generated method stub
		return null;
	}
	
	@Override
	public Vec2 getAnchor2() {
		// TODO Auto-generated method stub
		return null;
	}
	
	@Override
	public Vec2 getReactionForce() {
		// TODO Auto-generated method stub
		return null;
	}
	
	@Override
	public float getReactionTorque() {
		// TODO Auto-generated method stub
		return 0;
	}

}
