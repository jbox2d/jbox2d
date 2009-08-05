package org.jbox2d.dynamics.joints;

import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.TimeStep;
import org.jbox2d.dynamics.World;
import org.jbox2d.pooling.arrays.Vec2Array;

public class ConstantVolumeJoint extends Joint {
	Body[] bodies;
	float[] targetLengths;
	float targetVolume;
	//float relaxationFactor;//1.0 is perfectly stiff (but doesn't work, unstable)
	World world;

	Vec2[] normals;

	TimeStep m_step;
	private float m_impulse = 0.0f;


	DistanceJoint[] distanceJoints;

	public Body[] getBodies() {
		return bodies;
	}

	public void inflate(final float factor) {
		targetVolume *= factor;
	}

	// djm this is not a hot method, so no pool. no one wants
	// to swim when it's cold out.  except when you have a hot
	// tub.....then its amazing.....hmmmm......
	public ConstantVolumeJoint(final ConstantVolumeJointDef def) {
		super(def);
		if (def.bodies.length <= 2) {
			throw new IllegalArgumentException("You cannot create a constant volume joint with less than three bodies.");
		}
		world = def.bodies[0].getWorld();
		bodies = def.bodies;
		//relaxationFactor = def.relaxationFactor;
		targetLengths = new float[bodies.length];
		for (int i=0; i<targetLengths.length; ++i) {
			final int next = (i == targetLengths.length-1)?0:i+1;
			final float dist = bodies[i].getMemberWorldCenter().sub(bodies[next].getMemberWorldCenter()).length();
			targetLengths[i] = dist;
		}
		targetVolume = getArea();

		distanceJoints = new DistanceJoint[bodies.length];
		for (int i=0; i<targetLengths.length; ++i) {
			final int next = (i == targetLengths.length-1)?0:i+1;
			final DistanceJointDef djd = new DistanceJointDef();
			djd.frequencyHz = def.frequencyHz;//20.0f;
			djd.dampingRatio = def.dampingRatio;//50.0f;
			djd.initialize(bodies[i], bodies[next], bodies[i].getMemberWorldCenter(), bodies[next].getMemberWorldCenter());
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
		// i'm glad i changed these all to member access
		area += bodies[bodies.length-1].getMemberWorldCenter().x * bodies[0].getMemberWorldCenter().y -
		bodies[0].getMemberWorldCenter().x * bodies[bodies.length-1].getMemberWorldCenter().y;
		for (int i=0; i<bodies.length-1; ++i){
			area += bodies[i].getMemberWorldCenter().x * bodies[i+1].getMemberWorldCenter().y -
			bodies[i+1].getMemberWorldCenter().x * bodies[i].getMemberWorldCenter().y;
		}
		area *= .5f;
		return area;
	}

	/**
	 * Apply the position correction to the particles.
	 * @param step
	 */
	public boolean constrainEdges(final TimeStep step) {
		float perimeter = 0.0f;
		for (int i=0; i<bodies.length; ++i) {
			final int next = (i==bodies.length-1)?0:i+1;
			final float dx = bodies[next].getMemberWorldCenter().x-bodies[i].getMemberWorldCenter().x;
			final float dy = bodies[next].getMemberWorldCenter().y-bodies[i].getMemberWorldCenter().y;
			float dist = MathUtils.sqrt(dx*dx+dy*dy);
			if (dist < Settings.EPSILON) {
				dist = 1.0f;
			}
			normals[i].x = dy / dist;
			normals[i].y = -dx / dist;
			perimeter += dist;
		}

		final float deltaArea = targetVolume - getArea();
		final float toExtrude = 0.5f*deltaArea / perimeter; //*relaxationFactor
		//float sumdeltax = 0.0f;
		boolean done = true;
		for (int i=0; i<bodies.length; ++i) {
			final int next = (i==bodies.length-1)?0:i+1;
			final Vec2 delta = new Vec2(toExtrude * (normals[i].x + normals[next].x),
			                            toExtrude * (normals[i].y + normals[next].y));
			//sumdeltax += dx;
			final float norm = delta.length();
			if (norm > Settings.maxLinearCorrection){
				delta.mulLocal(Settings.maxLinearCorrection/norm);
			}
			if (norm > Settings.linearSlop){
				done = false;
			}
			bodies[next].m_sweep.c.x += delta.x;
			bodies[next].m_sweep.c.y += delta.y;
			bodies[next].synchronizeTransform();
			//bodies[next].m_linearVelocity.x += delta.x * step.inv_dt;
			//bodies[next].m_linearVelocity.y += delta.y * step.inv_dt;
		}
		//System.out.println(sumdeltax);
		return done;
	}

	// djm pooled
	private static final Vec2Array tlD = new Vec2Array();
	@Override
	public void initVelocityConstraints(final TimeStep step) {
		m_step = step;
		
		final Vec2[] d = tlD.get(bodies.length);
		
		for (int i=0; i<bodies.length; ++i) {
			final int prev = (i==0)?bodies.length-1:i-1;
			final int next = (i==bodies.length-1)?0:i+1;
			d[i].set(bodies[next].getMemberWorldCenter());
			d[i].subLocal(bodies[prev].getMemberWorldCenter());
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
	public void solveVelocityConstraints(final TimeStep step) {
		float crossMassSum = 0.0f;
		float dotMassSum = 0.0f;
		
		final Vec2 d[] = tlD.get(bodies.length);

		for (int i=0; i<bodies.length; ++i) {
			final int prev = (i==0)?bodies.length-1:i-1;
			final int next = (i==bodies.length-1)?0:i+1;
			d[i].set(bodies[next].getMemberWorldCenter());
			d[i].subLocal(bodies[prev].getMemberWorldCenter());
			dotMassSum += (d[i].lengthSquared())/bodies[i].getMass();
			crossMassSum += Vec2.cross(bodies[i].getLinearVelocity(),d[i]);
		}
		final float lambda = -2.0f * crossMassSum / dotMassSum;
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
