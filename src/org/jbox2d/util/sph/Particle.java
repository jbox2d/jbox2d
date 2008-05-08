package org.jbox2d.util.sph;

import org.jbox2d.common.Vec2;

public class Particle {

	public Vec2 pos;
	public Vec2 vel;
	public double mass;
	protected int deleted;
	
	public Particle() {
		pos = new Vec2();
		vel = new Vec2();
		mass = 0.0;
		deleted = 1;
	}
		
	public Particle(Vec2 p, Vec2 v, double m) {
		pos = p.clone();
		vel = v.clone();
		mass = m;
	}
		
	public Particle(Particle P) {
		pos = P.pos.clone();
		vel = P.vel.clone();
		mass = P.mass;
		deleted = P.deleted;
	}

	public void deleteParticle()		       { deleted = 1; }
	public void undeleteParticle()		       { deleted = 0; }
	public boolean isEmpty()		      	   { return ((deleted!=0)?true:false); }
		
}

