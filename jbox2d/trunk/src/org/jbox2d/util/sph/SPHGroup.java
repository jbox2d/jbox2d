package org.jbox2d.util.sph;

import org.jbox2d.collision.AABB;
import org.jbox2d.common.Vec2;
import processing.core.PApplet;

//Some default values are in Master.cpp

public class SPHGroup {
	public SmoothParticle[] t;
	public Parameter param;
	
	public SPHGroup(int nParts, AABB aabb, PApplet parent, float tStep) {
		// set the parameters to some default values
		param = new Parameter();
		param.numPts 		= nParts;
		param.initVel = new Vec2(0.0f,0.0f);		
		param.machNum      	= 0.5;		
		param.initDensity	= 1000.0;	
		param.initPressure = 10000.0f;
		param.nu 		= 1000.0;
		param.deltaT		= tStep;
		param.numSteps		= 500000;
		param.outputEvery	= 5000;
		param.firstOutput	= 0;
		param.densityVariation   = 0.3;
		param.lengthScale	= 1000.0;
		param.bodyFX 		= 0.0;
		param.bodyFY		= 0.0;
		param.c = 1.0f;
		param.h = 1000.0f;
		t = new SmoothParticle[nParts];
		for (int i=0; i<nParts; ++i) {
			float x = parent.random(aabb.lowerBound.x,aabb.upperBound.x);
			float y = parent.random(aabb.lowerBound.y,aabb.upperBound.y);
			t[i] = new SmoothParticle();
			t[i].pos.set(new Vec2(x,y));
			t[i].vel.set(param.initVel);
			t[i].mass = (1.0);
			t[i].setDensity(param.initDensity);
			t[i].setPressure(param.initPressure);
			t[i].setSmoothingLength(param.h);
			t[i].setSpeedSound(param.c);
			t[i].undeleteParticle();
			t[i].ID = i;
			t[i].setShapeID(-1);
		}
	}
	
	/**
	 * This method calculates the pressure of each particle
	 */ 
	public void calcPressure() {	
		for(int i = 0; i < t.length; i++) {
			if (!t[i].isEmpty()) {
				t[i].setMinMaxDensity(); // remember our min and max density
				t[i].calcPressure(param);
			}
		}
//		for (int i=0; i<numShapePts; i++) {
//			shapePtsT[i].calcPressure(param);
//		}
	}
	
	/**
	 * Calculates SPH density/velocity contributions to p1 as a result of p2.
	 */
	public void calcSPHEquations(SmoothParticle p1, SmoothParticle p2) {

		if (p1.isEmpty() || p2.isEmpty()) System.out.println("Panic!  Particle is empty!");

		Vec2 v_aB = new Vec2((p1.vel.x - p2.vel.x),	(p1.vel.y - p2.vel.y));
		p1.calcChangeDensity(p2, v_aB);
		p1.calcChangeVelocity(p2, v_aB, param);

		p1.addForceX(param);
		p1.addForceY(param);
	}
	
	/**
	 * Calculates the correct timestep given the current setup of the
	 * system, prints a message if the current time step is too large.
	 */
	public void calcCorrectDeltaT() {
		int i = 0;
		double accelX = 0;
		double accelY = 0;
		double maxAccelSquared = 0;
		double accelSquared = 0;
		
		maxAccelSquared = 0;
		
		for(i = 0; i < t.length; i++) {
			if (!t[i].isEmpty()) {
				accelX = t[i].getChangeVelocityX();
				accelY = t[i].getChangeVelocityY();
				accelSquared = accelX*accelX + accelY*accelY;
				if (accelSquared > maxAccelSquared)
					maxAccelSquared = accelSquared;
			}	
		}


		double calcDeltaT = 10000000;
		if(maxAccelSquared > 0.0) {
			calcDeltaT = 0.25*Math.sqrt(param.h / Math.sqrt(maxAccelSquared));
		}
		
		if(calcDeltaT < param.deltaT) {
			System.out.println("current DT: " + param.deltaT + " is not small enough.");
			System.out.println("It should be: " + calcDeltaT);
		}
	}
	
	
	/**
	 * Handle a time step, including SPH forces
	 */ 
	public void updateParticles() {
		calcPressure();

		// Interact
		for (int i=0; i<t.length; ++i) {
			t[i].zeroSPHVars();
			for (int j=0; j<t.length; ++j) {
				if (i==j) continue;
				calcSPHEquations(t[i],t[j]);
			}
		}
		
		float dt = (float)param.deltaT;
		// Integrate
		for (int i=0; i<t.length; ++i) {
			SmoothParticle p = t[i];
			p.vel.x += (float)p.getChangeVelocityX()*dt;
			p.vel.y += (float)p.getChangeVelocityY()*dt;
			p.density += (float)p.getChangeDensity()*dt;
			p.pos.x += p.vel.x*dt;
			p.pos.y += p.vel.y*dt;
			System.out.println(p.getChangeVelocityY());
		}
		
	}	

}
