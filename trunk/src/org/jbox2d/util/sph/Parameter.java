package org.jbox2d.util.sph;

import org.jbox2d.common.Vec2;

// Defaults in Master.cpp

public class Parameter {
	public int numPts;
	public double velocityScale;
	public Vec2 initVel;
	public double initMass;
	public double initDensity;
	public double initPressure;
	public double initPtSpacing;
	public double h;
	public double c;
	public double machNum;
	public double betaMax;
	public double nu;	
	public double bodyFX;
	public double bodyFY;
	public double densityVariation;
	public double lengthScale;
		
	public int firstOutput;		// first step to output a frame
	public int outputEvery;		// how often to output results to master
	public int numSteps;		// how many total steps to run
	public double deltaT;		// change in time per time step

	public Parameter(Parameter p) {
		numPts = p.numPts;
		velocityScale = p.velocityScale;
		initVel = p.initVel;
		initMass = p.initMass;
		initDensity = p.initDensity;
		initPressure = p.initPressure;
		initPtSpacing = p.initPtSpacing;
		h = p.h;
		c = p.c;
		machNum = p.machNum;
		betaMax = p.betaMax;
		nu = p.nu;
		bodyFX = p.bodyFX;
		bodyFY = p.bodyFY;
		densityVariation = p.densityVariation;
		lengthScale = p.lengthScale;
		firstOutput = p.firstOutput;
		outputEvery = p.outputEvery;
		numSteps = p.numSteps;
		deltaT = p.deltaT;
	}

	public Parameter(){
		numPts = 0; 
		velocityScale = 0; 
		initVel = new Vec2(0, 0); 
		initMass = 0; 
		initDensity = 0; 
		initPressure = 0; 
		initPtSpacing = 0; 
		h = 0; 
		c = 0; 
		machNum = 0; 
		deltaT = 0; 
		outputEvery = 0; 
		betaMax = 0; 
		numSteps = 0; 
		firstOutput = 0; 
		nu = 0; 
		bodyFX = 0; 
		bodyFY = 0; 
		densityVariation = 0; 
		lengthScale = 0;
			
	}
	
}
