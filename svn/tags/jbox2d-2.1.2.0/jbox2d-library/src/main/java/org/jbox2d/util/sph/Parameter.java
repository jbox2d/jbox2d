/*******************************************************************************
 * Copyright (c) 2011, Daniel Murphy
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL DANIEL MURPHY BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
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
