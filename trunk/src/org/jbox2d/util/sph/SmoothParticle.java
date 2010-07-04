/*
 * JBox2D Smoothed Particle Hydrodynamics
 * JBox2D home: http://www.jbox2d.org
 * Box2D home: http://www.box2d.org
 * 
 * This file is part of a Java implementation of C++ code 
 * available from http://www.keithv.com/project/index.html.
 * Many thanks to Keith Vertanen and Brian Schlatter for letting us
 * integrate this code and make it available under a zlib license:
 * 
 * ==========================================================================
 * Copyright (c) 1999-2008 Keith Vertanen and Brian Schlatter
 * 
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 * ==========================================================================
 */

package org.jbox2d.util.sph;

import org.jbox2d.common.Vec2;

public class SmoothParticle extends Particle {
	
	public int ID;
	
	protected double pressure;
	protected double density;
	protected double h;
	protected double c;
	protected double ChangeDensity;
	protected Vec2 ChangeVelocity;
	protected int shapeID;		
	protected int numNeighbors;	
	protected double minDensity;
	protected double maxDensity;
	
	
	public SmoothParticle() {
		super();
		pressure = 0.0;
		density = 0.0;
		h = 0.0;
		c = 0.0;
		ChangeDensity = 0.0;
		ChangeVelocity = new Vec2(0.0f, 0.0f);
		ID = 0;
		shapeID = -1;
		minDensity = 999999.0;
		maxDensity = -999999.0;
		numNeighbors = 0;
	}
	
	public SmoothParticle(Vec2 p, Vec2 v, double m, double d, double pr, 
			           double len, double speedSnd) {
		super(p, v, m);
		pressure = pr;
		density = d;
		h = len;
		c = speedSnd; 
		ChangeDensity = 0.0;
		ChangeVelocity = new Vec2(0.0f,0.0f);
		shapeID = -1;
		ID = 0;
		minDensity = 999999.0;
		maxDensity = -999999.0;
		numNeighbors = 0;
	}
	
	public SmoothParticle(SmoothParticle SP) {
		pos = SP.pos.clone(); 
		vel = SP.vel.clone();
		mass = SP.mass; 
		density = SP.getDensity();
		pressure = SP.getPressure();
		h = SP.getSmoothingLength();
		c = SP.getSpeedSound();
		minDensity = SP.getMinDensity();
		maxDensity = SP.getMaxDensity();
		ChangeVelocity = SP.getChangeVelocity();
		ChangeDensity = SP.getChangeDensity();
		deleted = SP.deleted;
		ID = SP.ID;
		shapeID = SP.shapeID;
		numNeighbors = SP.numNeighbors;
	}
	
	// Getters and setters
		
	public void setPressure(double p)            	{ pressure = p;           		}
	public void setDensity(double den)           	{ density = den;          		}
	public void setSmoothingLength(double len)   	{ h = len;                		}
	public void setSpeedSound(double spSnd)      	{ c = spSnd;              		}
	public void setChangeVelocity(Vec2 ChangeV) 	{ ChangeVelocity.set(ChangeV);  }
	public void setChangeDensity(double cd)      	{ ChangeDensity = cd;     		}
	public void setMinDensity(double m)		  		{ minDensity = m;  		  		}
	public void setMaxDensity(double m)		  		{ maxDensity = m;  		  		}
	public void setShapeID(int id)		          	{ shapeID = id;   		  		}		
	public void setNumNeighbors(int n)	      		{ numNeighbors = n;   	  		}		

	public double getPressure()        	 	 		{ return pressure;        		}
	public double getSmoothingLength() 		 		{ return h;               		}
	public double getDensity()         		 		{ return density;         		}
	public double getSpeedSound()      		 		{ return c;               		}
	public double getChangeDensity()   		 		{ return ChangeDensity;   		}
	public Vec2 getChangeVelocity()   		 		{ return ChangeVelocity;  		}
	public double getChangeVelocityX() 		 		{ return ChangeVelocity.x;		}
	public double getChangeVelocityY() 		 		{ return ChangeVelocity.y;		}
	public int getShapeID() 	     		 		{ return shapeID; 		  		}
	public double getMinDensity()	    	 		{ return minDensity;      		}
	public double getMaxDensity()	    	 		{ return maxDensity;      		}
	public int getNumNeighbors()	    	 		{ return numNeighbors;    		}

	/**
	 * Calculates the pressure for this SPH particle.
	 * The state equation comes from **** and is used to model water.
	 */
	public void calcPressure(Parameter p) {
		// this form is given in paper by Morris.
		pressure = p.c * p.c * density;
	}
	
	/**
	 * Zeroes out the delta values. The reason is because the delta
	 * values come from summing up contributions from all neighbors and
	 * we don't want to add to the last time steps value.
	 */
	public void zeroSPHVars() {
		ChangeDensity = 0.0;
		ChangeVelocity.set(0.0f, 0.0f);
		numNeighbors = 0;
	}

	/**
	 * Adds the contributions of the change in density from the neighbor
	 * "sp".
	 */
	public void calcChangeDensity(SmoothParticle sp, Vec2 v_ij) {
		ChangeDensity += sp.mass *  
						(v_ij.x * gradientKernelX(sp) + 
						 v_ij.y * gradientKernelY(sp));
	}	
			
		
	/**
	 * Add the contribution of the acceleration due to the free
	 * particle "sp".
	 */
	public void calcChangeVelocity(SmoothParticle sp, Vec2 v_ij, Parameter param) {
		double pressTerm = pressureTerm(sp);
		double artVisc = artificialViscosity(sp, param);

		double oldChngVelX = ChangeVelocity.x;
		double oldChngVelY = ChangeVelocity.y;
			

		ChangeVelocity.x = (float)(oldChngVelX + sp.mass *
							(pressTerm * gradientKernelX(sp) +
							v_ij.x * artVisc));


		ChangeVelocity.y = (float)(oldChngVelY + sp.mass *
							(pressTerm * gradientKernelY(sp) +
							 v_ij.y * artVisc));
		//System.out.println(pressTerm+" "+gradientKernelY(sp));

	}
	
	/**
	 * Add the body force in the x direction.
	 */
	public void addForceX(Parameter param) {
		ChangeVelocity.x = (float)(ChangeVelocity.x + param.bodyFX);
	}


	/**
	 * Add the body force in the y direction.
	 */
	public void addForceY(Parameter param) {
		ChangeVelocity.x = (float)(ChangeVelocity.y + param.bodyFY);
	}


	/**
	 * Calculate the pressure gradient due to the free particle "sp".
	 */
	private double pressureTerm(SmoothParticle sp) {
		return (-1.0 * (pressure/(density*density)
		    + sp.getPressure() / (sp.getDensity()*sp.getDensity())));
	}

	/**
	 * This is used to simulate viscous fluids.
	 */
	protected double artificialViscosity(SmoothParticle sp, Parameter param) {
		double distance = Math.sqrt((pos.x - sp.pos.x)*
							   (pos.x - sp.pos.x) +
							   (pos.y - sp.pos.y)*
							   (pos.y - sp.pos.y));


		Vec2 r_ij = new Vec2((pos.x - sp.pos.x), 
				   (pos.y - sp.pos.y));

		return (param.nu*(density + sp.getDensity()) *
			(r_ij.x*gradientKernelX(sp) + r_ij.y*gradientKernelY(sp))) /
			((density * sp.getDensity())* (distance*distance+0.01*h*h)); //TODO: .01*h*h?  hmm...
	}
	
	/**
	 * SPH smoothing kernel
	 */
	protected double kernel(SmoothParticle sp) {
		double distance = 0;
		double normalization = 10.0 / (7.0*Math.PI*h*h);	
		
		double dist1 = pos.x - sp.pos.x;
		double dist2 = pos.y - sp.pos.y;

		distance = Math.sqrt(dist1 * dist1 + dist2 * dist2);

		if(distance < h)
			return (normalization) * (1.0 - 
			        1.5*distance*distance / (h*h) + 
					0.75*distance*distance*distance / (h*h*h));
		else if(distance < 2.0 * h)
			return  normalization * (2.0 - 3.0 * distance / h
			     + 1.5*distance*distance / (h*h) - 
				 0.25*distance*distance*distance / (h*h*h));
		else
			return 0.0;
			
	}
	
	/**
	 * X component of gradient of SPH smoothing kernel
	 */
	protected double gradientKernelX(SmoothParticle sp) {
		double distance = 0.0;
		double diffX = 0.0;
		double diffY = 0.0;
		double normalization = 10.0 / (7.0*Math.PI*h*h);	
		
		diffX = pos.x - sp.pos.x;
		diffY = pos.y - sp.pos.y;
		distance = Math.sqrt(diffX*diffX + diffY*diffY);

		if(distance < h)
			return normalization * (-3.0/ (h*h) +
			 9.0*distance / (4.0*h*h*h)) * diffX;
		else if(distance < 2.0*h)
			return normalization * (-3.0/ (h*distance) +
			  3.0/(h*h) -3.0*distance/(4.0*h*h*h)) * diffX;
		else
			return 0.0;
	}
		

	/**
	 * Y component of gradient of SPH smoothing kernel
	 */
	protected double gradientKernelY(SmoothParticle sp) {
		double distance = 0.0;
		double diffX = 0.0;
		double diffY = 0.0;
		double normalization = 10.0 / (7.0*Math.PI*h*h);	
		diffX = pos.x - sp.pos.x;
		diffY = pos.y - sp.pos.y;
		distance = Math.sqrt(diffX*diffX + diffY*diffY);

		if(distance < h)
			return  normalization* (-3.0/ (h*h) + 9.0*distance / (4.0*h*h*h)) * diffY;
		else if(distance < 2.0*h)
			return  normalization* (-3.0/ (h*distance) + 3.0/(h*h) -3.0*distance/(4.0*h*h*h)) * diffY;
		else{
			return 0.0;
		}
	}
	
	/**
	 * Keeps track of our particle's min and max density
	 */
	public void setMinMaxDensity(){
		if (density > maxDensity) maxDensity = density;
		else if (density < minDensity) minDensity = density;
	}
	
	/**
	 * Used to keep a total of the number of neighbors within 2*h of
	 * this smooth particle.  Used for debugging.
	 */
	void updateNumNeighbors(SmoothParticle sp) {
		double distance = Math.sqrt((pos.x - sp.pos.x)*
							   (pos.x - sp.pos.x) +
							   (pos.y - sp.pos.y)*
							   (pos.y - sp.pos.y));

		if (distance < 2*h) numNeighbors++;
	}

}
