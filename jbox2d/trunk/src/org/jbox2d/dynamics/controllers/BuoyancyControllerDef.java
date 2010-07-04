/**
 * 
 */
package org.jbox2d.dynamics.controllers;

import org.jbox2d.common.Vec2;

/**
 * @author eric
 *
 */
public class BuoyancyControllerDef extends ControllerDef {
	/// The outer surface normal
	public Vec2 normal;
	/// The height of the fluid surface along the normal
	public float offset;
	/// The fluid density
	public float density;
	/// Fluid velocity, for drag calculations
	public Vec2 velocity;
	/// Linear drag co-efficient
	public float linearDrag;
	/// Linear drag co-efficient
	public float angularDrag;
	/// If false, bodies are assumed to be uniformly dense, otherwise use the shapes densities
	public boolean useDensity; //False by default to prevent a gotcha
	/// If true, gravity is taken from the world instead of the gravity parameter.
	public boolean useWorldGravity;
	/// Gravity vector, if the world's gravity is not used
	public Vec2 gravity;

	public BuoyancyControllerDef() {
		normal = new Vec2(0,1);
		offset = 0;
		density = 0;
		velocity = new Vec2(0,0);
		linearDrag = 0;
		angularDrag = 0;
		useDensity = false;
		useWorldGravity = true;
		gravity = new Vec2(0,0);
	}

	/**
	 * @see org.jbox2d.dynamics.controllers.ControllerDef#create()
	 */
	@Override
	public Controller create() {
		return new BuoyancyController(this);
	}
}
