package org.jbox2d.dynamics;

import org.jbox2d.collision.shapes.Shape;

// updated to rev 100
/**
 * A fixture definition is used to create a fixture. This class defines an
 * abstract fixture definition. You can reuse fixture definitions safely.
 *
 * @author daniel
 */
public class FixtureDef {
	/**
	 * The shape, this must be set. The shape will be cloned, so you
	 * can create the shape on the stack.
	 */
	public Shape shape = null;
	
	/**
	 * Use this to store application specific fixture data.
	 */
	public Object userData;
	
	/**
	 * The friction coefficient, usually in the range [0,1].
	 */
	public float friction;
	
	/**
	 * The restitution (elasticity) usually in the range [0,1].
	 */
	public float restitution;
	
	/**
	 * The density, usually in kg/m^2
	 */
	public float density;
	
	/**
	 * A sensor shape collects contact information but never generates a collision
	 * response.
	 */
	public boolean isSensor;
	
	/**
	 * Contact filtering data;
	 */
	public Filter filter;
	
	public FixtureDef(){
		shape = null;
		userData = null;
		friction = 0.2f;
		restitution = 0f;
		density = 0f;
		filter = new Filter();
		filter.categoryBits = 0x0001;
		filter.maskBits = 0xFFFF;
		filter.groupIndex = 0;
		isSensor = false;
	}
}
