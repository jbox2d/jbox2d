package org.jbox2d.collision.shapes;

import org.jbox2d.collision.FilterData;
import org.jbox2d.collision.structs.ShapeType;

public class ShapeDef {
	
	/**
	 *  Holds the shape type for down-casting.
	 */
	public ShapeType type;

	/**
	 * Use this to store application specify shape data.
	 */
	public Object userData;

	/**
	 * The shape's friction coefficient, usually in the range [0,1].
	 */
	public float friction;

	/**
	 * The shape's restitution (elasticity) usually in the range [0,1].
	 */
	public float restitution;

	/**
	 * The shape's density, usually in kg/m^2.
	 */
	public float density;

	/**
	 * A sensor shape collects contact information but never generates a collision
	 * response.
	 */
	public boolean isSensor;

	/**
	 * Contact filtering data.
	 */
	public FilterData filter;
	
	/// The constructor sets the default shape definition values.
	public ShapeDef(){
		type = ShapeType.UNKNOWN_SHAPE;
		userData = null;
		friction = 0.2f;
		restitution = 0.0f;
		density = 0.0f;
		filter.categoryBits = 0x0001;
		filter.maskBits = 0xFFFF;
		filter.groupIndex = 0;
		isSensor = false;
	}
}
