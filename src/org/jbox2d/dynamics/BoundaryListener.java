package org.jbox2d.dynamics;

public interface BoundaryListener {
	/// This is called when a body's shape passes outside of the world boundary.

	/// This is called for each body that leaves the world boundary.
	/// @warning you can't modify the world inside this callback.
	public  void violation(Body body);
}

