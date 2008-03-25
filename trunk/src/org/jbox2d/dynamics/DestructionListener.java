package org.jbox2d.dynamics;

import org.jbox2d.collision.Shape;
import org.jbox2d.dynamics.joints.Joint;

public interface DestructionListener {
	/// Joints and shapes are destroyed when their associated
	/// body is destroyed. Implement this listener so that you
	/// may nullify references to these joints and shapes.
	
	/// Called when any joint is about to be destroyed due
	/// to the destruction of one of its attached bodies.
	public void sayGoodbye(Joint joint);

	/// Called when any shape is about to be destroyed due
	/// to the destruction of its parent body.
	public void sayGoodbye(Shape shape);
	
}
