/**
 * Created at 4:23:30 AM Jul 15, 2010
 */
package org.jbox2d.callbacks;

import org.jbox2d.dynamics.Fixture;

/**
 * Joints and fixtures are destroyed when their associated
 * body is destroyed. Implement this listener so that you
 * may nullify references to these joints and shapes.
 * @author Daniel Murphy
 */
public interface DestructionListener {
	
	/**
	 * Called when any joint is about to be destroyed due
	 * to the destruction of one of its attached bodies.
	 * @param joint
	 */
	public void sayGoodbye(Joint joint);
	
	/**
	 * Called when any fixture is about to be destroyed due
	 * to the destruction of its parent body.
	 * @param fixture
	 */
	public void sayGoodbye(Fixture fixture);
}
