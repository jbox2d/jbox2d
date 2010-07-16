/**
 * Created at 4:25:42 AM Jul 15, 2010
 */
package org.jbox2d.callbacks;

import org.jbox2d.dynamics.Filter;
import org.jbox2d.dynamics.Fixture;

// updated to rev 100
/**
 * Implement this class to provide collision filtering. In other words, you can implement
 * this class if you want finer control over contact creation.
 * @author Daniel Murphy
 */
public class ContactFilter {

	/**
	 * Return true if contact calculations should be performed between these two shapes.
	 * @warning for performance reasons this is only called when the AABBs begin to overlap.
	 * @param fixtureA
	 * @param fixtureB
	 * @return
	 */
	public boolean shouldCollide(Fixture fixtureA, Fixture fixtureB){
		Filter filterA = fixtureA.getFilterData();
		Filter filterB = fixtureB.getFilterData();

		if (filterA.groupIndex == filterB.groupIndex && filterA.groupIndex != 0){
			return filterA.groupIndex > 0;
		}

		boolean collide = (filterA.maskBits & filterB.categoryBits) != 0 &&
						  (filterA.categoryBits & filterB.maskBits) != 0;
		return collide;
	}
}
