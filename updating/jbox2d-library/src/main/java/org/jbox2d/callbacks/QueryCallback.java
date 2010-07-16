/**
 * Created at 4:30:03 AM Jul 15, 2010
 */
package org.jbox2d.callbacks;

import org.jbox2d.dynamics.Fixture;

// update to rev 100
/**
 * Callback class for AABB queries.
 * See World.query
 * @author Daniel Murphy
 */
public interface QueryCallback {

	/**
	 * Called for each fixture found in the query AABB.
	 * @param fixture
	 * @return false to terminate the query.
	 */
	public boolean reportFixture(Fixture fixture);
}
