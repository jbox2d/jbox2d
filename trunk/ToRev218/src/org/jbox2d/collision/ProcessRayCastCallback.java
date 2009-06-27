package org.jbox2d.collision;

import org.jbox2d.collision.structs.RayCastInput;

/**
 * A callback for ray casts.
 */
public interface ProcessRayCastCallback {
	/**
	 * Process a ray-cast. This allows the client to perform an exact ray-cast
	 * against their object (found from the proxyUserData pointer).
	 * @param input the original ray-cast segment with an adjusted maxFraction.
	 * @param maxFraction the clipping parameter, the ray extends from p1 to p1 + maxFraction * (p2 - p1).
	 * @param userData user data associated with the current proxy.
	 * @return the new max fraction. Return 0 to end the ray-cast. Return the input maxFraction to
	 * continue the ray cast. Return a value less than maxFraction to clip the ray-cast.
	 */
	public float process(RayCastInput input, Object userData);
}
