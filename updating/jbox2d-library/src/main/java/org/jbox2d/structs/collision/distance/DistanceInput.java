package org.jbox2d.structs.collision.distance;

import org.jbox2d.common.Transform;

/**
 * Input for Distance.
 * You have to option to use the shape radii
 * in the computation.
 *
 */
public class DistanceInput {
	public DistanceProxy proxyA = new DistanceProxy();
	public DistanceProxy proxyB = new DistanceProxy();
	public Transform transformA = new Transform();
	public Transform transformB = new Transform();
	public boolean useRadii;
}
