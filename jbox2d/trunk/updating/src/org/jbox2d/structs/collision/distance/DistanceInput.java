package org.jbox2d.structs.collision.distance;

import org.jbox2d.common.Transform;

/**
 * Input for Distance.
 * You have to option to use the shape radii
 * in the computation.
 *
 */
public class DistanceInput {
	public DistanceProxy proxyA;
	public DistanceProxy proxyB;
	public final Transform transformA = new Transform();
	public final Transform transformB = new Transform();
	public boolean useRadii;
}
