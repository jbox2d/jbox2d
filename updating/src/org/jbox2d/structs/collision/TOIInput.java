package org.jbox2d.structs.collision;

import org.jbox2d.common.Sweep;
import org.jbox2d.structs.collision.distance.DistanceProxy;

/**
 * Input parameters for TOI
 */
public class TOIInput {
	public DistanceProxy proxyA;
	public DistanceProxy proxyB;
	public Sweep sweepA;
	public Sweep sweepB;
	public float tolerance;
}
