package org.jbox2d.collision.structs;

import org.jbox2d.common.Vec2;

/**
 * Output for Distance.
 * @author Daniel
 *
 */
public class DistanceOutput {
	public final Vec2 pointA = new Vec2();
	public final Vec2 pointB = new Vec2();
	public float distance;
	public int iterations;
}
