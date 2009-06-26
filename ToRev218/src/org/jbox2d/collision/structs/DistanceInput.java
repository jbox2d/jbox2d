package org.jbox2d.collision.structs;

import org.jbox2d.common.XForm;

/**
 * Input for Distance.
 * You have to option to use the shape radii
 * in the computation. Even 
 * @author Daniel
 *
 */
public class DistanceInput {
	public final XForm transformA = new XForm();
	public final XForm transformB = new XForm();
	public boolean useRadii;
}
