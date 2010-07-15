/**
 * Created at 3:43:53 AM Jul 7, 2010
 */
package org.jbox2d.callbacks;

import org.jbox2d.common.Settings;

// updated to rev 100

/**
 * Contact impulses for reporting. Impulses are used instead of forces because
 * sub-step forces may approach infinity for rigid body collisions. These
 * match up one-to-one with the contact points in b2Manifold.
 * @author Daniel Murphy
 */
public class ContactImpulse {
	public float[] normalImpulses = new float[Settings.maxManifoldPoints];
	public float[] tangentImpulses = new float[Settings.maxManifoldPoints];
}
