package org.jbox2d.collision;

import org.jbox2d.collision.structs.RayCastInput;
import org.jbox2d.collision.structs.RayCastOutput;

public interface RayCastCallback {

	public void rayCastCallback(RayCastOutput output, RayCastInput subInput, Object userData);
}
