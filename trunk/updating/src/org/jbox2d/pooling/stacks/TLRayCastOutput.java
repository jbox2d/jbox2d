package org.jbox2d.pooling.stacks;

import org.jbox2d.structs.collision.RayCastOutput;

public class TLRayCastOutput extends ThreadLocal<RayCastOutput> {
	protected RayCastOutput initialValue(){
		return new RayCastOutput();
	}
}
