package org.jbox2d.pooling;

import org.jbox2d.collision.Distance;

public class TLDistance extends ThreadLocal<Distance> {
	protected Distance initialValue(){
		return new Distance();
	}
}
