package org.jbox2d.pooling;

import org.jbox2d.collision.Manifold;

public class TLManifold extends CustThreadLocal<Manifold> {
	protected Manifold initialValue(){
		return new Manifold();
	}
}
