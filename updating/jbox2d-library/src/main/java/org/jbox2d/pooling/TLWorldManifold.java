package org.jbox2d.pooling;

import org.jbox2d.collision.WorldManifold;

public class TLWorldManifold extends ThreadLocal<WorldManifold>{
	protected final WorldManifold initialValue(){
		return new WorldManifold();
	}
}
