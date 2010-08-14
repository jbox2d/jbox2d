package org.jbox2d.pooling;

import org.jbox2d.common.Settings;

public class CustThreadLocal<T> extends ThreadLocal<T> {
	
	// FIXME delete this for release, just here for debugging.
	@Override
	public T get() {
		if(Settings.POOLING){
			return super.get();
		}else{
			return this.initialValue();
		}
	}
}
