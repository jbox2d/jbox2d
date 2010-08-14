package org.jbox2d.pooling.arrays;

import java.util.HashMap;

import org.jbox2d.pooling.CustThreadLocal;

public abstract class DynamicTLArray<I> {
	
	private static class TLHashMap<K, V> extends CustThreadLocal<HashMap<K, V>>{
		protected HashMap<K, V> initialValue(){
			return new HashMap<K, V>();
		}
	}
	
	private final TLHashMap<Integer, I[]> tlMap = new TLHashMap<Integer, I[]>();
	
	public I[] get( int argLength){
		assert(argLength > 0);
		
		HashMap<Integer, I[]> map = tlMap.get();
		
		if(!map.containsKey(argLength)){
			map.put(argLength, getInitializedArray(argLength));
		}
		
		assert(map.get(argLength).length == argLength) : "Array not built of correct length";
		return map.get(argLength);
	}
	
	protected abstract I[] getInitializedArray(int argLength);
}
