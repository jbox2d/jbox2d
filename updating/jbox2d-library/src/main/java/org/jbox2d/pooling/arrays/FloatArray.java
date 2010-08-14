package org.jbox2d.pooling.arrays;

import java.util.HashMap;

import org.jbox2d.pooling.CustThreadLocal;

public class FloatArray {

	private static class TLHashMap<K, V> extends CustThreadLocal<HashMap<K, V>>{
		protected HashMap<K, V> initialValue(){
			return new HashMap<K, V>();
		}
	}
	
	private final TLHashMap<Integer, float[]> tlMap = new TLHashMap<Integer, float[]>();
	
	public float[] get( int argLength){
		assert(argLength > 0);
		
		HashMap<Integer, float[]> map = tlMap.get();
		
		if(!map.containsKey(argLength)){
			map.put(argLength, getInitializedArray(argLength));
		}
		
		assert(map.get(argLength).length == argLength) : "Array not built of correct length";
		return map.get(argLength);
	}
	
	protected float[] getInitializedArray(int argLength){
		return new float[argLength];
	}
}
