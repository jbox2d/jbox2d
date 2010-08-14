/**
 * Created at 4:14:34 AM Jul 17, 2010
 */
package org.jbox2d.pooling.arrays;

import java.util.HashMap;

import org.jbox2d.pooling.CustThreadLocal;

/**
 * @author Daniel Murphy
 */
public class IntArray {

	private static class TLHashMap<K, V> extends CustThreadLocal<HashMap<K, V>>{
		protected HashMap<K, V> initialValue(){
			return new HashMap<K, V>();
		}
	}
	
	private final TLHashMap<Integer, int[]> tlMap = new TLHashMap<Integer, int[]>();
	
	public int[] get( int argLength){
		assert(argLength > 0);
		
		HashMap<Integer, int[]> map = tlMap.get();
		
		if(!map.containsKey(argLength)){
			map.put(argLength, getInitializedArray(argLength));
		}
		
		assert(map.get(argLength).length == argLength) : "Array not built of correct length";
		return map.get(argLength);
	}
	
	protected int[] getInitializedArray(int argLength){
		return new int[argLength];
	}
}
