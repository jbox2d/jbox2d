package org.jbox2d.structs.collision.distance;


/**
 *  Used to warm start b2Distance.
 *  Set count to zero on first call.
 * @author daniel
 */
public class SimplexCache {
	/**
	 * length or area
	 */
	public float metric;
	
	public short count;
	
	/**
	 * vertices on shape A
	 */
	public final int indexA[] = new int[3];
	
	/**
	 * vertices on shape B
	 */
	public final int indexB[] = new int[3];

	public void set(SimplexCache sc){
		System.arraycopy(sc.indexA, 0, indexA, 0, indexA.length);
		System.arraycopy(sc.indexB, 0, indexB, 0, indexB.length);
		metric = sc.metric;
		count = sc.count;
	}
}
