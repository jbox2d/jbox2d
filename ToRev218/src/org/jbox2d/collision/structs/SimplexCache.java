package org.jbox2d.collision.structs;

/**
 * Used to warm start b2Distance.
 * Set count to zero on first call.
 * @author Daniel
 *
 */
public class SimplexCache {
	/**
	 * length or area
	 */
	public float metric;
	public short count;
	public final int indexA[] = new int[3];
	public final int indexB[] = new int[3];

	public void set(SimplexCache sc){
		System.arraycopy(sc.indexA, 0, indexA, 0, indexA.length);
		System.arraycopy(sc.indexB, 0, indexB, 0, indexB.length);
		metric = sc.metric;
		count = sc.count;
	}
}
