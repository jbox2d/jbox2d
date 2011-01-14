/**
 * Created at 4:32:38 AM Jan 14, 2011
 */
package org.jbox2d.utests;

import java.util.Random;

import org.jbox2d.common.MathUtils;

import junit.framework.TestCase;

/**
 * @author Daniel Murphy
 */
public class MathTest extends TestCase {
	
	private final static int MAX = (int)(Float.MAX_VALUE/1000);
	private final static int RAND_ITERS = 100;
	
	public void testFastMath() {
		Random r = new Random();
		for (int i = 0; i < RAND_ITERS; i++) {
			float a = r.nextFloat() * MAX - MAX/2;
			assertEquals((int)Math.floor(a), MathUtils.floor(a));
		}
		
		for (int i = 0; i < RAND_ITERS; i++) {
			float a = r.nextFloat() * MAX - MAX/2;
			assertEquals((int)Math.ceil(a), MathUtils.ceil(a));
		}
		
		for (int i = 0; i < RAND_ITERS; i++) {
			float a = r.nextFloat() * MAX - MAX/2;
			float b = r.nextFloat() * MAX - MAX/2;
			assertEquals(Math.max(a, b), MathUtils.max(a, b));
		}
		
		for (int i = 0; i < RAND_ITERS; i++) {
			float a = r.nextFloat() * MAX - MAX/2;
			float b = r.nextFloat() * MAX - MAX/2;
			assertEquals(Math.min(a, b), MathUtils.min(a, b));
		}
		
		for (int i = 0; i < RAND_ITERS; i++) {
			float a = r.nextFloat() * MAX - MAX/2;
			assertEquals(Math.round(a), MathUtils.round(a));
		}
		
		for (int i = 0; i < RAND_ITERS; i++) {
			float a = r.nextFloat() * MAX - MAX/2;
			assertEquals(Math.abs(a), MathUtils.abs(a));
		}
	}
}
