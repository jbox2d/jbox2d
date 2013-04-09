/*******************************************************************************
 * Copyright (c) 2013, Daniel Murphy
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 	* Redistributions of source code must retain the above copyright notice,
 * 	  this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright notice,
 * 	  this list of conditions and the following disclaimer in the documentation
 * 	  and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
/**
 * Created at 4:32:38 AM Jan 14, 2011
 */
package org.jbox2d.utests;

import java.util.Random;

import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;

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
	
	public void testVec2(){
		Vec2 v = new Vec2();
		v.x = 0;
		v.y = 1;
		v.subLocal(new Vec2(10,10));
		assertEquals(-10f, v.x);
		assertEquals(-9f, v.y);
		
		Vec2 v2 = v.add(new Vec2(1,1));
		assertEquals(-9f, v2.x);
		assertEquals(-8f, v2.y);
		assertFalse(v.equals(v2));
		
		// TODO write tests for the rest of common lib
	}
}
