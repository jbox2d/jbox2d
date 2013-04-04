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
 * Created at 8:05:23 AM Jan 18, 2011
 */
package org.jbox2d.testbed.perf;

import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import org.jbox2d.pooling.IWorldPool;
import org.jbox2d.pooling.normal.DefaultWorldPool;
import org.jbox2d.profile.BasicPerformanceTest;

//Test Name               Milliseconds Avg
//Creation                         70.6609
//World Pool                      251.3508
//Circle Pool                      75.3677
//Custom Stack                     75.6705
//ThreadLocal member               77.2405
//Member                           74.3760

// Windows results 1/19/11
//Test Name               Milliseconds Avg
//Creation                         74.6291
//World Pool                      273.5585
//Circle Pool                      82.2195
//ThreadLocal member               83.2970
//Member                           80.7545

// Windows results 7/5/2011
//Test Name               Milliseconds Avg
//Creation                         79.3003
//World Pool                       82.9722
//Circle Pool                      85.9589
//Custom Stack                     85.5465
//ThreadLocal member               87.7560
//Member                           84.1358


/**
 * @author Daniel Murphy
 */
public class PoolingPerf extends BasicPerformanceTest {
	


	public static final int INNER_ITERS = 50000;
	public static final int OUTER_ITERS = 1000;
	
	public static class CirclePool{
		final Vec2[] pool;
		int index;
		final int length;
		public CirclePool(){
			pool = new Vec2[200];
			for(int i=0; i<pool.length; i++){
				pool[i] = new Vec2();
			}
			length = 200;
			index = -1;
		}
		
		public final Vec2 get(){
			index++;
			if(index >= length){
				index = 0;
			}
			return pool[index];
		}
	}
	
	public static class CustStack{
		final Vec2[] pool;
		int index;
		public CustStack(){
			pool = new Vec2[50];
			for(int i=0; i<pool.length; i++){
				pool[i] = new Vec2();
			}
			index = 0;
		}
		
		public final Vec2 get(){
			return pool[index++];
		}
		
		public final void reduce(int i){
			index -= i;
		}
	}
	
	public static class TLVec2 extends ThreadLocal<Vec2>{
		@Override
		protected Vec2 initialValue() {
			return new Vec2();
		}
	}
	
	public String[] tests = new String[]{
		"Creation", "World Pool", "Circle Pool", "Custom Stack", "ThreadLocal member", "Member"
	};
	
	public float aStore = 0;
	public IWorldPool wp = new DefaultWorldPool(100, 10);
	public CirclePool cp = new CirclePool();
	public TLVec2 tlv = new TLVec2();
	public Vec2 mv = new Vec2();
	public CustStack stack = new CustStack();
	
	/**
	 * @param argNumTests
	 * @param argIters
	 */
	public PoolingPerf() {
		super(6, OUTER_ITERS);
		// TODO Auto-generated constructor stub
	}
	
	public float op(Vec2 argVec){
		argVec.set(MathUtils.randomFloat(-100, 100), MathUtils.randomFloat(-100, 100));
		argVec.mulLocal(3.2f);
		float s = argVec.length();
		argVec.normalize();
		return s;
	}

	/**
	 * @see org.jbox2d.testbed.perf.BasicPerformanceTest#runTest(int)
	 */
	@Override
	public void runTest(int argNum) {
		switch(argNum){
			case 0:
				runCreationTest();
				break;
			case 1:
				runWorldPoolTest();
				break;
			case 2:
				runCirclePoolTest();
				break;
			case 3:
				runCustStackTest();
				break;
			case 4:
				runThreadLocalTest();
				break;
			case 5:
				runMemberTest();
				break;
		}
	}
	
	public void runCreationTest(){
		Vec2 v;
		float a = 0;
		for(int i=0; i<INNER_ITERS; i++){
			v = new Vec2();
			a += op(v);
		}
		aStore += a;
	}

	public void runWorldPoolTest(){
		Vec2 v;
		float a = 0;
		for(int i=0; i<INNER_ITERS; i++){
			v = wp.popVec2();
			a += op(v);
			wp.pushVec2(1);
		}
		aStore += a;
	}
	
	public void runCirclePoolTest(){
		Vec2 v;
		float a = 0;
		for(int i=0; i<INNER_ITERS; i++){
			v = cp.get();
			a += op(v);
		}
		aStore += a;
	}
	
	public void runThreadLocalTest(){
		Vec2 v;
		float a = 0;
		for(int i=0; i<INNER_ITERS; i++){
			v = tlv.get();
			a += op(v);
		}
		aStore += a;
	}
	
	public void runCustStackTest(){
		Vec2 v;
		float a = 0;
		for(int i=0; i<INNER_ITERS; i++){
			v = stack.get();
			a += op(v);
			stack.reduce(1);
		}
		aStore += a;
	}
	
	public void runMemberTest(){
		float a = 0;
		for(int i=0; i<INNER_ITERS; i++){
			a += op(mv);
		}
		aStore += a;
	}


	/**
	 * @see org.jbox2d.testbed.perf.BasicPerformanceTest#getTestName(int)
	 */
	@Override
	public String getTestName(int argNum) {
		return tests[argNum];
	}
	
	public static void main(String[] c){
		PoolingPerf p = new PoolingPerf();
		p.go();
	}
}
