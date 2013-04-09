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
 * Created at 6:24:51 AM Jan 20, 2011
 */
package org.jbox2d.testbed.perf;

import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import org.jbox2d.pooling.IWorldPool;
import org.jbox2d.pooling.normal.DefaultWorldPool;
import org.jbox2d.profile.BasicPerformanceTest;


//Test Name               Milliseconds Avg
//Pop2Sep                           6.9922
//Pop2Cont                          7.0805
//Pop3Sep                           9.9359
//Pop3Cont                         10.1472
//Pop4Sep                          13.8380
//Pop4Cont                         14.0874
//Pop9Sep                          31.5494
//Pop9Cont                         31.0169

/**
 * @author Daniel Murphy
 */
public class StackTest extends BasicPerformanceTest {
	
	public static final int INNER_ITERS = 50000;
	public static final int OUTER_ITERS = 200;
	
	public static String[] tests = new String[]{
		"Pop2Sep", "Pop2Cont", "Pop3Sep", "Pop3Cont", "Pop4Sep", "Pop4Cont", "Pop9Sep", "Pop9Cont"
	};
 	public static float aStore = 0;
	
	public StackTest(){
		super(8, OUTER_ITERS);
	}
	
	
	public float op(Vec2 argVec){
		argVec.set(MathUtils.randomFloat(-100, 100), MathUtils.randomFloat(-100, 100));
		argVec.mulLocal(3.2f);
		float s = argVec.length();
		argVec.normalize();
		return s;
	}
	
	private final IWorldPool wp = new DefaultWorldPool(100, 10);

	
	/**
	 * @see org.jbox2d.testbed.perf.BasicPerformanceTest#runTest(int)
	 */
	@Override
	public void runTest(int argNum) {
		
		float a = 0;
		for(int i=0; i<INNER_ITERS; i++){
			switch(argNum){
				case 0:{
					final Vec2 v1 = wp.popVec2();
					final Vec2 v2 = wp.popVec2();
					a += op(v1);
					a += op(v2);
					wp.pushVec2(2);
					break;}
				case 1:{
					final Vec2[] pc = wp.popVec2(2);
					a += op(pc[0]);
					a += op(pc[1]);
					wp.pushVec2(2);
					break;}
				case 2:{
					final Vec2 v1 = wp.popVec2();
					final Vec2 v2 = wp.popVec2();
					final Vec2 v3 = wp.popVec2();
					a += op(v1);
					a += op(v2);
					a += op(v3);
					wp.pushVec2(3);
					break;}
				case 3:{
					final Vec2[] pc = wp.popVec2(3);
					a += op(pc[0]);
					a += op(pc[1]);
					a += op(pc[2]);
					wp.pushVec2(3);
					break;}
				case 4:{
					final Vec2 v1 = wp.popVec2();
					final Vec2 v2 = wp.popVec2();
					final Vec2 v3 = wp.popVec2();
					final Vec2 v4 = wp.popVec2();
					a += op(v1);
					a += op(v2);
					a += op(v3);
					a += op(v4);
					wp.pushVec2(4);
					break;}
				case 5:{
					final Vec2[] pc = wp.popVec2(4);
					a += op(pc[0]);
					a += op(pc[1]);
					a += op(pc[2]);
					a += op(pc[3]);
					wp.pushVec2(4);}
					break;
				case 6:{
					final Vec2 v1 = wp.popVec2();
					final Vec2 v2 = wp.popVec2();
					final Vec2 v3 = wp.popVec2();
					final Vec2 v4 = wp.popVec2();
					final Vec2 v5 = wp.popVec2();
					final Vec2 v6 = wp.popVec2();
					final Vec2 v7 = wp.popVec2();
					final Vec2 v8 = wp.popVec2();
					final Vec2 v9 = wp.popVec2();
					a += op(v1);
					a += op(v2);
					a += op(v3);
					a += op(v4);
					a += op(v5);
					a += op(v6);
					a += op(v7);
					a += op(v8);
					a += op(v9);
					wp.pushVec2(9);
					break;}
				case 7:{
					final Vec2[] pc = wp.popVec2(9);
					a += op(pc[0]);
					a += op(pc[1]);
					a += op(pc[2]);
					a += op(pc[3]);
					a += op(pc[4]);
					a += op(pc[5]);
					a += op(pc[6]);
					a += op(pc[7]);
					a += op(pc[8]);
					wp.pushVec2(9);
					break;}
			}
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
		(new StackTest()).go();
	}
	
}
