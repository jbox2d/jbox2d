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
 * Created at 7:09:57 PM Jan 18, 2011
 */
package org.jbox2d.testbed.perf;

import org.jbox2d.common.MathUtils;
import org.jbox2d.profile.BasicPerformanceTest;

// Results from Mac 1/19/11
//Test Name               Milliseconds Avg
//Sin                              86.2132
//SinLUT                           26.7498
//Pow                              74.5660
//FastPow                          20.9268
//Max                              11.6579
//FastMax                          11.6045
//Floor                             0.0593
//fastFloor                         0.0216
//aTan2                            20.1040
//fastATan2                        18.2086
//ceil                              0.0348
//fastCeil                          0.0215

// Results from Windows 1/19/11
//Test Name               Milliseconds Avg
//Sin                             254.5806
//SinLUT                           57.6727
//Pow                             125.5473
//FastPow                          25.7931
//Max                              16.9723
//FastMax                          16.9692
//Floor                             0.0659
//fastFloor                         0.0206
//aTan2                            31.6952
//fastATan2                        22.9149
//ceil                              0.0476
//fastCeil                          0.0238

/**
 * @author Daniel Murphy
 */
public class MathPerf extends BasicPerformanceTest {

	public static int INNER_ITERS = 500000;
	public static int OUTER_ITERS = 100;
	
	String[] tests = new String[]{
		"Sin", "SinLUT", "Pow", "FastPow", "Max", "FastMax", "Floor", "fastFloor", "aTan2", "fastATan2", "ceil", "fastCeil"
	};
	
	public float aStore = 0;
	
	/**
	 * @param argNumTests
	 * @param argIters
	 */
	public MathPerf() {
		super(12, OUTER_ITERS);
	}

	/**
	 * @see org.jbox2d.testbed.perf.BasicPerformanceTest#runTest(int)
	 */
	@Override
	public void runTest(int argNum) {
		float random = MathUtils.randomFloat(-Float.MAX_VALUE/3, Float.MAX_VALUE/3);
		switch(argNum){
			case 0:
				runSinTest(random);
				break;
			case 1:
				runSinLUTTest(random);
				break;
			case 2:
				runPowTest(random);
				break;
			case 3:
				runFastPowTest(random);
				break;
			case 4:
				runMaxTest(random);
				break;
			case 5:
				runFastMaxTest(random);
				break;
			case 6:
				runFloorTest(random);
				break;
			case 7:
				runFastFloorTest(random);
				break;
			case 8:
				runAtan2Test(random);
				break;
			case 9:
				runFastAtan2Test(random);
				break;
			case 10:
				runCeilTest(random);
				break;
			case 11:
				runFastCeilTest(random);
				break;
		}
	}
	
	public void runSinTest(float argRandom){
		float a = 0;
		for(int i=0; i<INNER_ITERS; i++){
			a = (float)StrictMath.sin(argRandom);
		}
		aStore += a;
	}
	
	public void runSinLUTTest(float argRandom){
		float a = 0;
		for(int i=0; i<INNER_ITERS; i++){
			a = MathUtils.sinLUT(argRandom);
		}
		aStore += a;
	}
	
	public void runPowTest(float argRandom){
		float a = 0;
		for(int i=0; i<INNER_ITERS; i++){
			a = (float)StrictMath.pow(argRandom, MathUtils.randomFloat(-100, 100));
		}
		aStore += a;
	}
	
	public void runFastPowTest(float argRandom){
		float a = 0;
		for(int i=0; i<INNER_ITERS; i++){
			a = MathUtils.fastPow(argRandom, MathUtils.randomFloat(-100, 100));
		}
		aStore += a;
	}
	
	public void runMaxTest(float argRandom){
		float a = 0;
		for(int i=0; i<INNER_ITERS; i++){
			a = StrictMath.max(argRandom, MathUtils.randomFloat(-100, 100));
		}
		aStore += a;
	}
	
	public void runFastMaxTest(float argRandom){
		float a = 0;
		for(int i=0; i<INNER_ITERS; i++){
			a = MathUtils.max(argRandom, MathUtils.randomFloat(-100, 100));
		}
		aStore += a;
	}
	
	public void runFloorTest(float argRandom){
		float a = 0;
		for(int i=0; i<INNER_ITERS; i++){
			a = (float) StrictMath.floor(argRandom);
		}
		aStore += a;
	}
	
	public void runFastFloorTest(float argRandom){
		float a = 0;
		for(int i=0; i<INNER_ITERS; i++){
			a = MathUtils.floor(argRandom);
		}
		aStore += a;
	}
	
	public void runAtan2Test(float argRandom){
		float a = 0;
		for(int i=0; i<INNER_ITERS; i++){
			a = (float)StrictMath.atan2(argRandom, MathUtils.randomFloat(-10000, 10000));
		}
		aStore += a;
	}
	
	public void runFastAtan2Test(float argRandom){
		float a = 0;
		for(int i=0; i<INNER_ITERS; i++){
			a = MathUtils.fastAtan2(argRandom, MathUtils.randomFloat(-10000, 10000));
		}
		aStore += a;
	}
	
	public void runCeilTest(float argRandom){
		float a = 0;
		for(int i=0; i<INNER_ITERS; i++){
			a = (float)StrictMath.ceil(argRandom);
		}
		aStore += a;
	}
	
	public void runFastCeilTest(float argRandom){
		float a = 0;
		for(int i=0; i<INNER_ITERS; i++){
			a = MathUtils.ceil(argRandom);
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
		MathPerf p = new MathPerf();
		p.go();
	}
	
}
