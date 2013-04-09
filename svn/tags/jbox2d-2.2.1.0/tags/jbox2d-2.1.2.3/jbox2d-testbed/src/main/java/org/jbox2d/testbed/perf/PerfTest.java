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
 * Created at 8:12:11 AM Jan 18, 2011
 */
package org.jbox2d.testbed.perf;


/**
 * @author Daniel Murphy
 */
public abstract class PerfTest {

	private final int numTests, iters;
	private final long[] times;
	
	public PerfTest(int argNumTests, int argIters){
		numTests = argNumTests;
		iters = argIters;
		times = new long[numTests];
		for(int i=0; i<numTests; i++){
			times[i] = 0;
		}
	}
	
	public void go(){
		long prev, after;
		for(int i=0; i<iters; i++){
			System.out.println(i*100.0/iters + "%");
			for(int test=0; test<numTests; test++){
				prev = System.nanoTime();
				runTest(test);
				after = System.nanoTime();
				times[test] += after-prev;
			}
		}
		for(int test=0; test<numTests; test++){
			times[test] /= iters;
		}
		printResults();
	}
	
	public void printResults(){
		System.out.printf("%-20s%20s\n","Test Name", "Milliseconds Avg");

		for(int i=0; i<numTests; i++){
			double milliseconds = times[i]* 1.0 / 1000000;
			System.out.printf("%-20s%20.4f\n",getTestName(i), milliseconds);
		}
	}
	
	public abstract void runTest(int argNum);
	
	public abstract String getTestName(int argNum);
}
