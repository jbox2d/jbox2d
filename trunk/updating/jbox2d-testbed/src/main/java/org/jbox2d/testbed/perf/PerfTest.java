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
		System.out.printf("%20s%20s\n","Test Name", "Milliseconds Avg");

		for(int i=0; i<numTests; i++){
			double milliseconds = (times[i] / 1000) * 1.0 / 1000;
			System.out.printf("%20s%20.4f\n",getTestName(i), milliseconds);
		}
	}
	
	public abstract void runTest(int argNum);
	
	public abstract String getTestName(int argNum);
}
