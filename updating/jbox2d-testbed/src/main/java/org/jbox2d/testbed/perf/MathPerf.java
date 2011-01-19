/**
 * Created at 7:09:57 PM Jan 18, 2011
 */
package org.jbox2d.testbed.perf;

import org.jbox2d.common.MathUtils;

//Sin took 171.891 ms on average
//SinLUT took 53.677 ms on average
//Pow took 149.635 ms on average
//FastPow took 42.247 ms on average
//Max took 23.392 ms on average
//FastMax took 23.335 ms on average
//Floor took 0.273 ms on average
//fastFloor took 0.098 ms on average
//aTan2 took 40.397 ms on average
//fastATan2 took 36.709 ms on average
//ceil took 0.198 ms on average
//fastCeil took 0.126 ms on average
//FastSin took 57.313 ms on average

/**
 * @author Daniel Murphy
 */
public class MathPerf extends PerfTest{

	public static int INNER_ITERS = 100000;
	public static int OUTER_ITERS = 20;
	
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
	 * @see org.jbox2d.testbed.perf.PerfTest#runTest(int)
	 */
	@Override
	public void runTest(int argNum) {
		float random = MathUtils.randomFloat(Float.MIN_VALUE/3, Float.MAX_VALUE/3);
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
	 * @see org.jbox2d.testbed.perf.PerfTest#getTestName(int)
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
