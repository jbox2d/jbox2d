/**
 * Created at 8:05:23 AM Jan 18, 2011
 */
package org.jbox2d.testbed.perf;

import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import org.jbox2d.pooling.TLVec2;
import org.jbox2d.pooling.WorldPool;

// Mac results 1/19/11
//	Test Name    Milliseconds Avg
//	 Creation             65.4416
// World Pool            202.3449
//Circle Pool             62.5145
//ThreadLocal member      62.1190
//     Member             60.6938

// Windows results 1/19/11
//Test Name               Milliseconds Avg
//Creation                         74.6291
//World Pool                      273.5585
//Circle Pool                      82.2195
//ThreadLocal member               83.2970
//Member                           80.7545


/**
 * @author Daniel Murphy
 */
public class PoolingPerf extends PerfTest{
	


	public static final int INNER_ITERS = 1000000;
	public static final int OUTER_ITERS = 100;
	
	public static class CirclePool{
		final Vec2[] pool;
		int index;
		public CirclePool(){
			pool = new Vec2[50];
			for(int i=0; i<pool.length; i++){
				pool[i] = new Vec2();
			}
			index = -1;
		}
		
		public final Vec2 get(){
			index = (index + 1)%pool.length;
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
	
	public String[] tests = new String[]{
		"Creation", "World Pool", "Circle Pool", "Custom Stack", "ThreadLocal member", "Member"
	};
	
	public float aStore = 0;
	public WorldPool wp = new WorldPool();
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
	 * @see org.jbox2d.testbed.perf.PerfTest#runTest(int)
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
			wp.pushVec2(v);
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
	 * @see org.jbox2d.testbed.perf.PerfTest#getTestName(int)
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
