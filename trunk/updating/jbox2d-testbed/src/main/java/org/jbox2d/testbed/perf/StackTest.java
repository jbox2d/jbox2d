/**
 * Created at 6:24:51 AM Jan 20, 2011
 */
package org.jbox2d.testbed.perf;

import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import org.jbox2d.pooling.PoolingStack.PoolContainer;
import org.jbox2d.pooling.WorldPool;


//Test Name               Milliseconds Avg
//Pop2Sep                          12.8310
//Pop2Cont                         12.9120
//Pop3Sep                          19.7939
//Pop3Cont                         20.0433
//Pop4Sep                          26.1034
//Pop4Cont                         26.8309
//Pop9Sep                          59.5768
//Pop9Cont                         58.4641

/**
 * @author Daniel Murphy
 */
public class StackTest extends PerfTest {
	
	public static final int INNER_ITERS = 100000;
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
	
	private final WorldPool wp = new WorldPool(100);

	
	/**
	 * @see org.jbox2d.testbed.perf.PerfTest#runTest(int)
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
					final PoolContainer<Vec2> pc = wp.popVec2(2);
					final Vec2 v1 = pc.p0;
					final Vec2 v2 = pc.p1;
					a += op(v1);
					a += op(v2);
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
					final PoolContainer<Vec2> pc = wp.popVec2(3);
					final Vec2 v1 = pc.p0;
					final Vec2 v2 = pc.p1;
					final Vec2 v3 = pc.p2;
					a += op(v1);
					a += op(v2);
					a += op(v3);
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
					final PoolContainer<Vec2> pc = wp.popVec2(4);
					final Vec2 v1 = pc.p0;
					final Vec2 v2 = pc.p1;
					final Vec2 v3 = pc.p2;
					final Vec2 v4 = pc.p3;
					a += op(v1);
					a += op(v2);
					a += op(v3);
					a += op(v4);
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
					final PoolContainer<Vec2> pc = wp.popVec2(9);
					final Vec2 v1 = pc.p0;
					final Vec2 v2 = pc.p1;
					final Vec2 v3 = pc.p2;
					final Vec2 v4 = pc.p3;
					final Vec2 v5 = pc.p4;
					final Vec2 v6 = pc.p5;
					final Vec2 v7 = pc.p6;
					final Vec2 v8 = pc.p7;
					final Vec2 v9 = pc.p8;
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
			}
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
		(new StackTest()).go();
	}
	
}
