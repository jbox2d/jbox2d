package org.jbox2d.testbed.mathtests;

import org.jbox2d.common.MathUtils;

public class SqrtTest {
	
	public static void main(String[] args) {
		
		// warm up
		float result;
		for(int i=0; i<1000000000; i++){
			result = MathUtils.sqrt(i);
			result = (float) Math.sqrt(i);
		}
		
		int times = 1000000000;
		
		long startTime,afterTime;
		startTime = System.nanoTime();
		for(int i=0; i<times; i++){
			result = (float) Math.sqrt(i/Math.sqrt(i));
		}
		afterTime = System.nanoTime();
		double jmathseconds = (afterTime-startTime)*1.0/1000000000;
		System.out.println("jmath time: "+jmathseconds);
		
		startTime = System.nanoTime();
		for(int i=0; i<times; i++){
			result = MathUtils.sqrt(i/MathUtils.sqrt(i));
		}
		afterTime = System.nanoTime();
		double mathutilseconds = (afterTime-startTime)*1.0/1000000000;
		System.out.println("math util time: "+mathutilseconds);
		
		
		startTime = System.nanoTime();
		for(int i=0; i<times; i++){
			result = (float) Math.sqrt(i/(float)Math.sqrt(i));
		}
		afterTime = System.nanoTime();
		jmathseconds = (afterTime-startTime)*1.0/1000000000;
		System.out.println("jmath time: "+jmathseconds);
		
		startTime = System.nanoTime();
		for(int i=0; i<times; i++){
			result = MathUtils.sqrt(i/MathUtils.sqrt(i));
		}
		afterTime = System.nanoTime();
		mathutilseconds = (afterTime-startTime)*1.0/1000000000;
		System.out.println("math util time: "+mathutilseconds);
		
		startTime = System.nanoTime();
		for(int i=0; i<times; i++){
			result = (float) Math.sqrt(i/(float)Math.sqrt(i));
		}
		afterTime = System.nanoTime();
		jmathseconds = (afterTime-startTime)*1.0/1000000000;
		System.out.println("jmath time: "+jmathseconds);
		
		startTime = System.nanoTime();
		for(int i=0; i<times; i++){
			result = MathUtils.sqrt(i/MathUtils.sqrt(i));
		}
		afterTime = System.nanoTime();
		mathutilseconds = (afterTime-startTime)*1.0/1000000000;
		System.out.println("math util time: "+mathutilseconds);
		
		startTime = System.nanoTime();
		for(int i=0; i<times; i++){
			result = (float) Math.sqrt(i/(float)Math.sqrt(i));
		}
		afterTime = System.nanoTime();
		jmathseconds = (afterTime-startTime)*1.0/1000000000;
		System.out.println("jmath time: "+jmathseconds);
		
		startTime = System.nanoTime();
		for(int i=0; i<times; i++){
			result = MathUtils.sqrt(i/MathUtils.sqrt(i));
		}
		afterTime = System.nanoTime();
		mathutilseconds = (afterTime-startTime)*1.0/1000000000;
		System.out.println("math util time: "+mathutilseconds);
		
		startTime = System.nanoTime();
		for(int i=0; i<times; i++){
			result = (float) Math.sqrt(i/(float)Math.sqrt(i));
		}
		afterTime = System.nanoTime();
		jmathseconds = (afterTime-startTime)*1.0/1000000000;
		System.out.println("jmath time: "+jmathseconds);
		
		startTime = System.nanoTime();
		for(int i=0; i<times; i++){
			result = MathUtils.sqrt(i/MathUtils.sqrt(i));
		}
		afterTime = System.nanoTime();
		mathutilseconds = (afterTime-startTime)*1.0/1000000000;
		System.out.println("math util time: "+mathutilseconds);
	}
}
