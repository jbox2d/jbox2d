package org.jbox2d.tests.math;

import org.jbox2d.common.MathUtils;

public class SinCosTable {
	
	public static final float TWOPI = (float) (Math.PI * 2);
	
	public static boolean LERP_LOOKUP = true;
	
	public final float precision;
	public final int tableLength;
	
	public final float[] sinLUT;
	//public final float[] cosLUT;

	public SinCosTable(float argPrecision){
		precision = argPrecision;
		tableLength = (int) Math.ceil(TWOPI / precision);
		
		sinLUT = new float[tableLength];
		//cosLUT = new float[tableLength];
		
		for(int i=0; i<tableLength; i++){
			sinLUT[i] = (float) Math.sin( i * precision);
			//cosLUT[i] = (float) Math.cos( i * precision);
		}
	}
	
	public final float sin(float x){
		x %= TWOPI;
		
		if(LERP_LOOKUP){
			
			x /= precision;
			
			final int index = (int)x;
			
			if(index != 0){
				x %= index;
			}
			
			// the next index is 0
			if(index == tableLength-1){
				return ( (1-x)*sinLUT[index] + x * sinLUT[0]);
			}else{
				return ( (1-x)*sinLUT[index] + x * sinLUT[index + 1]);
			}
			
		}else{
			return sinLUT[ MathUtils.round(x / precision) % tableLength];
		}
	}
}
