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
