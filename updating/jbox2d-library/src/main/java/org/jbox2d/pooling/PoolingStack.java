/*******************************************************************************
 * Copyright (c) 2011, Daniel Murphy
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL DANIEL MURPHY BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
/**
 * Created at 12:52:04 AM Jan 20, 2011
 */
package org.jbox2d.pooling;

import java.lang.reflect.Array;

/**
 * @author Daniel Murphy
 */
public class PoolingStack<E> {
	
	private final E[] pool;
	private int index;
	private final int size;
	private final PoolContainer<E> container;
	
	@SuppressWarnings("unchecked")
	public PoolingStack(Class<E> argClass, int argStackSize){
		size = argStackSize;
		pool = (E[]) Array.newInstance(argClass, argStackSize);
		for(int i=0; i<argStackSize; i++){
			try {
				pool[i] = argClass.newInstance();
			}
			catch (InstantiationException e) {
				System.err.println("Error creating pooled object "+argClass.getCanonicalName());
				e.printStackTrace();
			}
			catch (IllegalAccessException e) {
				System.err.println("Error creating pooled object "+argClass.getCanonicalName());
				e.printStackTrace();
			}
		}
		index = 0;
		container = new PoolContainer<E>();
	}
	
	public final E pop(){
		assert(index < size) : "End of stack reached, there is probably a leak somewhere";
		return pool[index++];
	}
	
	public final PoolContainer<E> pop(int argNum){
		assert(index + argNum < size) : "End of stack reached, there is probably a leak somewhere";
		
		switch(argNum){
			case 9:
				container.p8 = pool[index++];
			case 8:
				container.p7 = pool[index++];
			case 7:
				container.p6 = pool[index++];
			case 6:
				container.p5 = pool[index++];
			case 5:
				container.p4 = pool[index++];
			case 4:
				container.p3 = pool[index++];
			case 3:
				container.p2 = pool[index++];
			case 2:
				container.p1 = pool[index++];
			case 1:
				container.p0 = pool[index++];
				break;
			default:
				assert(false);
		}
		return container;
	}
	
	public final void push(int argNum){
		index -= argNum;
		assert (index >= 0) : "Beginning of stack reached, push/pops are unmatched";
	}
	
	
	public static class PoolContainer<E>{
		public static final int MAX_MEMBERS = 9;
		
		public E p0,p1,p2,p3,p4,p5,p6,p7,p8;
		
		public void populate(E[] argRay){
			p0 = argRay[0];
			p1 = argRay[1];
			p2 = argRay[2];
			p3 = argRay[3];
			p4 = argRay[4];
			p5 = argRay[5];
			p6 = argRay[6];
			p7 = argRay[7];
			p8 = argRay[8];
		}
	}
}
