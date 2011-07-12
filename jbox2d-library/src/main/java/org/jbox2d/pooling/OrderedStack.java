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

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * @author Daniel Murphy
 */
public class OrderedStack<E> implements IOrderedStack<E> {
	private static final Logger log = LoggerFactory.getLogger(OrderedStack.class);
	
	private final E[] pool;
	private int index;
	private final int size;
	private final E[] container;
	
	@SuppressWarnings("unchecked")
	public OrderedStack(Class<E> argClass, int argStackSize, int argContainerSize){
		size = argStackSize;
		pool = (E[]) Array.newInstance(argClass, argStackSize);
		for(int i=0; i<argStackSize; i++){
			try {
				pool[i] = argClass.newInstance();
			}
			catch (InstantiationException e) {
				log.error("Error creating pooled object " + argClass.getSimpleName(), e);
				assert(false) : "Error creating pooled object " + argClass.getCanonicalName();
			}
			catch (IllegalAccessException e) {
				log.error("Error creating pooled object " + argClass.getSimpleName(), e);
				assert(false) : "Error creating pooled object " + argClass.getCanonicalName();
			}
		}
		index = 0;
		container = (E[]) Array.newInstance(argClass, argContainerSize);
	}
	
	/* (non-Javadoc)
	 * @see org.jbox2d.pooling.IPoolingStack#pop()
	 */
	public final E pop(){
		assert(index < size) : "End of stack reached, there is probably a leak somewhere";
		return pool[index++];
	}
	
	/* (non-Javadoc)
	 * @see org.jbox2d.pooling.IPoolingStack#pop(int)
	 */
	public final E[] pop(int argNum){
		assert(index + argNum < size) : "End of stack reached, there is probably a leak somewhere";
		assert(argNum <= container.length) : "Container array is too small";
		System.arraycopy(pool, index, container, 0, argNum);
		index += argNum;
		return container;
	}
	
	/* (non-Javadoc)
	 * @see org.jbox2d.pooling.IPoolingStack#push(int)
	 */
	public final void push(int argNum){
		index -= argNum;
		assert (index >= 0) : "Beginning of stack reached, push/pops are unmatched";
	}
}
