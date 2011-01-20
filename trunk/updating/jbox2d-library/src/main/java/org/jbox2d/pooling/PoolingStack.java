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
		assert(index < size);
		return pool[index++];
	}
	
	public final PoolContainer<E> pop(int argNum){
		assert(index + argNum < size);
		
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
		assert (index >= 0);
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
