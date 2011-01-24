package org.jbox2d.pooling;

public abstract class MutableStack<E> {

	private E[] stack;
	private int index;
	private int size;
	
	public MutableStack(){
		index = 4; size = 5;
		stack = createArray(5);
	}
	
	protected abstract E[] createArray(int argSize);
	
	public final E pop(){
		if(index >= size){
			stack = createArray(size*2);
			size = stack.length;
		}
		return stack[index++];
	}
	
	public final void push(E argObject){
		assert(index > 0);
		stack[--index] = argObject;
	}
}
