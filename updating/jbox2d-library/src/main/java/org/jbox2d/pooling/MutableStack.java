package org.jbox2d.pooling;

public abstract class MutableStack<E> {

	private E[] stack;
	private int index;
	private int size;
	
	public MutableStack(){
		index = 0;
		size = 0;
	}
	
	protected void initStack(int argSize){
		index = argSize - 1; size = argSize;
		stack = createArray(argSize, null);
	}
	
	protected abstract E[] createArray(int argSize, E[] argOld);
	
	public final E pop(){
		if(index >= size){
			stack = createArray(size*2, stack);
			size = stack.length;
		}
		return stack[index++];
	}
	
	public final void push(E argObject){
		assert(index > 0);
		stack[--index] = argObject;
	}
}
