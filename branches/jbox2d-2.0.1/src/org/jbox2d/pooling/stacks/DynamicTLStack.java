package org.jbox2d.pooling.stacks;

import java.util.Stack;

import org.jbox2d.pooling.TLStack;

public abstract class DynamicTLStack<T> {

	private final TLStack<T> tlStack = new TLStack<T>();
	
	public T get(){
		Stack<T> stack = tlStack.get();
		
		if(stack.isEmpty()){
			stack.push(newObjectInstance());
			stack.push(newObjectInstance());
			stack.push(newObjectInstance());
		}
		
		return stack.pop();
	}
	
	public void recycle(T argObject){
		tlStack.get().push(argObject);
	}
	
	protected abstract T newObjectInstance();
}
