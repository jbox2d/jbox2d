package org.jbox2d.pooling.stacks;

import java.util.Stack;


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
	
	public void recycle(T... argObjects){
		Stack<T> stack = tlStack.get();
		for(T object : argObjects){
			stack.push(object);
		}
	}
	
	protected abstract T newObjectInstance();
}
