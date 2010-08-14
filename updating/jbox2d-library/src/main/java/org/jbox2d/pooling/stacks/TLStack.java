package org.jbox2d.pooling.stacks;

import java.util.Stack;

import org.jbox2d.pooling.CustThreadLocal;

public class TLStack<T> extends CustThreadLocal<Stack<T>> {
	
	protected Stack<T> initialValue(){
		return new Stack<T>();
	}
}
