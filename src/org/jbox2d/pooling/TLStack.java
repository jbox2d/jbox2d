package org.jbox2d.pooling;

import java.util.Stack;

public class TLStack<T> extends ThreadLocal<Stack<T>> {
	protected Stack<T> initialValue(){
		return new Stack<T>();
	}
}
