package org.jbox2d.pooling.normal;

import java.lang.reflect.Array;

import org.jbox2d.pooling.IOrderedStack;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class CircleStack<E> implements IOrderedStack<E>{
  private static final Logger log = LoggerFactory.getLogger(CircleStack.class);

  private final E[] pool;
  private int index;
  private final int size;
  private final E[] container;

  @SuppressWarnings("unchecked")
  public CircleStack(Class<E> argClass, int argStackSize, int argContainerSize) {
    size = argStackSize;
    pool = (E[]) Array.newInstance(argClass, argStackSize);
    for (int i = 0; i < argStackSize; i++) {
      try {
        pool[i] = argClass.newInstance();
      } catch (InstantiationException e) {
        log.error("Error creating pooled object " + argClass.getSimpleName(), e);
        assert (false) : "Error creating pooled object " + argClass.getCanonicalName();
      } catch (IllegalAccessException e) {
        log.error("Error creating pooled object " + argClass.getSimpleName(), e);
        assert (false) : "Error creating pooled object " + argClass.getCanonicalName();
      }
    }
    index = 0;
    container = (E[]) Array.newInstance(argClass, argContainerSize);
  }

  public final E pop() {
    index++;
    if(index >= size){
      index = 0;
    }
    return pool[index];
  }

  public final E[] pop(int argNum) {
    assert (argNum <= container.length) : "Container array is too small";
    if(index + argNum < size){
      System.arraycopy(pool, index, container, 0, argNum);
      index += argNum;
    }else{
      int overlap = (index + argNum) - size;
      System.arraycopy(pool, index, container, 0, argNum - overlap);
      System.arraycopy(pool, 0, container, argNum - overlap, overlap);
      index = overlap;
    }
    return container;
  }

  @Override
  public void push(int argNum) {}
}
