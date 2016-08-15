package org.jbox2d.testbed.framework.javafx;

import java.util.HashMap;
import java.util.Map;

public class DoubleArray {

  private final Map<Integer, double[]> map = new HashMap<Integer, double[]>();

  public double[] get(int argLength) {
    assert (argLength > 0);

    if (!map.containsKey(argLength)) {
      map.put(argLength, getInitializedArray(argLength));
    }

    assert (map.get(argLength).length == argLength) : "Array not built of correct length";
    return map.get(argLength);
  }

  protected double[] getInitializedArray(int argLength) {
    return new double[argLength];
  }
}
