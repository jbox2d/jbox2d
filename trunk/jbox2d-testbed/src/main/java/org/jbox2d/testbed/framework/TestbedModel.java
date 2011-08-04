package org.jbox2d.testbed.framework;

import java.util.Vector;

import javax.swing.DefaultComboBoxModel;

import org.jbox2d.callbacks.DebugDraw;
import org.jbox2d.common.Vec2;

/**
 * Testbed model
 * 
 * @author Daniel
 */
public class TestbedModel {

  private final DefaultComboBoxModel tests;
  private final TestbedSettings settings;
  private DebugDraw draw;
  private TestbedTest test;
  private final Vec2 mouse;
  private final Vector<TestChangedListener> listeners;
  public final boolean[] keys = new boolean[512];
  public final boolean[] codedKeys = new boolean[512];

  public TestbedModel() {
    settings = new TestbedSettings();
    tests = new DefaultComboBoxModel();
    listeners = new Vector<TestbedModel.TestChangedListener>();
    mouse = new Vec2();
  }

  public void setDebugDraw(DebugDraw argDraw) {
    draw = argDraw;
  }

  public DebugDraw getDebugDraw() {
    return draw;
  }

  public void setCurrTest(TestbedTest argTest) {
    test = argTest;
    for (TestChangedListener listener : listeners) {
      listener.testChanged(test);
    }
  }

  public TestbedTest getCurrTest() {
    return test;
  }

  public Vec2 getMouse() {
    return mouse;
  }

  public void setMouse(Vec2 argMouse) {
    mouse.set(argMouse);
  }

  /**
   * Gets the array of keys, index corresponding to the char value.
   * 
   * @return
   */
  public boolean[] getKeys(){
    return keys;
  }

  /**
   * Gets the array of coded keys, index corresponding to the coded key value.
   * 
   * @return
   */
  public boolean[] getCodedKeys(){
    return codedKeys;
  }
  
  public void addTestChangeListener(TestChangedListener argListener) {
    listeners.add(argListener);
  }

  public void removeTestChangeListener(TestChangedListener argListener) {
    listeners.remove(argListener);
  }

  public void addTest(TestbedTest argTest) {
    tests.addElement(new ListItem(argTest));
  }

  public void addCategory(String argName) {
    tests.addElement(new ListItem(argName));
  }

  public TestbedTest getTestAt(int argIndex) {
    ListItem item = (ListItem) tests.getElementAt(argIndex);
    if (item.isCategory()) {
      return null;
    }
    return item.test;
  }

  public boolean isTestAt(int argIndex) {
    ListItem item = (ListItem) tests.getElementAt(argIndex);
    return !item.isCategory();
  }

  public void clearTestList() {
    tests.removeAllElements();
  }

  public int getTestsSize() {
    return tests.getSize();
  }

  public DefaultComboBoxModel getComboModel() {
    return tests;
  }

  public TestbedSettings getSettings() {
    return settings;
  }

  public class ListItem {
    public String category;
    public TestbedTest test;

    public ListItem(String argCategory) {
      category = argCategory;
    }

    public ListItem(TestbedTest argTest) {
      test = argTest;
    }

    public boolean isCategory() {
      return category != null;
    }

    @Override
    public String toString() {
      return isCategory() ? category : test.getTestName();
    }
  }

  public static interface TestChangedListener {
    public void testChanged(TestbedTest argTest);
  }
}
