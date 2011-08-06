package org.jbox2d.testbed.framework;

import java.util.Vector;

import javax.swing.DefaultComboBoxModel;

import org.jbox2d.callbacks.DebugDraw;
import org.jbox2d.common.Vec2;

/**
 * Model for the testbed
 * 
 * @author Daniel
 */
public class TestbedModel {

  private final DefaultComboBoxModel tests = new DefaultComboBoxModel();
  private final TestbedSettings settings = new TestbedSettings();
  private DebugDraw draw;
  private TestbedTest test;
  private final Vec2 mouse = new Vec2();
  private final Vector<TestChangedListener> listeners = new Vector<TestChangedListener>();
  private final boolean[] keys = new boolean[512];
  private final boolean[] codedKeys = new boolean[512];
  private float calculatedFps;
  private float panelWidth;
  private int currTestIndex;
  
  public TestbedModel() {
  }

  public void setCalculatedFps(float calculatedFps) {
    this.calculatedFps = calculatedFps;
  }

  public float getCalculatedFps() {
    return calculatedFps;
  }

  public void setPanelWidth(float panelWidth) {
    this.panelWidth = panelWidth;
  }

  public float getPanelWidth() {
    return panelWidth;
  }

  public void setDebugDraw(DebugDraw argDraw) {
    draw = argDraw;
  }

  public DebugDraw getDebugDraw() {
    return draw;
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
  
  public void setCurrTestIndex(int argCurrTestIndex) {
    if(argCurrTestIndex < 0 || argCurrTestIndex >= tests.getSize()){
      throw new IllegalArgumentException("Invalid test index");
    }
    if(currTestIndex == argCurrTestIndex){
      return;
    }
    
    if(!isTestAt(argCurrTestIndex)){
      throw new IllegalArgumentException("No test at " + argCurrTestIndex);
    }
    currTestIndex = argCurrTestIndex;
    ListItem item = (ListItem) tests.getElementAt(argCurrTestIndex);
    test = item.test;
    for (TestChangedListener listener : listeners) {
      listener.testChanged(test, currTestIndex);
    }
  }

  public int getCurrTestIndex() {
    return currTestIndex;
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
    public void testChanged(TestbedTest argTest, int argIndex);
  }
}
