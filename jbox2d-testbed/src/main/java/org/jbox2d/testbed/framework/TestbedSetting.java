package org.jbox2d.testbed.framework;

/**
 * Defines a setting used in the testbed.
 * @author Daniel Murphy
 *
 */
public class TestbedSetting {
  
  /**
   * Whether the setting effects the engine's behavior or
   * modifies drawing.
   *
   */
  public static enum SettingsType {
    DRAWING, ENGINE
  }
  
  /**
   * The type of value this setting pertains to
   */
  public static enum ConstraintType {
    BOOLEAN, RANGE
  }
  
  public final String name;
  public final SettingsType settingsType;
  public final ConstraintType constraintType;
  public boolean enabled;
  public int value;
  public final int min;
  public final int max;
  
  public TestbedSetting(String argName, SettingsType argType, boolean argValue){
    name = argName;
    settingsType = argType;
    enabled = argValue;
    constraintType = ConstraintType.BOOLEAN;
    min = max = value = 0;
  }
  
  public TestbedSetting(String argName, SettingsType argType, int argValue, int argMinimum, int argMaximum){
    name = argName;
    settingsType = argType;
    value = argValue;
    min = argMinimum;
    max = argMaximum;
    constraintType = ConstraintType.RANGE;
    enabled = false;
  }
}
