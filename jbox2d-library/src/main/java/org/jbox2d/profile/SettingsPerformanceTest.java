package org.jbox2d.profile;

import org.jbox2d.common.Settings;

public abstract class SettingsPerformanceTest extends BasicPerformanceTest {

  private static int NUM_TESTS = 14;

  public SettingsPerformanceTest(int iters) {
    super(NUM_TESTS, iters);
  }

  @Override
  public void runTest(int testNum) {
    Settings.FAST_ABS = testNum == 1;
    Settings.FAST_ATAN2 = testNum == 2;
    Settings.FAST_CEIL = testNum == 3;
    Settings.FAST_FLOOR = testNum == 4;
    Settings.FAST_ROUND = testNum == 5;
    Settings.SINCOS_LUT_ENABLED = testNum == 6;

    if (testNum == 7) {
      Settings.FAST_ABS = true;
      Settings.FAST_ATAN2 = true;
      Settings.FAST_CEIL = true;
      Settings.FAST_FLOOR = true;
      Settings.FAST_ROUND = true;
      Settings.SINCOS_LUT_ENABLED = true;
    }

    if (testNum > 7) {
      Settings.FAST_ABS = testNum != 8;
      Settings.FAST_ATAN2 = testNum != 9;
      Settings.FAST_CEIL = testNum != 10;
      Settings.FAST_FLOOR = testNum != 11;
      Settings.FAST_ROUND = testNum != 12;
      Settings.SINCOS_LUT_ENABLED = testNum != 13;
    }

    runBenchmarkWorld();
  }

  public abstract void runBenchmarkWorld();

  @Override
  public String getTestName(int testNum) {
    switch (testNum) {
      case 0:
        return "No optimizations";
      case 1:
        return "Fast abs";
      case 2:
        return "Fast atan2";
      case 3:
        return "Fast ceil";
      case 4:
        return "Fast floor";
      case 5:
        return "Fast round";
      case 6:
        return "Sincos lookup table";
      case 7:
        return "All optimizations on";
      case 8:
        return "no Fast abs";
      case 9:
        return "no Fast atan2";
      case 10:
        return "no Fast ceil";
      case 11:
        return "no Fast floor";
      case 12:
        return "no Fast round";
      case 13:
        return "no Sincos lookup table";
      default:
        return "";
    }
  }
}
