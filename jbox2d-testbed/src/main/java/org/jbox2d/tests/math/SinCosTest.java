/*******************************************************************************
 * Copyright (c) 2013, Daniel Murphy
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 	* Redistributions of source code must retain the above copyright notice,
 * 	  this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright notice,
 * 	  this list of conditions and the following disclaimer in the documentation
 * 	  and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
package org.jbox2d.tests.math;

import org.jbox2d.common.MathUtils;

public class SinCosTest {
  // formating stuff
  public static final int COLUMN_PADDING = 3;
  public static final int NUM_DECIMALS = 8;

  public static int numTables = 50;
  // accuracy
  public static float mostPreciseTable = .00001f;
  public static float leastPreciseTable = .01f;
  public static int accuracyIterations = 100000;

  // speed
  public static int speedTrials = 20;
  public static int speedIterations = 50000;

  private static SinCosTable[] tables;

  /**
   * @param args
   */
  public static void main(String[] args) {
    int overall = 1;
    try {
      numTables = Integer.parseInt(args[0]);
      mostPreciseTable = Float.parseFloat(args[1]);
      leastPreciseTable = Float.parseFloat(args[2]);
      accuracyIterations = Integer.parseInt(args[3]);
      speedTrials = Integer.parseInt(args[4]);
      speedIterations = Integer.parseInt(args[5]);
      overall = Integer.parseInt(args[6]);
    } catch (Exception e) {
      System.out
          .println("Parameters: <number of tables to use> <most precise table value (smallest)> "
              + "<least precise table value> <number of accuracy test iterations> <number of speed test trials>"
              + "<number of speed test iterations> <number of overall speed test sets>");
      System.out.println("Sample parameters: 200 .00001 .01 100000 20 5000 2");
      // return;
    }
    System.out.println("Tables: " + numTables);
    System.out.println("Most Precise Table: " + mostPreciseTable);
    System.out.println("Least Precise Table: " + leastPreciseTable);
    System.out.println("Accuracy Iterations: " + accuracyIterations);
    System.out.println("Speed Trials: " + speedTrials);
    System.out.println("Speed Iterations: " + speedIterations);

    constructTables();
    doAccuracyTest(true);
    for (int i = 0; i < overall; i++) {
      doSpeedTest(true);
      try {
        Thread.sleep(1000);
      } catch (InterruptedException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    }
  }

  /**
   * constructs the tables from the static parameters
   */
  public static final void constructTables() {
    tables = new SinCosTable[numTables];

    System.out.println("constructing tables");
    for (int i = 0; i < numTables; i++) {
      // well... basic lerp
      float precision = i * 1f / numTables * (leastPreciseTable - mostPreciseTable)
          + mostPreciseTable;
      tables[i] = new SinCosTable(precision);
    }
  }

  /**
   * accuracy test from the static parameters, the tables array needs to be constructed as well,
   * returns double[tables][0-3 (no lerp, lerp, then the difference)]
   * 
   * @return
   */
  public static final double[][] doAccuracyTest(boolean print) {

    System.out.println("doing accuracy tests");

    double[][] accuracyResults = new double[numTables][3];

    SinCosTable.LERP_LOOKUP = false;
    // without lerp
    for (int i = 0; i < numTables; i++) {
      accuracyResults[i][0] = accuracyTest(tables[i], accuracyIterations);
    }

    SinCosTable.LERP_LOOKUP = true;
    // with lerp
    for (int i = 0; i < numTables; i++) {
      accuracyResults[i][1] = accuracyTest(tables[i], accuracyIterations);
    }

    for (int i = 0; i < numTables; i++) {
      accuracyResults[i][2] = accuracyResults[i][0] - accuracyResults[i][1];
    }

    if (print) {
      System.out.println("Accuracy results, average displacement");
      String header[] = { "Not lerped", "Lerped", "Difference" };
      String side[] = new String[numTables + 1];
      side[0] = "Table precision";
      for (int i = 0; i < tables.length; i++) {
        side[i + 1] = formatDecimal(tables[i].precision, NUM_DECIMALS);
      }
      printTable(header, side, accuracyResults);
    }
    return accuracyResults;
  }

  /**
   * speed test from the static parameters the tables array needs to be constructed as well, returns
   * double[tables][0-3 (no lerp, lerp, then the difference)]
   * 
   * @return
   */
  public static final double[][] doSpeedTest(boolean print) {
    System.out.println("\nDoing speed tests");
    double[][] speedResults = new double[numTables][4];

    SinCosTable.LERP_LOOKUP = false;
    // without lerp
    for (int i = 0; i < numTables; i++) {
      speedResults[i][0] = speedTest(tables[i], speedIterations, speedTrials);
    }

    SinCosTable.LERP_LOOKUP = true;
    // with lerp
    for (int i = 0; i < numTables; i++) {
      speedResults[i][1] = speedTest(tables[i], speedIterations, speedTrials);
    }

    // with the Math calls
    for (int i = 0; i < numTables; i++) {
      speedResults[i][3] = speedTestMath(speedIterations, speedTrials);
    }

    for (int i = 0; i < numTables; i++) {
      speedResults[i][2] = speedResults[i][0] - speedResults[i][1];
    }

    if (print) {
      System.out.println("Speed results, in iterations per second (higher number means faster)");
      String header[] = { "Not lerped", "Lerped", "Difference", "Java Math" };
      String side[] = new String[numTables + 1];
      side[0] = "Table precision";
      for (int i = 0; i < tables.length; i++) {
        side[i + 1] = formatDecimal(tables[i].precision, NUM_DECIMALS);
      }
      printTable(header, side, speedResults);
    }
    return speedResults;
  }

  private static double accuracyTest(SinCosTable table, int iterations) {
    double totalDiff = 0f, diff = 0f;

    for (int i = 0; i < iterations; i++) {
      float querry = (float) Math.random() * MathUtils.TWOPI;
      diff = MathUtils.abs((float) Math.sin(querry) - table.sin(querry));
      totalDiff += diff;
    }
    totalDiff /= iterations;
    return totalDiff;
  }

  private static void printTable(String header[], String side[], double[][] results) {

    // first determine the amount of space we need for each column
    int[] colLengths = new int[results[0].length + 1];
    for (int i = 0; i < colLengths.length; i++) {
      colLengths[i] = 0;
    }
    for (int j = -1; j < results[0].length; j++) {
      if (j == -1) {
        int colLength = side[j + 1].length() + COLUMN_PADDING;
        if (colLength > colLengths[j + 1]) {
          colLengths[j + 1] = colLength;
        }
      } else {
        int colLength = header[j].length() + COLUMN_PADDING;
        if (colLength > colLengths[j + 1]) {
          colLengths[j + 1] = colLength;
        }
        for (int i = 0; i < results.length; i++) {
          colLength = (formatDecimal(results[i][j], NUM_DECIMALS)).length() + COLUMN_PADDING;
          if (colLength > colLengths[j + 1]) {
            colLengths[j + 1] = colLength;
          }
        }
      }

    }

    // header

    System.out.print(spaceString(side[0], colLengths[0]));
    for (int i = 1; i < colLengths.length; i++) {
      System.out.print(spaceString(header[i - 1], colLengths[i]));
    }
    System.out.println();

    for (int i = 0; i < results.length; i++) {

      for (int j = -1; j < results[i].length; j++) {
        if (j == -1) {
          System.out.print(spaceString(side[i + 1], colLengths[j + 1]));
        } else {
          String toPrint = formatDecimal(results[i][j], NUM_DECIMALS);
          System.out.print(spaceString(toPrint, colLengths[j + 1]));
        }
      }
      System.out.println();
    }
  }

  private static long speedTest(SinCosTable table, final int numIterations, final int numTrials) {
    long startTime, endTime;
    long totalTime = 0;
    float i, j;
    float k = 0;

    final float jstep = MathUtils.TWOPI / numIterations;

    for (i = 0; i < numTrials; i++) {

      startTime = System.nanoTime();
      for (j = 0; j < MathUtils.TWOPI; j += jstep) {
        k += table.sin(j);
      }
      endTime = System.nanoTime();
      totalTime += endTime - startTime;
    }
    i += k;

    return numIterations * numTrials * 1000000000l / (totalTime);
  }

  private static long speedTestMath(final int numIterations, final int numTrials) {
    long startTime, endTime;
    long totalTime = 0;
    float i, j;
    float k = 0;

    final float jstep = MathUtils.TWOPI / numIterations;

    for (i = 0; i < numTrials; i++) {

      startTime = System.nanoTime();
      for (j = 0; j < MathUtils.TWOPI; j += jstep) {
        k += (float) StrictMath.sin(j);
      }
      endTime = System.nanoTime();
      totalTime += endTime - startTime;
    }
    i += k;

    return numIterations * numTrials * 1000000000l / (totalTime);
  }

  private static String spaceString(String str, int space) {
    // if the string is more than the space
    if (str.length() == space) {
      return str;
    } else if (str.length() >= space) {
      return str.substring(0, space);
    }
    String s = new String(str);

    for (int i = s.length(); i < space; i++) {
      s = " " + s;
    }
    return s;
  }

  private static String formatDecimal(double n, int decimals) {
    String num = n + "";
    // no decimal
    if (num.indexOf(".") == -1) {
      return num;
    }

    boolean ePresent = false;
    String e = null;

    if (num.indexOf("E") != -1) {
      e = num.substring(num.indexOf("E"));
      decimals -= e.length();
      num = num.substring(0, num.indexOf("E"));
      ePresent = true;
    }

    int decLen = num.substring(num.indexOf(".") + 1).length();
    int numLen = num.substring(0, num.indexOf(".")).length();

    // if not enough decimals
    if (decLen < decimals) {
      for (int i = 0; i < (decimals - decLen); i++) {
        num = num + " ";
      }
    } else if (decLen > decimals) { // more decimals than needed
      num = num.substring(0, numLen + decimals + 1);
    }
    if (ePresent) {
      num += e;
    }
    return num;
  }
}
