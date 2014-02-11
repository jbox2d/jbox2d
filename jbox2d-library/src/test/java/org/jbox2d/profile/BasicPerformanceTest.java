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
/**
 * Created at 8:12:11 AM Jan 18, 2011
 */
package org.jbox2d.profile;

import java.util.ArrayList;
import java.util.Collections;

import org.apache.commons.math3.stat.descriptive.DescriptiveStatistics;
import org.jbox2d.common.MathUtils;

/**
 * @author Daniel Murphy
 */
public abstract class BasicPerformanceTest {

  private final int numTests, iters, frames;
  protected final DescriptiveStatistics[] stats;
  private ArrayList<Integer> testOrder = new ArrayList<Integer>();

  public BasicPerformanceTest(int numTests, int iters, int frames) {
    this.numTests = numTests;
    this.iters = iters;
    this.frames = frames;
    stats = new DescriptiveStatistics[numTests];
    for (int i = 0; i < numTests; i++) {
      stats[i] = new DescriptiveStatistics(iters*frames + 1);
      testOrder.add(i);
    }
  }

  public void go() {
    long prev, after;
    // warmup
    println("Warmup");
    for (int i = 0; i < MathUtils.max(1, iters / 10); i++) {
      Collections.shuffle(testOrder);
      for (int test = 0; test < numTests; test++) {
        int runningTest = testOrder.get(test);
        setupTest(runningTest);
        for (int j = 0; j < frames; j++) {
          step(runningTest);
        }
      }
    }
    for (int i = 0; i < iters; i++) {
      println(i * 100.0 / iters + "%");
      Collections.shuffle(testOrder);
      for (int test = 0; test < numTests; test++) {
        int runningTest = testOrder.get(test);
        setupTest(runningTest);
        for (int j = 0; j < frames; j++) {
          prev = System.nanoTime();
          step(runningTest);
          after = System.nanoTime();
          stats[runningTest].addValue((after - prev) * 1d / 1000000);
        }
      }
    }
    printResults();
  }

  public void printResults() {
    printf("%-20s%20s%20s%20s\n", "Test Name", "Milliseconds Avg", "StdDev", "95% Interval");

    for (int i = 0; i < numTests; i++) {
      double mean = stats[i].getMean();
      double stddev = stats[i].getStandardDeviation();
      double diff = 1.96 * stddev / MathUtils.sqrt(stats[i].getN());
      printf("%-20s%20.3f%20.3f  (%7.3f,%7.3f)\n", getTestName(i), mean, stddev, mean - diff, mean
          + diff);
    }
  }

  public void setupTest(int testNum) {}

  public abstract void step(int testNum);

  public abstract String getTestName(int testNum);

  public int getFrames(int testNum) {
    return 0;
  }

  // override to change output
  public void println(String s) {
    System.out.println(s);
  }

  public void printf(String s, Object... args) {
    System.out.printf(s, args);
  }
}
