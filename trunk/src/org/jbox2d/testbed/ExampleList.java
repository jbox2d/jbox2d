package org.jbox2d.testbed;

import java.util.ArrayList;
import java.util.List;

import org.jbox2d.testbed.tests.BipedTest;
import org.jbox2d.testbed.tests.Bridge;
import org.jbox2d.testbed.tests.BuoyancyTest;
import org.jbox2d.testbed.tests.Chain;
import org.jbox2d.testbed.tests.CharacterControlTest;
import org.jbox2d.testbed.tests.CompoundShapes;
import org.jbox2d.testbed.tests.Domino;
import org.jbox2d.testbed.tests.DominoTower;
import org.jbox2d.testbed.tests.EdgeTest;
import org.jbox2d.testbed.tests.Gears;
import org.jbox2d.testbed.tests.LiquidTest;
import org.jbox2d.testbed.tests.MotorsAndLimits;
import org.jbox2d.testbed.tests.Pulleys;
import org.jbox2d.testbed.tests.Pyramid;
import org.jbox2d.testbed.tests.RaycastTest;
import org.jbox2d.testbed.tests.SpriteBinding;
import org.jbox2d.testbed.tests.VaryingFriction;
import org.jbox2d.testbed.tests.VerticalStack;

public class ExampleList {
	public static List<AbstractExample> getExamples(TestbedMain testbed) {
		// Simple functionality examples
    	//
    	ArrayList<AbstractExample> exampleList = new ArrayList<AbstractExample>();
    	//*
    	exampleList.add(new CharacterControlTest(testbed));
    	exampleList.add(new BuoyancyTest(testbed));
    	exampleList.add(new RaycastTest(testbed));
//    	exampleList.add(new ScratchPad(testbed));
//    	exampleList.add(new SensorTest(testbed));
    	exampleList.add(new EdgeTest(testbed));
//    	exampleList.add(new CCDTest(testbed));
//    	exampleList.add(new Motox(testbed));
//    	exampleList.add(new BlobTest3(testbed));
//    	exampleList.add(new BlobTest4(testbed));
//    	exampleList.add(new BlobTest5(testbed));
//   	exampleList.add(new BlobTest6(testbed));
//    	exampleList.add(new BlobTest2(testbed));
//    	exampleList.add(new BlobTest7(testbed));
    	exampleList.add(new LiquidTest(testbed));
    	exampleList.add(new BipedTest(testbed));
    	exampleList.add(new SpriteBinding(testbed));
    	exampleList.add(new Pulleys(testbed));
//    	exampleList.add(new Overhang(testbed));
//    	exampleList.add(new VaryingRestitution(testbed));
    	exampleList.add(new VaryingFriction(testbed));
    	exampleList.add(new MotorsAndLimits(testbed));
    	exampleList.add(new VerticalStack(testbed));
    	exampleList.add(new Domino(testbed));
    	exampleList.add(new CompoundShapes(testbed));
    	exampleList.add(new Chain(testbed));
    	exampleList.add(new Bridge(testbed));
    	exampleList.add(new Gears(testbed));
//    	exampleList.add(new RestitutionCannon(testbed));
//    	exampleList.add(new BlobTest(testbed));
    	
    	// Shape drawing demo
//    	exampleList.add(new ShapeDrawing(testbed));
    	
    	// Stress tests
    	exampleList.add(new Pyramid(testbed));
    	exampleList.add(new DominoTower(testbed));
//    	exampleList.add(new Circles(testbed));
    	
    	// Bug tests
//    	exampleList.add(new DistanceTest(testbed));
    	//exampleList.add(new BugTest(testbed));
//*/
    	return exampleList;
	}
}
