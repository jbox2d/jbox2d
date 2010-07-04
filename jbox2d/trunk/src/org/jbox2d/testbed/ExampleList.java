package org.jbox2d.testbed;

import java.util.ArrayList;
import java.util.List;

import org.jbox2d.testbed.tests.*;

public class ExampleList {
	public static List<AbstractExample> getExamples(TestbedMain testbed) {
		// Simple functionality examples
    	//
    	ArrayList<AbstractExample> exampleList = new ArrayList<AbstractExample>();
    	//*
    	exampleList.add(new Overhang(testbed));
    	exampleList.add(new SpriteBinding(testbed));
    	exampleList.add(new ViewportTest(testbed));
    	exampleList.add(new VaryingRestitution(testbed));
    	exampleList.add(new RestitutionCannon(testbed));
    	exampleList.add(new VaryingFriction(testbed));
    	exampleList.add(new VerticalStack(testbed));
    	exampleList.add(new Domino(testbed));
    	exampleList.add(new CompoundShapes(testbed));
    	exampleList.add(new Chain(testbed));
    	exampleList.add(new Bridge(testbed));
    	exampleList.add(new MotorsAndLimits(testbed));
    	exampleList.add(new Gears(testbed));
    	exampleList.add(new BipedTest(testbed));
    	exampleList.add(new Pulleys(testbed));
    	exampleList.add(new SensorTest(testbed));
//    	exampleList.add(new ScratchPad(testbed));
    	exampleList.add(new EdgeTest(testbed));
    	exampleList.add(new CCDTest(testbed));
//    	exampleList.add(new Motox(testbed));
    	exampleList.add(new BlobTest3(testbed));
    	exampleList.add(new BlobTest4(testbed));
    	exampleList.add(new BlobTest5(testbed));
    	exampleList.add(new BlobTest6(testbed));
    	exampleList.add(new BlobTest2(testbed));
    	exampleList.add(new BlobTest7(testbed));
    	exampleList.add(new LiquidTest(testbed));
    	exampleList.add(new BuoyancyTest(testbed));
    	exampleList.add(new RaycastTest(testbed));
    	
//    	exampleList.add(new BlobTest(testbed));
    	
    	// Shape drawing demo
    	exampleList.add(new ShapeDrawing(testbed));
    	
    	exampleList.add(new CharacterControlTest(testbed));

    	
    	// Stress tests
    	exampleList.add(new Pyramid(testbed));
    	exampleList.add(new Circles(testbed));
    	exampleList.add(new DominoTower(testbed));
    	
    	// Bug tests
//    	exampleList.add(new DistanceTest(testbed));
    	//exampleList.add(new BugTest(testbed));
//*/
    	return exampleList;
	}
}
