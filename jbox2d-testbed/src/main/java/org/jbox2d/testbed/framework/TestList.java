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
 * Created at 5:34:33 PM Jul 17, 2010
 */
package org.jbox2d.testbed.framework;

import org.jbox2d.testbed.tests.ApplyForce;
import org.jbox2d.testbed.tests.BlobTest4;
import org.jbox2d.testbed.tests.BodyTypes;
import org.jbox2d.testbed.tests.Breakable;
import org.jbox2d.testbed.tests.Cantilever;
import org.jbox2d.testbed.tests.Car;
import org.jbox2d.testbed.tests.Chain;
import org.jbox2d.testbed.tests.CharacterCollision;
import org.jbox2d.testbed.tests.CircleStress;
import org.jbox2d.testbed.tests.CollisionFiltering;
import org.jbox2d.testbed.tests.CollisionProcessing;
import org.jbox2d.testbed.tests.CompoundShapes;
import org.jbox2d.testbed.tests.ConfinedTest;
import org.jbox2d.testbed.tests.ContinuousTest;
import org.jbox2d.testbed.tests.ConvexHull;
import org.jbox2d.testbed.tests.ConveyorBelt;
import org.jbox2d.testbed.tests.DistanceTest;
import org.jbox2d.testbed.tests.DominoTest;
import org.jbox2d.testbed.tests.DominoTower;
import org.jbox2d.testbed.tests.DynamicTreeTest;
import org.jbox2d.testbed.tests.EdgeShapes;
import org.jbox2d.testbed.tests.FixedPendulumTest;
import org.jbox2d.testbed.tests.FreePendulumTest;
import org.jbox2d.testbed.tests.Gears;
import org.jbox2d.testbed.tests.LiquidTest;
import org.jbox2d.testbed.tests.OneSidedTest;
import org.jbox2d.testbed.tests.PistonTest;
import org.jbox2d.testbed.tests.PolyShapes;
import org.jbox2d.testbed.tests.PrismaticTest;
import org.jbox2d.testbed.tests.Pulleys;
import org.jbox2d.testbed.tests.PyramidTest;
import org.jbox2d.testbed.tests.RayCastTest;
import org.jbox2d.testbed.tests.RevoluteTest;
import org.jbox2d.testbed.tests.RopeTest;
import org.jbox2d.testbed.tests.SensorTest;
import org.jbox2d.testbed.tests.ShapeEditing;
import org.jbox2d.testbed.tests.SliderCrankTest;
import org.jbox2d.testbed.tests.SphereStack;
import org.jbox2d.testbed.tests.TheoJansen;
import org.jbox2d.testbed.tests.Tumbler;
import org.jbox2d.testbed.tests.VaryingFrictionTest;
import org.jbox2d.testbed.tests.VaryingRestitution;
import org.jbox2d.testbed.tests.VerticalStack;
import org.jbox2d.testbed.tests.Web;

/**
 * @author Daniel Murphy
 */
public class TestList {

  public static void populateModel(TestbedModel model) {

    model.addCategory("Featured");
    model.addTest(new Car());
    model.addTest(new DominoTest());
    model.addTest(new CompoundShapes());
    model.addTest(new BlobTest4());
    model.addTest(new TheoJansen());

    // watching...
    model.addCategory("Collision Watching");
    model.addTest(new VaryingRestitution());
    model.addTest(new VaryingFrictionTest());
    model.addTest(new ConveyorBelt());
    model.addTest(new SphereStack());
    model.addTest(new Tumbler());
    model.addTest(new PistonTest());
    model.addTest(new PyramidTest());
    model.addTest(new CircleStress());
    model.addTest(new DominoTower());

    // more interactive..
    model.addCategory("Interactive");
    model.addTest(new VerticalStack());
    model.addTest(new Breakable());
    model.addTest(new ShapeEditing());
    model.addTest(new OneSidedTest());
    model.addTest(new PolyShapes());
    model.addTest(new BodyTypes());
    model.addTest(new CharacterCollision());
    model.addTest(new ApplyForce());

    // processing/filtering
    model.addCategory("Processing/Filtering");
    model.addTest(new CollisionFiltering());
    model.addTest(new CollisionProcessing());
    model.addTest(new SensorTest());

    // joints
    model.addCategory("Joints");
    model.addTest(new PrismaticTest());
    model.addTest(new RevoluteTest());
    model.addTest(new FixedPendulumTest(true));
    model.addTest(new FreePendulumTest(true));
    model.addTest(new Chain());
    model.addTest(new RopeTest());
    model.addTest(new Pulleys());
    model.addTest(new Gears());
    model.addTest(new Web());
    model.addTest(new Cantilever());
    model.addTest(new SliderCrankTest());

    // ccd
    model.addCategory("CCD");
    model.addTest(new ContinuousTest());
    model.addTest(new ConfinedTest());

    // raycast
    model.addCategory("Raycast");
    model.addTest(new RayCastTest());
    model.addTest(new EdgeShapes());

    // misc
    model.addCategory("Misc");
    model.addTest(new ConvexHull());
    model.addTest(new DynamicTreeTest());
    model.addTest(new DistanceTest());
    model.addTest(new LiquidTest());
  }
}
