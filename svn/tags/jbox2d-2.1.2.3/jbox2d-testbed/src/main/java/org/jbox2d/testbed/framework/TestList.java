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
import org.jbox2d.testbed.tests.Chain;
import org.jbox2d.testbed.tests.CharacterCollision;
import org.jbox2d.testbed.tests.CircleStress;
import org.jbox2d.testbed.tests.CollisionFiltering;
import org.jbox2d.testbed.tests.CollisionProcessing;
import org.jbox2d.testbed.tests.CompoundShapesTest;
import org.jbox2d.testbed.tests.ConfinedTest;
import org.jbox2d.testbed.tests.ContinuousTest;
import org.jbox2d.testbed.tests.DistanceTest;
import org.jbox2d.testbed.tests.DominoTest;
import org.jbox2d.testbed.tests.DominoTower;
import org.jbox2d.testbed.tests.DynamicTreeTest;
import org.jbox2d.testbed.tests.EdgeShapes;
import org.jbox2d.testbed.tests.Gears;
import org.jbox2d.testbed.tests.LineJointTest;
import org.jbox2d.testbed.tests.LiquidTest;
import org.jbox2d.testbed.tests.OneSidedTest;
import org.jbox2d.testbed.tests.PolyShapes;
import org.jbox2d.testbed.tests.PrismaticTest;
import org.jbox2d.testbed.tests.Pulleys;
import org.jbox2d.testbed.tests.PyramidTest;
import org.jbox2d.testbed.tests.RayCastTest;
import org.jbox2d.testbed.tests.RevoluteTest;
import org.jbox2d.testbed.tests.SensorTest;
import org.jbox2d.testbed.tests.ShapeEditing;
import org.jbox2d.testbed.tests.SliderCrankTest;
import org.jbox2d.testbed.tests.SphereStack;
import org.jbox2d.testbed.tests.TheoJansen;
import org.jbox2d.testbed.tests.VaryingFrictionTest;
import org.jbox2d.testbed.tests.VaryingRestitution;
import org.jbox2d.testbed.tests.VerticalStack;
import org.jbox2d.testbed.tests.Web;

/**
 * @author Daniel Murphy
 */
public class TestList {
  
  public static void populateModel(TestbedModel argModel){
      
      argModel.addCategory("Featured");
      argModel.addTest(new DominoTest());
      argModel.addTest(new CompoundShapesTest());
      argModel.addTest(new BlobTest4());
      argModel.addTest(new TheoJansen());
      
      // watching...
      argModel.addCategory("Collision Watching");
      argModel.addTest(new VaryingRestitution());
      argModel.addTest(new VaryingFrictionTest());
      argModel.addTest(new SphereStack());
      argModel.addTest(new VerticalStack());
      argModel.addTest(new PyramidTest());
      argModel.addTest(new DominoTower());
      argModel.addTest(new CircleStress());
      
      // more interactive..
      argModel.addCategory("Interactive");
      argModel.addTest(new ShapeEditing());
      argModel.addTest(new Breakable());
      argModel.addTest(new OneSidedTest());
      argModel.addTest(new PolyShapes());
      argModel.addTest(new BodyTypes());
      argModel.addTest(new CharacterCollision());
      argModel.addTest(new ApplyForce());

      // processing/filtering
      argModel.addCategory("Processing/Filtering");
      argModel.addTest(new CollisionFiltering());
      argModel.addTest(new CollisionProcessing());
      argModel.addTest(new SensorTest());

      // joints
      argModel.addCategory("Joints");
      argModel.addTest(new PrismaticTest());
      argModel.addTest(new RevoluteTest());
      argModel.addTest(new Pulleys());
      argModel.addTest(new LineJointTest());
      argModel.addTest(new Gears());
      argModel.addTest(new Web());
      argModel.addTest(new Chain());
      argModel.addTest(new Cantilever());
      argModel.addTest(new SliderCrankTest());
      
      // ccd
      argModel.addCategory("CCD");
      argModel.addTest(new ContinuousTest());
      argModel.addTest(new ConfinedTest());
      
      // raycast
      argModel.addCategory("Raycast");
      argModel.addTest(new RayCastTest());
      argModel.addTest(new EdgeShapes());

      // misc
      argModel.addCategory("Misc");
      argModel.addTest(new DynamicTreeTest());
      argModel.addTest(new DistanceTest());
      argModel.addTest(new LiquidTest());
  }
}
