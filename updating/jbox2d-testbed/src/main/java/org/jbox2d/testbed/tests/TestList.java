/*******************************************************************************
 * Copyright (c) 2011, Daniel Murphy
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL DANIEL MURPHY BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
/**
 * Created at 5:34:33 PM Jul 17, 2010
 */
package org.jbox2d.testbed.tests;

import java.util.ArrayList;

import org.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class TestList {
	public static final ArrayList<TestbedTest> tests = new ArrayList<TestbedTest>();
	static{
		
		// general collision
		// watching...
		tests.add(new VaryingRestitution());
		tests.add(new VaryingFrictionTest());
		tests.add(new SphereStack());
		tests.add(new CompoundShapesTest());
		tests.add(new DominoTest());
		tests.add(new VerticalStack());
		tests.add(new PyramidTest());
		tests.add(new DominoTower());
		tests.add(new CircleStress());
		
		// more interactive..
		tests.add(new ShapeEditing());
		tests.add(new Breakable());
		tests.add(new OneSidedTest());
		tests.add(new PolyShapes());
		tests.add(new BodyTypes());
		tests.add(new CharacterCollision());
		tests.add(new ApplyForce());

		// processing/filtering
		tests.add(new CollisionFiltering());
		tests.add(new CollisionProcessing());
		tests.add(new SensorTest());

		// joints
		tests.add(new PrismaticTest());
		tests.add(new RevoluteTest());
		tests.add(new Pulleys());
		tests.add(new LineJointTest());
		tests.add(new Gears());
		tests.add(new Web());
		tests.add(new Chain());
		tests.add(new Cantilever());
		tests.add(new SliderCrankTest());
		tests.add(new BlobTest4());
		tests.add(new TheoJansen());
		
		// ccd
		tests.add(new ContinuousTest());
		tests.add(new ConfinedTest());
		
		// raycast
		tests.add(new RayCastTest());
		tests.add(new EdgeShapes());

		// misc
		tests.add(new DynamicTreeTest());
		tests.add(new DistanceTest());
		tests.add(new LiquidTest());
	}
}
