-------------
JBox2d README
-------------

JBox2d (http://www.sf.net/projects/jbox2d) is a Java port of Box2d (http://www.box2d.org), a 2D rigid body physics engine written in C++.
Box2d is maintained by Erin Catto.
JBox2d is maintained by quixote_arg, ewjordan, and dmurphy (aka toucansam).

There is also an Actionscript 3 version of this engine for Flash programmers, maintained by skatehead.

Issues can be discussed at the Box2d forums at http://www.box2d.org/forum/

In the dist folder, you will find two jars. The Library jar only has the physics library included, while the Full jar includes the processing jar classes (core.jar) and the tests and testbed.  You should really only need the library.

It is highly recommended that you enable assertions when debugging a JBox2d program.  In Eclipse, this is done by adding -ea to the "Run"->"Open Run Dialog"->"Arguments"->"VM arguments" entry for your run configuration.
The assertions in JBox2d are generally lightweight, so shouldn't cause much of a runtime hit.  If you encounter one, it's probably because you've done something wrong, and the engine will likely not work properly with that input.  If you're sure you haven't done anything bad (please follow the assert to the source code so you can see if there is an explanatory comment), please let us know at the forums.

Current release version: 2.0.1
Current SVN version: 2.0.1+

----
Docs
----

Currently JBox2d has Javadocs (see the doc directory), but no separate documentation.
Please refer to the Box2d manual for the time being, keeping in mind the following syntax differences between Box2d and JBox2d:
- Class names generally lack the b2 prefix in JBox2d.  For instance:
  [C++]      [Java]
  b2Shape -> Shape
  b2Vec2  -> Vec2
  etc.
- Method names are camel-cased in JBox2d:
  myb2Vec2.Dot(otherb2Vec2) -> myVec2.dot(otherVec2)
  ThisIsAMethod(parameter) -> thisIsAMethod(parameter)
- Built-in types don't have bit-length modifiers:
  float32 -> float
  int32   -> int
  uint8   -> int (nothing is gained by using smaller data types in Java)
- Generally speaking, pointers turn into references in JBox2d:
  m_vehicle->CreateShape(&poly1); -> m_vehicle.createShape(poly1);
  As a rule of thumb, you can usually start by just deleting any *s and &s that you see in the C++ code, and resolve any problems as they arise.
- Polygon definitions use a List<Vec2> instead of a preallocated array:
  Box2d (C++):
  	b2PolygonDef sd;
  	sd.vertexCount = 3;
	sd.vertices[0].Set(-0.5f, 0.0f);
	sd.vertices[1].Set(0.5f, 0.0f);
	sd.vertices[2].Set(0.0f, 1.5f);
  JBox2d (Java):
    PolygonDef sd = new PolygonDef();
    sd.vertices.add(new Vec2(-0.5f, 0.0f));
    sd.vertices.add(new Vec2(0.5f, 0.0f));
    sd.vertices.add(new Vec2(0.0f, 1.5f));
- Global methods in the C++ version had to be put into classes for the Java one.
  The choices as to where to put these were largely arbitrary, though they usually correspond loosely to whatever .cpp/.h file they were declared within in the C++ version.
  These functions are mostly for internal use only, so it shouldn't affect you too much.
- No operator overloading for Vec2/Mat22 methods:
	myb2VecA = vecb + vecc  ->  myVecA = vecb.add(vecc);
- Math-related b2* methods are added to the relevant classes, unless they are already present in Math:
	b2Dot(veca, vecb)     -> Vec2.dot(veca, vecb)
	b2Mul(matrixa, vecb)  -> Mat22.mul(matrixa, vecb)
	b2Max(floata, floatb) -> Math.max(floata, floatb)
 
We strongly recommend using Eclipse, NetBeans, or some other Java IDE that has syntax highlighting and code hints, as this will help you ensure that you are using the appropriate methods.  It's also good for your productivity overall.

We also recommend that you link to the JBox2d source instead of just a .jar so that if you hit assertions you can jump to their location in code.

------
Issues
------
.jar distribution: Three .jars are contained in this directory, jbox2d-2.0.1-full.jar, which is the full distribution, and includes Javadoc, source code attachments, and all examples.  The jbox2d-2.0.1-library-only.jar file is a trimmed down version for release, weighing in at only 156 kb.  The jbox2d-2.0.1-speedTest.jar is an executable .jar for running a basic speed test.

Performance: We're aware that JBox2d does not achieve the same impressive performance as Box2d.  Some of this is an unavoidable limitation of Java, some of it is due to lack of Java-specific optimizations, and some of it is due to rendering.  This should improve a bit as we further optimize, but we're not expecting miracles.

Lack of Java-ness: This is a difficult issue for us.  We would love to make all the fields private, and refactor everything like mad until it actually looks like Java code.  But we are trying to balance Java-ness, maintainability, and performance, and until the C++ version stabilizes a bit more we need to keep very close to its organizational structure, which means public fields all over the place.  We're aware of this issue, and we take it seriously, but for now it will have to wait if we want to have any chance of keeping up to date.

Dependencies in testbed: Right now the testbed code depends on the Processing core library (http://www.processing.org) for drawing.  Some people have asked for a plain old AWT version of the tests, which we will do if we have time, but it's not a huge priority since it seems like most people are using game engines of some sort.  We also might add Slick [2.0: simple example exists now], PulpCore [in progress], and JMonkeyEngine demos.  Ultimately, though, it's up to the end user to figure out their own drawing code - it's really not that tough if you look at the way we do it with Processing, the methods are pretty transparent (drawLine(x0,y0,x1,y1) is easy to translate to any engine, no?).

Other bugs: JBox2d should produce roughly the same output as Box2d.  The exact numerical values may be off by a little bit due to different floating point handling, but we're really shooting for identical large-scale behavior.  If you see any problems, please tell us about them!