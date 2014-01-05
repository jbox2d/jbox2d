jbox2d
======

a 2d Java physics engine, native port of the C++ Box2d engine


If you've downloaded this as an archive, you should find the built java jars in the 'target' directories of each project.

jbox2d-library - this is the main physics library.  The only dependency is the SLF4J logging library.

jbox2d-serialization - this adds serialization tools.  Requires google's protocol buffer library installed to fully build (http://code.google.com/p/protobuf/), but this is optional, as the generated sources are included.

jbox2d-testbed - A simple framework for creating and running physics tests.