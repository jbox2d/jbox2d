jbox2d
======

**This project is currently being moved from [Google Code](https://code.google.com/p/jbox2d/) to Github.**

JBox2d is a Java port of the C++ physics engines [LiquidFun](http://google.github.io/liquidfun/) and [Box2d](http://box2d.org).

If you're looking for *help*, see the [old wiki](http://code.google.com/p/jbox2d/w/list) or come visit us at the [Java Box2d subforum](http://box2d.org/forum/viewforum.php?f=9).  Please post bugs here on the [issues](https://github.com/dmurph/jbox2d/issues) page.

If you're planning on maintaining/customizing your *own copy* of the code, please join our [group](http://groups.google.com/group/jbox2d-announce) so we can keep you updated.

If you're looking to deploy on the web, see [PlayN](https://code.google.com/p/playn/), which compiles JBox2d through GWT so it runs in the browser.  The JBox2d library has GWT support out of the box.


If you've downloaded this as an archive, you should find the built java jars in the 'target' directories of each project.

======

jbox2d-library - this is the main physics library.  The only dependency is the SLF4J logging library.

jbox2d-serialization - this adds serialization tools.  Requires google's protocol buffer library installed to fully build (http://code.google.com/p/protobuf/), but this is optional, as the generated sources are included.

jbox2d-testbed - A simple framework for creating and running physics tests.

jbox2d-testbed-jogl - The testbed with OpenGL rendering.

jbox2d-jni-broadphase - Experiment with moving parts of the engine to C++.  Not faster.
