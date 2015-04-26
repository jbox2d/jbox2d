jbox2d
======
JBox2d is a 2D Java physics engine ported from the C++ physics engines [LiquidFun](http://google.github.io/liquidfun/) and [Box2d](http://box2d.org).
See the project page online for more information: http://code.google.com/p/jbox2d/

**Please see the [project's BountySource page](https://www.bountysource.com/teams/jbox2d) to vote on issues that matter to you.**  Commenting/voting on issues helps me prioritize the small amount of time I have to maintain this library :)

**Watch/star to follow along with progress!**

If you're looking for *help*, see the [wiki](https://github.com/jbox2d/jbox2d/wiki) or come visit us at the [Java Box2d subforum](http://box2d.org/forum/viewforum.php?f=9).
If you're looking to deploy on the web, see [PlayN](https://code.google.com/p/playn/), which compiles JBox2d through GWT so it runs in the browser.  The JBox2d library has GWT support out of the box.   Also, [TeaVM](http://teavm.org/) support jbox2d in the browser as well.

### Report Issues

- **[Report New Issue](https://github.com/jbox2d/jbox2d/issues/new )**
- **[View Open Issues](https://github.com/jbox2d/jbox2d/issues )**

If you're planning on maintaining/customizing your *own copy* of the code, please join our [group](http://groups.google.com/group/jbox2d-announce) so we can keep you updated.

If you've downloaded this as an archive, you should find the built java jars in the 'target' directories of each project.

- jbox2d-library - this is the main physics library.  The only dependency is the SLF4J logging library.
- jbox2d-serialization - this adds serialization tools.  Requires google's protocol buffer library installed to fully build (http://code.google.com/p/protobuf/), but this is optional, as the generated sources are included.
- jbox2d-testbed - A simple framework for creating and running physics tests.
- jbox2d-testbed-jogl - The testbed with OpenGL rendering.
- jbox2d-jni-broadphase - Experiment with moving parts of the engine to C++.  Not faster.

***

###Licenece
Copyright (c) 2014, Daniel Murphy
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimer in the documentation and/or
  other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

***
