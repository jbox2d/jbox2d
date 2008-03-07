package org.jbox2d.collision;

import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;

//Related to rev 108 of b2Distance.cpp
//Necessary for DistanceGeneric(Vec2 x1, Vec2 x2,
//SupportsGenericDistance shape1, XForm xf1,
//SupportsGenericDistance shape2, XForm xf2) function (cleaner than template)

public interface SupportsGenericDistance {
	public Vec2 support(XForm xf, Vec2 v);
	public Vec2 getFirstVertex(XForm xf);

}
