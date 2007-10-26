/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/ 
 * Box2D homepage: http://www.gphysics.com
 * 
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/ 
 * Box2D homepage: http://www.gphysics.com
 * 
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
package dynamics.joints;

import common.Vec2;

public class PulleyJointDef extends JointDef {

    public Vec2 groundPoint1;

    public Vec2 groundPoint2;

    public Vec2 anchorPoint1;

    public Vec2 anchorPoint2;

    public float maxLength1;

    public float maxLength2;

    public float ratio;

    public PulleyJointDef() {
        type = JointType.PULLEY_JOINT;
        groundPoint1 = new Vec2(-1.0f, 1.0f);
        groundPoint2 = new Vec2(1.0f, 1.0f);
        anchorPoint1 = new Vec2(-1.0f, 0.0f);
        anchorPoint2 = new Vec2(1.0f, 0.0f);
        maxLength1 = 0.5f * PulleyJoint.MIN_PULLEY_LENGTH;
        maxLength2 = 0.5f * PulleyJoint.MIN_PULLEY_LENGTH;
        ratio = 1.0f;
        collideConnected = true;
    }
}
