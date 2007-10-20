package collision;

import common.Vec2;
import common.Mat22;

public class OBB {
    public Mat22 R;

    public Vec2 center;

    public Vec2 extents;

    public OBB(Mat22 _R, Vec2 _center, Vec2 _extents) {
        R = _R.clone();
        center = _center.clone();
        extents = _extents.clone();
    }

    public OBB(OBB copy) {
        this(copy.R.clone(), copy.center.clone(), copy.extents.clone());
    }

    public OBB() {
        R = new Mat22();
        center = new Vec2();
        extents = new Vec2();
    }

    public OBB clone() {
        return new OBB(this);
    }

}