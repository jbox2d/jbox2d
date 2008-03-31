package org.jbox2d.util.nonconvex;

import org.jbox2d.collision.PolygonDef;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.BodyDef;

/**
 * This code is very old and doesn't work anymore.
 * Will be updated for 2.0 soon...
 * 
 * @author ewjordan
 */
public class Polygon {
//
//    public static int maxVerticesPerPolygon = org.jbox2d.common.Settings.maxPolygonVertices;
//
//    public float[] x;
//
//    public float[] y;
//
//    public int nVertices;
//
//    public Polygon(float[] _x, float[] _y) {
//    	nVertices = _x.length;
//        // println("length "+nVertices);
//        x = new float[nVertices];
//        y = new float[nVertices];
//        for (int i = 0; i < nVertices; ++i) {
//            x[i] = _x[i];
//            y[i] = _y[i];
//        }
//    }
//
//    public Polygon(Vec2[] v) {
//        nVertices = v.length;
//        x = new float[nVertices];
//        y = new float[nVertices];
//        for (int i = 0; i < nVertices; ++i) {
//            x[i] = v[i].x;
//            y[i] = v[i].y;
//        }
//    }
//
//    public Vec2[] getVertexVecs() {
//        Vec2[] out = new Vec2[nVertices];
//        for (int i = 0; i < nVertices; ++i) {
//            out[i] = new Vec2(x[i], y[i]);
//        }
//        return out;
//    }
//
//    public Polygon(Triangle t) {
//        this(t.x, t.y);
//    }
//
//    public void set(Polygon p) {
//        nVertices = p.nVertices;
//        x = new float[nVertices];
//        y = new float[nVertices];
//        for (int i = 0; i < nVertices; ++i) {
//            x[i] = p.x[i];
//            y[i] = p.y[i];
//        }
//    }
//
//    /*
//     * Assuming the polygon is simple, checks if it is convex.
//     */
//    public boolean isConvex() {
//        boolean isPositive = false;
//        for (int i = 0; i < nVertices; ++i) {
//            int lower = (i == 0) ? (nVertices - 1) : (i - 1);
//            int middle = i;
//            int upper = (i == nVertices - 1) ? (0) : (i + 1);
//            float dx0 = x[middle] - x[lower];
//            float dy0 = y[middle] - y[lower];
//            float dx1 = x[upper] - x[middle];
//            float dy1 = y[upper] - y[middle];
//            float cross = dx0 * dy1 - dx1 * dy0;
//            // Cross product should have same sign
//            // for each vertex if poly is convex.
//            boolean newIsP = (cross >= 0) ? true : false;
//            if (i == 0) {
//                isPositive = newIsP;
//            }
//            else if (isPositive != newIsP) {
//                return false;
//            }
//        }
//        return true;
//    }
//
//    public boolean isSimple() {
//        //FIXME: Not implemented
//        return true;
//    }
//    
//    /*
//     * Tries to add a triangle to the polygon. Returns null if it can't connect
//     * properly. Assumes bitwise equality of joined vertices.
//     */
//    public Polygon add(Triangle t) {
//        // First, find vertices that connect
//        int firstP = -1;
//        int firstT = -1;
//        int secondP = -1;
//        int secondT = -1;
//        for (int i = 0; i < nVertices; i++) {
//            if (t.x[0] == x[i] && t.y[0] == y[i]) {
//                if (firstP == -1) {
//                    firstP = i;
//                    firstT = 0;
//                }
//                else {
//                    secondP = i;
//                    secondT = 0;
//                }
//            }
//            else if (t.x[1] == x[i] && t.y[1] == y[i]) {
//                if (firstP == -1) {
//                    firstP = i;
//                    firstT = 1;
//                }
//                else {
//                    secondP = i;
//                    secondT = 1;
//                }
//            }
//            else if (t.x[2] == x[i] && t.y[2] == y[i]) {
//                if (firstP == -1) {
//                    firstP = i;
//                    firstT = 2;
//                }
//                else {
//                    secondP = i;
//                    secondT = 2;
//                }
//            }
//            else {
//            }
//        }
//        // Fix ordering if first should be last vertex of poly
//        if (firstP == 0 && secondP == nVertices - 1) {
//            firstP = nVertices - 1;
//            secondP = 0;
//        }
//
//        // Didn't find it
//        if (secondP == -1)
//            return null;
//
//        // Find tip index on triangle
//        int tipT = 0;
//        if (tipT == firstT || tipT == secondT)
//            tipT = 1;
//        if (tipT == firstT || tipT == secondT)
//            tipT = 2;
//
//        float[] newx = new float[nVertices + 1];
//        float[] newy = new float[nVertices + 1];
//        int currOut = 0;
//        for (int i = 0; i < nVertices; i++) {
//            newx[currOut] = x[i];
//            newy[currOut] = y[i];
//            if (i == firstP) {
//                ++currOut;
//                newx[currOut] = t.x[tipT];
//                newy[currOut] = t.y[tipT];
//            }
//            ++currOut;
//        }
//        return new Polygon(newx, newy);
//    }
//
//    /*
//     * Adds this polygon to a PolyDef.
//     */
//    public void addTo(PolygonDef pd) {
//        Vec2[] vecs = getVertexVecs();
//        for (int i = 0; i < vecs.length; ++i) {
//            pd.vertices.add(vecs[i]);
//        }
//    }
//
//    /*
//     * public void draw(){ draw(color(150,150,150),color(255,0,0)); }
//     * 
//     * public void draw(int fll, int strk){ // fill(fll); noFill();
//     * stroke(strk); beginShape(POLYGON); for (int i=0; i<x.length; i++){
//     * vertex(x[i],y[i]); } endShape(); }
//     */
//
//    /**
//     * Triangulates a polygon using simple O(N^2) ear-clipping algorithm. Returns
//     * a Triangle array unless the polygon can't be triangulated, in which case
//     * null is returned. This should only happen if the polygon self-intersects,
//     * though it will not _always_ return null for a bad polygon - it is the
//     * caller's responsibility to check for self-intersection, and if it
//     * doesn't, it should at least check that the return value is non-null
//     * before using. You're warned!
//     * 
//     * This is totally unoptimized, so for large polygons it should not be part
//     * of the simulation loop. For smaller ones it's less of an issue.
//     */
//    public static Triangle[] triangulatePolygon(float[] xv, float[] yv, int vNum) {
//        if (vNum < 3)
//            return null;
//
//        Triangle[] buffer = new Triangle[vNum];
//        int bufferSize = 0;
//        float[] xrem = new float[vNum];
//        float[] yrem = new float[vNum];
//        for (int i = 0; i < vNum; ++i) {
//            xrem[i] = xv[i];
//            yrem[i] = yv[i];
//        }
//
//        while (vNum > 3) {
//            // Find an ear
//            int earIndex = -1;
//            for (int i = 0; i < vNum; ++i) {
//                if (isEar(i, xrem, yrem)) {
//                    earIndex = i;
//                    break;
//                }
//            }
//
//            // If we still haven't found an ear, we're screwed.
//            // The user did Something Bad, so return null.
//            // This will probably crash their program, since
//            // they won't bother to check the return value.
//            // At this we shall laugh, heartily and with great gusto.
//            if (earIndex == -1)
//                return null;
//
//            // Clip off the ear:
//            // - remove the ear tip from the list
//
//            // Opt note: actually creates a new list, maybe
//            // this should be done in-place instead. A linked
//            // list would be even better to avoid array-fu.
//            --vNum;
//            float[] newx = new float[vNum];
//            float[] newy = new float[vNum];
//            int currDest = 0;
//            for (int i = 0; i < vNum; ++i) {
//                if (currDest == earIndex)
//                    ++currDest;
//                newx[i] = xrem[currDest];
//                newy[i] = yrem[currDest];
//                ++currDest;
//            }
//
//            // - add the clipped triangle to the triangle list
//            int under = (earIndex == 0) ? (xrem.length - 1) : (earIndex - 1);
//            int over = (earIndex == xrem.length - 1) ? 0 : (earIndex + 1);
//
//            Triangle toAdd = new Triangle(xrem[earIndex], yrem[earIndex],
//                    xrem[over], yrem[over], xrem[under], yrem[under]);
//            buffer[bufferSize] = toAdd;
//            ++bufferSize;
//
//            // - replace the old list with the new one
//            xrem = newx;
//            yrem = newy;
//        }
//        Triangle toAdd = new Triangle(xrem[1], yrem[1], xrem[2], yrem[2],
//                xrem[0], yrem[0]);
//        buffer[bufferSize] = toAdd;
//        ++bufferSize;
//
//        Triangle[] res = new Triangle[bufferSize];
//        for (int i = 0; i < bufferSize; i++) {
//            res[i] = buffer[i];
//        }
//        return res;
//    }
//
//    /**
//     * Turns a list of triangles into a list of convex polygons. Very simple
//     * method - start with a seed triangle, keep adding triangles to it until
//     * you can't add any more without making the polygon non-convex.
//     * 
//     * Takes O(N*P) where P is the number of resultant polygons, N is triangle
//     * count.
//     * 
//     * The final polygon list will not necessarily be minimal, though in
//     * practice it works fairly well.
//     */
//    public static Polygon[] polygonizeTriangles(Triangle[] triangulated) {
//        Polygon[] polys;
//        int polyIndex = 0;
//
//        if (triangulated == null) {
//            return null;
//        }
//        else {
//            polys = new Polygon[triangulated.length];
//            boolean[] covered = new boolean[triangulated.length];
//            for (int i = 0; i < triangulated.length; i++) {
//                covered[i] = false;
//            }
//
//            boolean notDone = true;
//
//            while (notDone) {
//                int currTri = -1;
//                for (int i = 0; i < triangulated.length; i++) {
//                    if (covered[i])
//                        continue;
//                    currTri = i;
//                    break;
//                }
//                if (currTri == -1) {
//                    notDone = false;
//                }
//                else {
//                    Polygon poly = new Polygon(triangulated[currTri]);
//                    covered[currTri] = true;
//                    for (int i = 0; i < triangulated.length; i++) {
//                        if (covered[i])
//                            continue;
//                        Polygon newP = poly.add(triangulated[i]);
//                        if (newP == null)
//                            continue;
//                        if (newP.nVertices > maxVerticesPerPolygon)
//                            break;
//                        if (newP.isConvex()) {
//                            poly = newP;
//                            covered[i] = true;
//                        }
//                    }
//                    polys[polyIndex] = poly;
//                    polyIndex++;
//                }
//            }
//        }
//        Polygon[] ret = new Polygon[polyIndex];
//        for (int i = 0; i < polyIndex; i++) {
//            ret[i] = polys[i];
//        }
//        return ret;
//    }
//
//    /**
//     * Checks if vertex i is the tip of an ear in polygon defined by xv[] and
//     * yv[]
//     */
//    private static boolean isEar(int i, float[] xv, float[] yv) {
//        float dx0, dy0, dx1, dy1;
//        dx0 = dy0 = dx1 = dy1 = 0;
//        if (i >= xv.length || i < 0 || xv.length < 3) {
//            return false;
//        }
//        int upper = i + 1;
//        int lower = i - 1;
//        if (i == 0) {
//            dx0 = xv[0] - xv[xv.length - 1];
//            dy0 = yv[0] - yv[yv.length - 1];
//            dx1 = xv[1] - xv[0];
//            dy1 = yv[1] - yv[0];
//            lower = xv.length - 1;
//        }
//        else if (i == xv.length - 1) {
//            dx0 = xv[i] - xv[i - 1];
//            dy0 = yv[i] - yv[i - 1];
//            dx1 = xv[0] - xv[i];
//            dy1 = yv[0] - yv[i];
//            upper = 0;
//        }
//        else {
//            dx0 = xv[i] - xv[i - 1];
//            dy0 = yv[i] - yv[i - 1];
//            dx1 = xv[i + 1] - xv[i];
//            dy1 = yv[i + 1] - yv[i];
//        }
//        float cross = dx0 * dy1 - dx1 * dy0;
//        if (cross > 0)
//            return false;
//        Triangle myTri = new Triangle(xv[i], yv[i], xv[upper], yv[upper],
//                xv[lower], yv[lower]);
//        for (int j = 0; j < xv.length; ++j) {
//            if (j == i || j == lower || j == upper)
//                continue;
//            if (myTri.isInside(xv[j], yv[j]))
//                return false;
//        }
//        return true;
//    }
//
//    private static void reversePolygon(float[] x, float[] y, int n) {
//        if (n == 1)
//            return;
//        int low = 1;
//        int high = n - 1;
//        while (low < high) {
//            float buffer = x[low];
//            x[low] = x[high];
//            x[high] = buffer;
//            buffer = y[low];
//            y[low] = y[high];
//            y[high] = buffer;
//            ++low;
//            --high;
//        }
//    }
//
//    /**
//     * Decomposes a non-convex polygon into a number of convex polygons. Makes
//     * sure that each polygon has no more than common.Settings.maxPolyVertices
//     * vertices.
//     * 
//     * Returns null if operation fails (usually due to self-intersection of
//     * polygon). Please report any other failures to ewjordan at gmail.
//     */
//    public static Polygon[] decomposeConvex(Polygon p) {
//        Triangle[] triangulated = triangulatePolygon(p.x, p.y, p.nVertices);
//        if (triangulated == null) {
//            // For some reason the triangulation failed - we'll
//            // try reversing the orientation.
//            reversePolygon(p.x, p.y, p.nVertices);
//            triangulated = triangulatePolygon(p.x, p.y, p.nVertices);
//            reversePolygon(p.x, p.y, p.nVertices); // Set the orientation back!
//        }
//        if (triangulated == null)
//            return null;
//        Polygon[] polys = polygonizeTriangles(triangulated);
//        return polys;
//    }
//
//    /**
//     * Decomposes a polygon into convex polygons and adds all pieces to BodyDef
//     * using a prototype PolyDef. All fields of the prototype are used for every
//     * shape except the vertices (friction, restitution, density, etc).
//     * 
//     * If you want finer control, you'll have to add everything by hand.
//     * 
//     * This is the simplest method to add a complicated polygon to a body.
//     */
//    public static void decomposeConvexAndAddTo(Polygon p, BodyDef bd,
//            PolygonDef prototype) {
//        Polygon[] decomposed = decomposeConvex(p);
//        for (int i = 0; i < decomposed.length; ++i) {
//            PolygonDef toAdd = new PolygonDef();
//
//            // This would be better wrapped up as a clone() method
//            // in the PolyDef class in case it changes...
//            toAdd.type = prototype.type;
//            toAdd.localPosition = prototype.localPosition.clone();
//            toAdd.localRotation = prototype.localRotation;
//            toAdd.friction = prototype.friction;
//            toAdd.restitution = prototype.restitution;
//            toAdd.density = prototype.density;
//            toAdd.userData = prototype.userData;
//            toAdd.categoryBits = prototype.categoryBits;
//            toAdd.maskBits = prototype.maskBits;
//            toAdd.groupIndex = prototype.groupIndex;
//
//            decomposed[i].addTo(toAdd);
//            bd.addShape(toAdd);
//        }
//    }
//
//    /**
//     * Find the convex hull of a point cloud using "Gift-wrap" algorithm - start
//     * with an extremal point, and walk around the outside edge by testing
//     * angles.
//     * 
//     * Runs in O(N*S) time where S is number of sides of resulting polygon.
//     * Worst case: point cloud is all vertices of convex polygon -> O(N^2).
//     * 
//     * There may be faster algorithms to do this, should you be needy for one -
//     * this is just the simplest. You can get O(N log N) expected time if you
//     * try, I think.
//     * 
//     * Returns null if number of vertices passed is less than 3.
//     * 
//     * FIXME?: May be buggy with colinear points on hull. Couldn't find a test
//     * case that resulted in wrong behavior. If one turns up, the solution is to
//     * supplement angle check with an equality resolver that always picks the
//     * longer edge. I think the current solution is working, though.
//     */
//    public static Polygon convexHull(Vec2[] v) {
//        return convexHull(v, v.length);
//    }
//
//    public static Polygon convexHull(Vec2[] v, int nVert) {
//        float[] cloudX = new float[nVert];
//        float[] cloudY = new float[nVert];
//        for (int i = 0; i < nVert; ++i) {
//            cloudX[i] = v[i].x;
//            cloudY[i] = v[i].y;
//        }
//        return convexHull(cloudX, cloudY, nVert);
//    }
//
//    public static Polygon convexHull(float[] cloudX, float cloudY[]) {
//        return convexHull(cloudX, cloudY, cloudX.length);
//    }
//
//    public static Polygon convexHull(float[] cloudX, float cloudY[], int nVert) {
//        if (nVert < 3)
//            return null;
//        int[] edgeList = new int[nVert];
//        int numEdges = 0;
//
//        float minY = Float.MAX_VALUE;
//        int minYIndex = nVert;
//        for (int i = 0; i < nVert; ++i) {
//            if (cloudY[i] < minY) {
//                minY = cloudY[i];
//                minYIndex = i;
//            }
//        }
//
//        //ellipse(cloudX[minYIndex], minY, 4, 4);
//
//        int startIndex = minYIndex;
//        int winIndex = -1;
//        float dx = -1;
//        float dy = 0;
//        while (winIndex != minYIndex) {
//            float newdx = 0;
//            float newdy = 0;
//            float maxDot = -2;
//            for (int i = 0; i < nVert; ++i) {
//                if (i == startIndex)
//                    continue;
//                newdx = cloudX[i] - cloudX[startIndex];
//                newdy = cloudY[i] - cloudY[startIndex];
//                float nrm = (float) Math.sqrt(newdx * newdx + newdy * newdy);
//                nrm = (nrm == 0f) ? 1 : nrm;
//                newdx /= nrm;
//                newdy /= nrm;
//                float newCross = newdx * dy - newdy * dx;
//                float newDot = newdx * dx + newdy * dy;
//                if (newCross >= 0 && newDot > maxDot) {
//                    maxDot = newDot;
//                    winIndex = i;
//                }
//            }
//            edgeList[numEdges++] = winIndex;
//            dx = cloudX[winIndex] - cloudX[startIndex];
//            dy = cloudY[winIndex] - cloudY[startIndex];
//            float nrm = (float) Math.sqrt(dx * dx + dy * dy);
//            nrm = (nrm == 0f) ? 1 : nrm;
//            dx /= nrm;
//            dy /= nrm;
//            startIndex = winIndex;
//        }
//
//        float[] xres = new float[numEdges];
//        float[] yres = new float[numEdges];
//        for (int i = 0; i < numEdges; i++) {
//            xres[i] = cloudX[edgeList[i]];
//            yres[i] = cloudY[edgeList[i]];
//        }
//
//        return new Polygon(xres, yres);
//
//    }

}