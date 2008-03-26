/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/ 
 * Box2D homepage: http://www.box2d.org
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

package org.jbox2d.collision;

//Version: b2BroadPhase.h/.cpp rev 108

import java.util.Arrays;
import java.util.Collections;

import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;

/*
This broad phase uses the Sweep and Prune algorithm as described in:
Collision Detection in Interactive 3D Environments by Gino van den Bergen
Also, some ideas, such as using integral values for fast compares comes from
Bullet (http:/www.bulletphysics.com).
*/

// Notes:
// - we use bound arrays instead of linked lists for cache coherence.
// - we use quantized integral values for fast compares.
// - we use short indices rather than pointers to save memory.
// - we use a stabbing count for fast overlap queries (less than order N).
// - we also use a time stamp on each proxy to speed up the registration of
// overlap query results.
// - where possible, we compare bound indices instead of values to reduce
// cache misses (TODO_ERIN).
// - no broadphase is perfect and neither is this one: it is not great for
// huge worlds (use a multi-SAP instead), it is not great for large objects.

class BoundValues {
    public int[] lowerValues;
    public int[] upperValues;
    
    public BoundValues() {
        lowerValues = new int[2];
        upperValues = new int[2];
    }
}

public class BroadPhase {
    public static final int INVALID = Integer.MAX_VALUE;

    public static final int NULL_EDGE = Integer.MAX_VALUE;

    public PairManager m_pairManager;

    public Proxy m_proxyPool[];

    int m_freeProxy;

    BufferedPair pairBuffer[];

    int m_pairBufferCount;

    public Bound m_bounds[][];

    //PairCallback pairCallback;

    int m_queryResults[];

    int m_queryResultCount;

    public AABB m_worldAABB;

    public Vec2 m_quantizationFactor;

    public int m_proxyCount;

    int m_timeStamp;

    private static final boolean debugPrint = false;

    public static final boolean s_validate = false;

    // Dumps m_bounds array to console for debugging
    private void dump() {
        for (int i = 0; i < 10; i++) {
            System.out.printf("bounds[ %d ] = %d, %d \n", i,
                    m_bounds[0][i].value, m_bounds[1][i].value);
        }
    }

    public BroadPhase(AABB worldAABB, PairCallback callback) {
        if (debugPrint) {
            System.out.println("BroadPhase()");
        }
        
        // array initialization
        m_proxyPool = new Proxy[Settings.maxProxies];
        pairBuffer = new BufferedPair[Settings.maxPairs];
        m_bounds = new Bound[2][2 * Settings.maxProxies];
        m_queryResults = new int[Settings.maxProxies];

        for (int i = 0; i < 2 * Settings.maxProxies; i++) {
            m_bounds[0][i] = new Bound();
            m_bounds[1][i] = new Bound();
        }

        for (int i = 0; i < Settings.maxProxies; i++) {
            pairBuffer[i] = new BufferedPair();
        }

        m_pairManager = new PairManager();
        m_pairManager.initialize(this, callback);
        
        assert worldAABB.isValid();

        m_worldAABB = new AABB(worldAABB);
        m_proxyCount = 0;

        Vec2 d = worldAABB.upperBound.sub(worldAABB.lowerBound);
        m_quantizationFactor = new Vec2(Integer.MAX_VALUE / d.x,
                Integer.MAX_VALUE / d.y);

        for (int i = 0; i < Settings.maxProxies - 1; ++i) {
            m_proxyPool[i] = new Proxy();
            m_proxyPool[i].setNext(i + 1);
            m_proxyPool[i].timeStamp = 0;
            m_proxyPool[i].overlapCount = INVALID;
            m_proxyPool[i].userData = null;
        }

        m_proxyPool[Settings.maxProxies - 1] = new Proxy();
        m_proxyPool[Settings.maxProxies - 1].setNext(PairManager.NULL_PROXY);
        m_proxyPool[Settings.maxProxies - 1].timeStamp = 0;
        m_proxyPool[Settings.maxProxies - 1].overlapCount = INVALID;
        m_proxyPool[Settings.maxProxies - 1].userData = null;
        m_freeProxy = 0;

        m_timeStamp = 1;
        m_queryResultCount = 0;
    }
    
 // This one is only used for validation.
    protected boolean testOverlap(Proxy p1, Proxy p2) {
        for (int axis = 0; axis < 2; ++axis)
        {
            Bound[] bounds = m_bounds[axis];

            assert(p1.lowerBounds[axis] < 2 * m_proxyCount);
            assert(p1.upperBounds[axis] < 2 * m_proxyCount);
            assert(p2.lowerBounds[axis] < 2 * m_proxyCount);
            assert(p2.upperBounds[axis] < 2 * m_proxyCount);

            if (bounds[p1.lowerBounds[axis]].value > bounds[p2.upperBounds[axis]].value)
                return false;

            if (bounds[p1.upperBounds[axis]].value < bounds[p2.lowerBounds[axis]].value)
                return false;
        }

        return true;
    }

    private boolean testOverlap(BoundValues b, Proxy p) {
        for (int axis = 0; axis < 2; ++axis)
        {
            Bound[] bounds = m_bounds[axis];

            assert(p.lowerBounds[axis] < 2 * m_proxyCount);
            assert(p.upperBounds[axis] < 2 * m_proxyCount);

            if (b.lowerValues[axis] > bounds[p.upperBounds[axis]].value)
                return false;

            if (b.upperValues[axis] < bounds[p.lowerBounds[axis]].value)
                return false;
        }

        return true;
    }

//    boolean shouldCollide(int id1, int id2) {
//        assert (id1 < Settings.maxProxies);
//        assert (id2 < Settings.maxProxies);
//        Proxy p1 = m_proxyPool[id1];
//        Proxy p2 = m_proxyPool[id2];
//
//        if (p1.groupIndex == p2.groupIndex && p1.groupIndex != 0) {
//            return p1.groupIndex > 0;
//        }
//
//        return (p1.maskBits & p2.categoryBits) != 0
//                && (p1.categoryBits & p2.maskBits) != 0;
//    }

    public Proxy getProxy(int proxyId) {
        if (proxyId == PairManager.NULL_PROXY
                || (m_proxyPool[proxyId].isValid() == false)) {
            return null;
        }
        else {
            return m_proxyPool[proxyId];
        }
    }

    // Create and destroy proxies. These call Flush first.
    int createProxy(AABB aabb, //int groupIndex, int categoryBits, int maskBits,
            Object userData) {
        if (debugPrint) {
            System.out.println("CreateProxy()");
        }
        
        assert(m_proxyCount < Settings.maxProxies);
        assert(m_freeProxy != PairManager.NULL_PROXY);
        
        int proxyId = m_freeProxy;
        Proxy proxy = m_proxyPool[proxyId];
        m_freeProxy = proxy.getNext();

        proxy.overlapCount = 0;
        proxy.userData = userData;
//        proxy.groupIndex = groupIndex;
//        proxy.categoryBits = categoryBits;
//        proxy.maskBits = maskBits;
        
        //assert m_proxyCount < Settings.maxProxies;

        int boundCount = 2 * m_proxyCount;

        int lowerValues[] = new int[2];
        int upperValues[] = new int[2];
        computeBounds(lowerValues, upperValues, aabb);

        for (int axis = 0; axis < 2; ++axis) {
            Bound[] bounds = m_bounds[axis];
            int[] indexes = new int[2];

            query(indexes, lowerValues[axis], upperValues[axis], bounds,
                    boundCount, axis);
            int lowerIndex = indexes[0];
            int upperIndex = indexes[1];

            // System.out.println(edgeCount + ", "+lowerValues[axis] + ",
            // "+upperValues[axis]);
            // memmove(bounds[upperIndex + 2], bounds[upperIndex],
            // (edgeCount - upperIndex) * sizeof(b2Bound));

            System.arraycopy(m_bounds[axis], upperIndex, m_bounds[axis],
                    upperIndex + 2, boundCount - upperIndex);
            for (int i = 0; i < boundCount - upperIndex; i++) {
                m_bounds[axis][upperIndex + 2 + i] = new Bound(
                        m_bounds[axis][upperIndex + 2 + i]);
            }

            // memmove(bounds[lowerIndex + 1], bounds[lowerIndex],
            // (upperIndex - lowerIndex) * sizeof(b2Bound));
            // System.out.println(lowerIndex+" "+upperIndex);
            System.arraycopy(m_bounds[axis], lowerIndex, m_bounds[axis],
                    lowerIndex + 1, upperIndex - lowerIndex);
            for (int i = 0; i < upperIndex - lowerIndex; i++) {
                m_bounds[axis][lowerIndex + 1 + i] = new Bound(
                        m_bounds[axis][lowerIndex + 1 + i]);
            }

            // The upper index has increased because of the lower bound
            // insertion.
            ++upperIndex;

            // Copy in the new bounds.

            // if (bounds[lowerIndex] == null)
            assert (bounds[lowerIndex] != null) : "Null pointer (lower)";
            // if (bounds[upperIndex] == null)
            assert (bounds[upperIndex] != null) : "Null pointer (upper)";

            bounds[lowerIndex].value = lowerValues[axis];
            bounds[lowerIndex].proxyId = proxyId;
            bounds[upperIndex].value = upperValues[axis];
            bounds[upperIndex].proxyId = proxyId;

            bounds[lowerIndex].stabbingCount = lowerIndex == 0 ? 0
                    : bounds[lowerIndex - 1].stabbingCount;
            bounds[upperIndex].stabbingCount = bounds[upperIndex - 1].stabbingCount;

            // System.out.printf("lv: %d , lid: %d, uv: %d, uid: %d
            // \n",lowerValues[axis],proxyId,upperValues[axis],proxyId);

            // Adjust the stabbing count between the new bounds.
            for (int index = lowerIndex; index < upperIndex; ++index) {
                ++bounds[index].stabbingCount;
            }

            // Adjust the all the affected bound indices.
            for (int index = lowerIndex; index < boundCount + 2; ++index) {
                Proxy proxyn = m_proxyPool[bounds[index].proxyId];
                if (bounds[index].isLower()) {
                    proxyn.lowerBounds[axis] = index;
                }
                else {
                    proxyn.upperBounds[axis] = index;
                }
            }
        }

        ++m_proxyCount;

        assert m_queryResultCount < Settings.maxProxies;
        // Create pairs if the AABB is in range.
        for (int i = 0; i < m_queryResultCount; ++i) {
            assert(m_queryResults[i] < Settings.maxProxies);
            assert(m_proxyPool[m_queryResults[i]].isValid());

            m_pairManager.addBufferedPair(proxyId, m_queryResults[i]);
        }

        m_pairManager.commit();

        if (s_validate)
        {
            validate();
        }

        // Prepare for next query.
        m_queryResultCount = 0;
        incrementTimeStamp();

        return proxyId;
    }

    public void destroyProxy(int proxyId) {
        assert(0 < m_proxyCount && m_proxyCount <= Settings.maxProxies);
        Proxy proxy = m_proxyPool[proxyId];
        assert(proxy.isValid());

        int boundCount = 2 * m_proxyCount;

        for (int axis = 0; axis < 2; ++axis) {
            Bound[] bounds = m_bounds[axis];

            int lowerIndex = proxy.lowerBounds[axis];
            int upperIndex = proxy.upperBounds[axis];
            int lowerValue = bounds[lowerIndex].value;
            int upperValue = bounds[upperIndex].value;

            // memmove(bounds + lowerIndex, bounds + lowerIndex + 1,
            // (upperIndex - lowerIndex - 1) * sizeof(b2Bound));
            // memmove(bounds[lowerIndex + 1], bounds[lowerIndex],
            // (upperIndex - lowerIndex) * sizeof(b2Bound));
            System.arraycopy(m_bounds[axis], lowerIndex + 1, m_bounds[axis],
                    lowerIndex, upperIndex - lowerIndex - 1);
            for (int i = 0; i < upperIndex - lowerIndex - 1; i++) {
                m_bounds[axis][lowerIndex + i] = new Bound(
                        m_bounds[axis][lowerIndex + i]);
            }
            // memmove(bounds + upperIndex-1, bounds + upperIndex + 1,
            // (edgeCount - upperIndex - 1) * sizeof(b2Bound));
            System.arraycopy(m_bounds[axis], upperIndex + 1, m_bounds[axis],
                    upperIndex - 1, boundCount - upperIndex - 1);
            for (int i = 0; i < boundCount - upperIndex - 1; i++) {
                m_bounds[axis][upperIndex - 1 + i] = new Bound(
                        m_bounds[axis][upperIndex - 1 + i]);
            }

            // Fix bound indices.
            for (int index = lowerIndex; index < boundCount - 2; ++index) {
                Proxy proxyn = m_proxyPool[bounds[index].proxyId];
                if (bounds[index].isLower()) {
                    proxyn.lowerBounds[axis] = index;
                }
                else {
                    proxyn.upperBounds[axis] = index;
                }
            }

            // Fix stabbing count.
            for (int index = lowerIndex; index < upperIndex - 1; ++index) {
                --bounds[index].stabbingCount;
            }

            // Query for pairs to be removed. lowerIndex and upperIndex are not
            // needed.
            int[] ignored = new int[2];
            query(ignored, lowerValue, upperValue, bounds, boundCount - 2, axis);
        }

        assert (m_queryResultCount < Settings.maxProxies);

        for (int i = 0; i < m_queryResultCount; ++i) {
            assert(m_proxyPool[m_queryResults[i]].isValid());
            m_pairManager.removeBufferedPair(proxyId, m_queryResults[i]);
        }
        
        m_pairManager.commit();

        // Prepare for next query.
        m_queryResultCount = 0;
        incrementTimeStamp();

        // Return the proxy to the pool.
        proxy.userData = null;
        proxy.overlapCount = BroadPhase.INVALID;
        proxy.lowerBounds[0] = BroadPhase.INVALID;
        proxy.lowerBounds[1] = BroadPhase.INVALID;
        proxy.upperBounds[0] = BroadPhase.INVALID;
        proxy.upperBounds[1] = BroadPhase.INVALID;


        // Return the proxy to the pool.
        proxy.setNext(m_freeProxy);
        m_freeProxy = proxyId;
        --m_proxyCount;

         if (s_validate) {
             validate();
         }
    }

    // Call MoveProxy as many times as you like, then when you are done
    // call Flush to finalized the proxy pairs (for your time step).
    void moveProxy(int proxyId, AABB aabb) {
        if (debugPrint) {
            System.out.println("MoveProxy()");
        }

        if (proxyId == PairManager.NULL_PROXY || Settings.maxProxies <= proxyId) {
            return;
        }

        assert (aabb.isValid()) : "invalid AABB";

        int boundCount = 2 * m_proxyCount;

        Proxy proxy = m_proxyPool[proxyId];
        
        //Get new bound values
        BoundValues newValues = new BoundValues();
        computeBounds(newValues.lowerValues, newValues.upperValues, aabb);
        
        //Get old bound values
        BoundValues oldValues = new BoundValues();
        for (int axis = 0; axis < 2; ++axis) {
            oldValues.lowerValues[axis] = m_bounds[axis][proxy.lowerBounds[axis]].value;
            oldValues.upperValues[axis] = m_bounds[axis][proxy.upperBounds[axis]].value;
        }

        for (int axis = 0; axis < 2; ++axis) {
            Bound[] bounds = m_bounds[axis];

            int lowerIndex = proxy.lowerBounds[axis];
            int upperIndex = proxy.upperBounds[axis];

            int lowerValue = newValues.lowerValues[axis];
            int upperValue = newValues.upperValues[axis];

            int deltaLower = lowerValue - bounds[lowerIndex].value;
            int deltaUpper = upperValue - bounds[upperIndex].value;

            bounds[lowerIndex].value = lowerValue;
            bounds[upperIndex].value = upperValue;

            //
            // Expanding adds overlaps
            //

            // Should we move the lower bound down?
            if (deltaLower < 0) {
                int index = lowerIndex;
                while (index > 0 && lowerValue < bounds[index - 1].value) {
                    Bound bound = bounds[index];
                    Bound prevBound = bounds[index - 1];

                    int prevProxyId = prevBound.proxyId;
                    Proxy prevProxy = m_proxyPool[prevBound.proxyId];

                    ++prevBound.stabbingCount;

                    if (prevBound.isUpper() == true) {
                        if (testOverlap(newValues, prevProxy)) {
                            m_pairManager.addBufferedPair(proxyId, prevProxyId);
                        }

                        ++prevProxy.upperBounds[axis];
                        ++bound.stabbingCount;
                    }
                    else {
                        ++prevProxy.lowerBounds[axis];
                        --bound.stabbingCount;
                    }

                    --proxy.lowerBounds[axis];

                    // b2Swap(*bound, *prevEdge);
                    Bound tmp = new Bound(bound);
                    bound.set(prevBound);
                    prevBound.set(tmp);
                    --index;
                }
            }

            // Should we move the upper bound up?
            if (deltaUpper > 0) {
                int index = upperIndex;
                while (index < boundCount - 1
                        && bounds[index + 1].value <= upperValue) {
                    Bound bound = bounds[index];
                    Bound nextBound = bounds[index + 1];
                    int nextProxyId = nextBound.proxyId;
                    Proxy nextProxy = m_proxyPool[nextProxyId];

                    ++nextBound.stabbingCount;

                    if (nextBound.isLower() == true) {
                        if (testOverlap(newValues, nextProxy)) {
                            m_pairManager.addBufferedPair(proxyId, nextProxyId);
                        }

                        --nextProxy.lowerBounds[axis];
                        ++bound.stabbingCount;
                    }
                    else {
                        --nextProxy.upperBounds[axis];
                        --bound.stabbingCount;
                    }

                    ++proxy.upperBounds[axis];
                    // b2Swap(*bound, *nextEdge);
                    // wasn't actually swapping! bounds[index] and
                    // bounds[index+1] need to be swapped by VALUE
                    Bound tmp = new Bound(bound);
                    bound.set(nextBound);
                    nextBound.set(tmp);
                    ++index;
                }
            }

            //
            // Shrinking removes overlaps
            //

            // Should we move the lower bound up?
            if (deltaLower > 0) {
                int index = lowerIndex;
                while (index < boundCount - 1
                        && bounds[index + 1].value <= lowerValue) {
                    Bound bound = bounds[index];
                    Bound nextBound = bounds[index + 1];

                    int nextProxyId = nextBound.proxyId;
                    Proxy nextProxy = m_proxyPool[nextProxyId];

                    --nextBound.stabbingCount;

                    if (nextBound.isUpper()) {
                        if (testOverlap(oldValues,nextProxy)) {
                            m_pairManager.removeBufferedPair(proxyId, nextProxyId);
                        }

                        --nextProxy.upperBounds[axis];
                        --bound.stabbingCount;
                    }
                    else {
                        --nextProxy.lowerBounds[axis];
                        ++bound.stabbingCount;
                    }

                    ++proxy.lowerBounds[axis];
                    // b2Swap(*bound, *nextEdge);
                    // Bound tmp = bound;
                    // bound = nextEdge;
                    // nextEdge = tmp;
                    Bound tmp = new Bound(bound);
                    bound.set(nextBound);
                    nextBound.set(tmp);
                    ++index;
                }
            }

            // Should we move the upper bound down?
            if (deltaUpper < 0) {
                int index = upperIndex;
                while (index > 0 && upperValue < bounds[index - 1].value) {
                    Bound bound = bounds[index];
                    Bound prevBound = bounds[index - 1];

                    int prevProxyId = prevBound.proxyId;
                    Proxy prevProxy = m_proxyPool[prevProxyId];

                    --prevBound.stabbingCount;

                    if (prevBound.isLower() == true) {
                        if (testOverlap(oldValues, prevProxy)) {
                            m_pairManager.removeBufferedPair(proxyId, prevProxyId);
                        }

                        ++prevProxy.lowerBounds[axis];
                        --bound.stabbingCount;
                    }
                    else {
                        ++prevProxy.upperBounds[axis];
                        ++bound.stabbingCount;
                    }

                    --proxy.upperBounds[axis];
                    // b2Swap(*bound, *prevEdge);
                    // Bound tmp = bound;
                    // bound = prevEdge;
                    // prevEdge = tmp;
                    Bound tmp = new Bound(bound);
                    bound.set(prevBound);
                    prevBound.set(tmp);
                    --index;
                }
            }
        }

        if (s_validate) {
            validate();
        }
    }
    
    public void commit() {
        m_pairManager.commit();
    }



    // Query an AABB for overlapping proxies, returns the user data and
    // the count, up to the supplied maximum count.
    public Object[] query(AABB aabb, int maxCount) {
        if (debugPrint) {
            System.out.println("Query(2 args)");
        }

        int lowerValues[] = new int[2];
        int upperValues[] = new int[2];
        computeBounds(lowerValues, upperValues, aabb);

        int indexes[] = new int[2]; // lowerIndex, upperIndex;

        query(indexes, lowerValues[0], upperValues[0], m_bounds[0],
                2 * m_proxyCount, 0);
        query(indexes, lowerValues[1], upperValues[1], m_bounds[1],
                2 * m_proxyCount, 1);

        assert m_queryResultCount < Settings.maxProxies;

        Object[] results = new Object[maxCount];

        int count = 0;
        for (int i = 0; i < m_queryResultCount && count < maxCount; ++i, ++count) {
            assert m_queryResults[i] < Settings.maxProxies;
            Proxy proxy = m_proxyPool[m_queryResults[i]];
            proxy.isValid();
            results[i] = proxy.userData;
        }
        
        Object[] copy = new Object[count];
        System.arraycopy(results,0,copy,0,count);

        // Prepare for next query.
        m_queryResultCount = 0;
        incrementTimeStamp();

        return copy;//results;
    }

    public void validate() {
        if (debugPrint) {
            System.out.println("Validate()");
        }

        for (int axis = 0; axis < 2; ++axis) {
            Bound[] bounds = m_bounds[axis];

            int boundCount = 2 * m_proxyCount;
            int stabbingCount = 0;

            for (int i = 0; i < boundCount; ++i) {
                Bound bound = bounds[i];
                assert(i == 0 || bounds[i-1].value <= bound.value);
                assert(bound.proxyId != PairManager.NULL_PROXY);
                assert(m_proxyPool[bound.proxyId].isValid());


                if (bound.isLower() == true) {
                    assert (m_proxyPool[bound.proxyId].lowerBounds[axis] == i) : (m_proxyPool[bound.proxyId].lowerBounds[axis]
                            + " not " + i);
                    ++stabbingCount;
                }
                else {
                    assert (m_proxyPool[bound.proxyId].upperBounds[axis] == i);
                    --stabbingCount;
                }

                assert (bound.stabbingCount == stabbingCount);
            }
        }

    }


    private void computeBounds(int[] lowerValues, int[] upperValues, AABB aabb) {
        if (debugPrint) {
            System.out.println("ComputeBounds()");
        }
        assert(aabb.upperBound.x > aabb.lowerBound.x);
        assert(aabb.upperBound.y > aabb.lowerBound.y);
        
        Vec2 minVertex = MathUtils.clamp(aabb.lowerBound, m_worldAABB.lowerBound,
                m_worldAABB.upperBound);
        Vec2 maxVertex = MathUtils.clamp(aabb.upperBound, m_worldAABB.lowerBound,
                m_worldAABB.upperBound);

        // System.out.printf("minV = %f %f, maxV = %f %f
        // \n",aabb.minVertex.x,aabb.minVertex.y,aabb.maxVertex.x,aabb.maxVertex.y);

        // Bump lower bounds downs and upper bounds up. This ensures correct
        // sorting of
        // lower/upper bounds that would have equal values.
        // TODO_ERIN implement fast float to int conversion.
        lowerValues[0] = (int) (m_quantizationFactor.x * (minVertex.x - m_worldAABB.lowerBound.x))
                & (Integer.MAX_VALUE - 1);
        upperValues[0] = (int) (m_quantizationFactor.x * (maxVertex.x - m_worldAABB.lowerBound.x)) | 1;

        lowerValues[1] = (int) (m_quantizationFactor.y * (minVertex.y - m_worldAABB.lowerBound.y))
                & (Integer.MAX_VALUE - 1);
        upperValues[1] = (int) (m_quantizationFactor.y * (maxVertex.y - m_worldAABB.lowerBound.y)) | 1;
    }

 

    /**
     * @param results
     *            out variable
     */
    private void query(int[] results, int lowerValue, int upperValue,
            Bound[] bounds, int boundCount, int axis) {
        if (debugPrint) {
            System.out.println("Query(6 args)");
        }

        int lowerQuery = binarySearch(bounds, boundCount, lowerValue);
        int upperQuery = binarySearch(bounds, boundCount, upperValue);

        // Easy case: lowerQuery <= lowerIndex(i) < upperQuery
        // Solution: search query range for min bounds.
        for (int i = lowerQuery; i < upperQuery; ++i) {
            if (bounds[i].isLower()) {
                incrementOverlapCount(bounds[i].proxyId);
            }
        }
        // Hard case: lowerIndex(i) < lowerQuery < upperIndex(i)
        // Solution: use the stabbing count to search down the bound array.
        if (lowerQuery > 0) {
            int i = lowerQuery - 1;
            int s = bounds[i].stabbingCount;
            // Find the s overlaps.
            while (s != 0) {
                assert (i >= 0) : ("i = " + i + "; s = " + s);
                if (bounds[i].isLower()) {
                    Proxy proxy = m_proxyPool[bounds[i].proxyId];
                    if (lowerQuery <= proxy.upperBounds[axis]) {
                        incrementOverlapCount(bounds[i].proxyId);
                        --s;
                    }
                }
                --i;
            }
        }

        results[0] = lowerQuery;
        results[1] = upperQuery;
    }

    private void incrementOverlapCount(int proxyId) {
        if (debugPrint) {
            System.out.println("IncrementOverlapCount()");
        }

        Proxy proxy = m_proxyPool[proxyId];
        if (proxy.timeStamp < m_timeStamp) {
            proxy.timeStamp = m_timeStamp;
            proxy.overlapCount = 1;
        }
        else {
            proxy.overlapCount = 2;
            assert m_queryResultCount < Settings.maxProxies;
            m_queryResults[m_queryResultCount] = proxyId;
            ++m_queryResultCount;
        }
    }

    private void incrementTimeStamp() {
        if (debugPrint) {
            System.out.println("IncrementTimeStamp()");
        }

        if (m_timeStamp == Integer.MAX_VALUE) {
            for (int i = 0; i < Settings.maxProxies; ++i) {
                m_proxyPool[i].timeStamp = 0;
            }
            m_timeStamp = 1;
        }
        else {
            ++m_timeStamp;
        }
    }

    static int binarySearch(Bound[] bounds, int count, int value) {
        if (debugPrint) {
            System.out.println("BinarySearch()");
        }

        int low = 0;
        int high = count - 1;
        while (low <= high) {
            int mid = (low + high) >> 1;
            if (bounds[mid].value > value) {
                high = mid - 1;
            }
            else if (bounds[mid].value < value) {
                low = mid + 1;
            }
            else {
                return mid;
            }
        }

        return low;

    }

    public boolean inRange(AABB aabb) {
        Vec2 d = Vec2.max(aabb.lowerBound.sub(m_worldAABB.upperBound),
                m_worldAABB.lowerBound.sub(aabb.upperBound));
        return (Math.max(d.x, d.y) < 0.0f);
    }
}
