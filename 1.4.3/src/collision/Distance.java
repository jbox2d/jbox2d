//Rev 55 of b2Distance.cpp

///*
//* Copyright (c) 2007 Erin Catto http://www.gphysics.com
//*
//* This software is provided 'as-is', without any express or implied
//* warranty.  In no event will the authors be held liable for any damages
//* arising from the use of this software.
//* Permission is granted to anyone to use this software for any purpose,
//* including commercial applications, and to alter it and redistribute it
//* freely, subject to the following restrictions:
//* 1. The origin of this software must not be misrepresented; you must not
//* claim that you wrote the original software. If you use this software
//* in a product, an acknowledgment in the product documentation would be
//* appreciated but is not required.
//* 2. Altered source versions must be plainly marked as such, and must not be
//* misrepresented as being the original software.
//* 3. This notice may not be removed or altered from any source distribution.
//*/
//
//#include "b2Collision.h"
//#include "b2Shape.h"
//
//int32 g_GJK_Iterations = 0;
//
//// GJK using Voronoi regions (Christer Ericson) and region selection
//// optimizations (Casey Muratori).
//
//// The origin is either in the region of points[1] or in the edge region. The origin is
//// not in region of points[0] because that is the old point.
//static int32 ProcessTwo(b2Vec2* p1Out, b2Vec2* p2Out, b2Vec2* p1s, b2Vec2* p2s, b2Vec2* points)
//{
//    // If in point[1] region
//    b2Vec2 r = -points[1];
//    b2Vec2 d = points[0] - points[1];
//    float32 length = d.Normalize();
//    float32 lambda = b2Dot(r, d);
//    if (lambda <= 0.0f || length < FLT_EPSILON)
//    {
//        // The simplex is reduced to a point.
//        *p1Out = p1s[1];
//        *p2Out = p2s[1];
//        p1s[0] = p1s[1];
//        p2s[0] = p2s[1];
//        points[0] = points[1];
//        return 1;
//    }
//
//    // Else in edge region
//    lambda /= length;
//    *p1Out = p1s[1] + lambda * (p1s[0] - p1s[1]);
//    *p2Out = p2s[1] + lambda * (p2s[0] - p2s[1]);
//    return 2;
//}
//
//// Possible regions:
//// - points[2]
//// - edge points[0]-points[2]
//// - edge points[1]-points[2]
//// - inside the triangle
//static int32 ProcessThree(b2Vec2* p1Out, b2Vec2* p2Out, b2Vec2* p1s, b2Vec2* p2s, b2Vec2* points)
//{
//    b2Vec2 a = points[0];
//    b2Vec2 b = points[1];
//    b2Vec2 c = points[2];
//
//    b2Vec2 ab = b - a;
//    b2Vec2 ac = c - a;
//    b2Vec2 bc = c - b;
//
//    float32 sn = -b2Dot(a, ab), sd = b2Dot(b, ab);
//    float32 tn = -b2Dot(a, ac), td = b2Dot(c, ac);
//    float32 un = -b2Dot(b, bc), ud = b2Dot(c, bc);
//
//    // In vertex c region?
//    if (td <= 0.0f && ud <= 0.0f)
//    {
//        // Single point
//        *p1Out = p1s[2];
//        *p2Out = p2s[2];
//        p1s[0] = p1s[2];
//        p2s[0] = p2s[2];
//        points[0] = points[2];
//        return 1;
//    }
//
//    // Should not be in vertex a or b region.
//    NOT_USED(sd);
//    NOT_USED(sn);
//    b2Assert(sn > 0.0f || tn > 0.0f);
//    b2Assert(sd > 0.0f || un > 0.0f);
//
//    float32 n = b2Cross(ab, ac);
//
//    // Should not be in edge ab region.
//    float32 vc = n * b2Cross(a, b);
//    b2Assert(vc > 0.0f || sn > 0.0f || sd > 0.0f);
//
//    // In edge bc region?
//    float32 va = n * b2Cross(b, c);
//    if (va <= 0.0f && un >= 0.0f && ud >= 0.0f)
//    {
//        b2Assert(un + ud > 0.0f);
//        float32 lambda = un / (un + ud);
//        *p1Out = p1s[1] + lambda * (p1s[2] - p1s[1]);
//        *p2Out = p2s[1] + lambda * (p2s[2] - p2s[1]);
//        p1s[0] = p1s[2];
//        p2s[0] = p2s[2];
//        points[0] = points[2];
//        return 2;
//    }
//
//    // In edge ac region?
//    float32 vb = n * b2Cross(c, a);
//    if (vb <= 0.0f && tn >= 0.0f && td >= 0.0f)
//    {
//        b2Assert(tn + td > 0.0f);
//        float32 lambda = tn / (tn + td);
//        *p1Out = p1s[0] + lambda * (p1s[2] - p1s[0]);
//        *p2Out = p2s[0] + lambda * (p2s[2] - p2s[0]);
//        p1s[1] = p1s[2];
//        p2s[1] = p2s[2];
//        points[1] = points[2];
//        return 2;
//    }
//
//    // Inside the triangle, compute barycentric coordinates
//    float32 denom = va + vb + vc;
//    b2Assert(denom > 0.0f);
//    denom = 1.0f / denom;
//    float32 u = va * denom;
//    float32 v = vb * denom;
//    float32 w = 1.0f - u - v;
//    *p1Out = u * p1s[0] + v * p1s[1] + w * p1s[2];
//    *p2Out = u * p2s[0] + v * p2s[1] + w * p2s[2];
//    return 3;
//}
//
//static bool InPoints(const b2Vec2& w, const b2Vec2* points, int32 pointCount)
//{
//    for (int32 i = 0; i < pointCount; ++i)
//    {
//        if (w == points[i])
//        {
//            return true;
//        }
//    }
//
//    return false;
//}
//
//float32 b2Distance(b2Vec2* p1Out, b2Vec2* p2Out, const b2Shape* shape1, const b2Shape* shape2)
//{
//    b2Vec2 p1s[3], p2s[3];
//    b2Vec2 points[3];
//    int32 pointCount = 0;
//
//    *p1Out = shape1->m_position;
//    *p2Out = shape2->m_position;
//
//    float32 vSqr = 0.0f;
//    const int32 maxIterations = 20;
//    for (int32 iter = 0; iter < maxIterations; ++iter)
//    {
//        b2Vec2 v = *p2Out - *p1Out;
//        b2Vec2 w1 = shape1->Support(v);
//        b2Vec2 w2 = shape2->Support(-v);
//
//        vSqr = b2Dot(v, v);
//        b2Vec2 w = w2 - w1;
//        float32 vw = b2Dot(v, w);
//        if (vSqr - vw <= 0.01f * vSqr || InPoints(w, points, pointCount)) // or w in points
//        {
//            if (pointCount == 0)
//            {
//                *p1Out = w1;
//                *p2Out = w2;
//            }
//            g_GJK_Iterations = iter;
//            return sqrtf(vSqr);
//        }
//
//        switch (pointCount)
//        {
//        case 0:
//            p1s[0] = w1;
//            p2s[0] = w2;
//            points[0] = w;
//            *p1Out = p1s[0];
//            *p2Out = p2s[0];
//            ++pointCount;
//            break;
//            
//        case 1:
//            p1s[1] = w1;
//            p2s[1] = w2;
//            points[1] = w;
//            pointCount = ProcessTwo(p1Out, p2Out, p1s, p2s, points);
//            break;
//
//        case 2:
//            p1s[2] = w1;
//            p2s[2] = w2;
//            points[2] = w;
//            pointCount = ProcessThree(p1Out, p2Out, p1s, p2s, points);
//            break;
//        }
//
//        // If we have three points, then the origin is in the corresponding triangle.
//        if (pointCount == 3)
//        {
//            g_GJK_Iterations = iter;
//            return 0.0f;
//        }
//
//        float32 maxSqr = -FLT_MAX;
//        for (int32 i = 0; i < pointCount; ++i)
//        {
//            maxSqr = b2Max(maxSqr, b2Dot(points[i], points[i]));
//        }
//
//        if (pointCount == 3 || vSqr <= 100.0f * FLT_EPSILON * maxSqr)
//        {
//            g_GJK_Iterations = iter;
//            return sqrtf(vSqr);
//        }
//    }
//
//    g_GJK_Iterations = maxIterations;
//    return sqrtf(vSqr);
//}
