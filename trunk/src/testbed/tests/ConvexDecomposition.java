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
package testbed.tests;

import java.util.Random;

import processing.core.PApplet;
import testbed.PTest;
import collision.BoxDef;
import collision.CircleShape;
import collision.PolyDef;
import collision.PolyShape;
import collision.Shape;
import collision.ShapeType;

import common.Vec2;

import dynamics.Body;
import dynamics.BodyDef;
import dynamics.World;

import util.nonconvex.*;

public class ConvexDecomposition extends PTest {

    int bodyIndex;

    Body bodies[];

    PolyDef polyprot; //PolyDef prototype
    
    Polygon pgon;

    World world;

    public ConvexDecomposition() {
        super("ConvexDecomposition");


    }

    @Override
    public void go(World world) {
        bodies = new Body[256];
        polyprot = new PolyDef();
        this.world = world;
        
        // Ground body
        {
            BoxDef sd = new BoxDef();
            sd.extents = new Vec2(50.0f, 10.0f);
            sd.friction = 0.3f;

            BodyDef bd = new BodyDef();
            bd.position = new Vec2(0.0f, -10.0f);
            bd.addShape(sd);
            world.createBody(bd);
        }

        //Create prototype polydef
        polyprot = new PolyDef();
        polyprot.density = 1.0f;
        polyprot.friction = 0.3f;
        polyprot.restitution = 0.2f;
        
        //Create non-convex polygon
        float[] x = { 0f, 10f, 10f, 2f, 2f, 7f, 7f, 2f, 2f, 10f, 10f, 0f };
        float[] y = { 0f, 0f,  2f,  2f, 5f, 5f, 7f, 7f, 10f, 10f, 12f, 12f };
        for (int i=0; i<x.length; i++){
            x[i] -= 5f;
            y[i] -= 6f;
            x[i] += random(-.01f, .01f);
            y[i] += random(-.01f, .01f);
            x[i] *= .3f;
            y[i] *= .3f;
        }
        pgon = new Polygon(x,y);
        
        bodyIndex = 0;
    }
    
    void CreateBody() {
        if (bodies[bodyIndex] != null) {
            world.destroyBody(bodies[bodyIndex]);
            bodies[bodyIndex] = null;
        }

        Random r = new Random();

        BodyDef bd = new BodyDef();
        Polygon.decomposeConvexAndAddTo(pgon, bd, polyprot);
        float x = r.nextFloat() * 4 - 2;
        bd.position = new Vec2(x, 20.0f);
        bd.rotation = 0f;//(float) (r.nextFloat() * Math.PI * 4 - Math.PI * 2);

        bodies[bodyIndex] = world.createBody(bd);
        bodyIndex = (bodyIndex + 1) % bodies.length;
    }

    @Override
    protected void checkKeys() {
        if (newKeyDown['1']) {
            CreateBody();
        }
    }
    
    @Override
    public void DrawShape(Shape shape, int c) {
        noStroke();
        fill(c);
        if (shape.m_type == ShapeType.POLY_SHAPE
                || shape.m_type == ShapeType.BOX_SHAPE) {
            PolyShape poly = (PolyShape) shape;

            beginShape(POLYGON);
            for (int i = 0; i < poly.m_vertexCount; ++i) {
                Vec2 v = poly.m_R.mul(poly.m_vertices[i]).addLocal(
                        poly.m_position);
                vertex(v.x, v.y);
            }
            Vec2 v = poly.m_R.mul(poly.m_vertices[0]).addLocal(poly.m_position);
            vertex(v.x, v.y);
            endShape();
        }
        else if (shape.m_type == ShapeType.CIRCLE_SHAPE) {
            CircleShape circle = (CircleShape) shape;
            ellipse(circle.m_position.x, circle.m_position.y,
                    circle.m_radius * 2, circle.m_radius * 2);
            float xR = circle.m_radius
                    * (float) Math.cos(shape.m_body.m_rotation);
            float yR = circle.m_radius
                    * (float) Math.sin(shape.m_body.m_rotation);
            line(circle.m_position.x, circle.m_position.y, circle.m_position.x
                    + xR, circle.m_position.y + yR);
        }
    }

    public static void main(String[] args) {
        PApplet.main(new String[] { "testbed.tests.ConvexDecomposition" });
    }
}