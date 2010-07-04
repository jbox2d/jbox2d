package org.jbox2d.p5;

//import java.awt.geom.AffineTransform;
//import java.awt.geom.GeneralPath;
//import java.awt.geom.PathIterator;
//import java.util.ArrayList;
//import java.util.LinkedHashSet;
//
//import org.jbox2d.collision.PolygonDef;
//import org.jbox2d.collision.ShapeDef;
//import org.jbox2d.common.Vec2;
//import org.jbox2d.dynamics.Body;
//import org.jbox2d.dynamics.BodyDef;
//import org.jbox2d.util.nonconvex.Polygon;

//Inactive in Processing JBox2d handler for now...

/**
 * A general class for holding static methods that do things with or to
 * polygons. Things like convex decomposition, polygon merging, &etc...
 * 
 * @author Greg
 * 
 */
public class PolygonUtils
{
//
//	/**
//	 * Creates a GeneralPath from two arrays of points (one array for x's, one
//	 * array for y's).
//	 * 
//	 * @param x1Points
//	 * @param y1Points
//	 * @return
//	 */
//	public static GeneralPath createGeneralPath(float[] x1Points, float[] y1Points)
//	{
//		GeneralPath polygon = new GeneralPath(GeneralPath.WIND_NON_ZERO, x1Points.length);
//		polygon.moveTo(x1Points[0], y1Points[0]);
//
//		for (int index = 1; index < x1Points.length; index++)
//		{
//			polygon.lineTo(x1Points[index], y1Points[index]);
//		}
//
//		polygon.closePath();
//
//		return polygon;
//	}
//
//	/**
//	 * The most fine-grained method for convex decomposition: turns a given
//	 * java.awt.Shape object into an array of JBox2D ShapeDefs. If your Shape
//	 * contains curved regions, then the fineness parameter will determine the
//	 * maximum deviation from the curve when the Shape is decomposed into line
//	 * segments.
//	 * 
//	 * @param physics
//	 * @param s a java.awt.Shape
//	 * @param fineness the maximum allowed deviation of the polygon shapes from the potentially curved java.awt.Shape object.
//	 * @return
//	 */
//	public static ShapeDef[] areaToShapes(PhysicsLayer physics, java.awt.Shape s, double fineness)
//	{
//		/*
//		 * Use the two-argument call to getPathIterator, which returns an iterator over the shape but
//		 * decomposed into line segments.
//		 */
//		PathIterator it = s.getPathIterator(AffineTransform.getTranslateInstance(0, 0), fineness);
//		/*
//		 * Create a 6-element buffer array to hold the coordinates as they come in.
//		 * NOTE: we will only need the first two elements, since the above call creates an iterator
//		 * that doesn't return curves.
//		 */
//		float[] coords = new float[6];
//		ArrayList<Vec2> vertices = new ArrayList<Vec2>();
//		LinkedHashSet<String> set = new LinkedHashSet<String>();
//		while (!it.isDone())
//		{
//			int type = it.currentSegment(coords);
//			Vec2 v = new Vec2(coords[0], coords[1]);
//			if (!set.contains(v.toString()))
//			{
//				vertices.add(v);
//				set.add(v.toString());
//			}
//			it.next();
//		}
//		Vec2[] vertArray = vertices.toArray(new Vec2[] {});
//
//		/*
//		 * Note that this is org.jbox2d.util.nonconvex.Polygon, NOT java.awt.Polygon
//		 */
//		Polygon poly = new Polygon(vertArray);
//		Polygon[] polygons = Polygon.decomposeConvex(poly);
//		ArrayList<ShapeDef> shapes = new ArrayList<ShapeDef>();
//		for (Polygon p : polygons)
//		{
//			PolygonDef def = new PolygonDef();
//			p.addTo(def);
//			shapes.add(def);
//		}
//		return shapes.toArray(new ShapeDef[] {});
//	}
//
//	/**
//	 * Simply calls the areaToShapes method and adds all Shapes from the returned ShapeDef array to a new Body.
//	 *  
//	 * @param physics
//	 * @param s
//	 * @param fineness
//	 * @return a new Body containing all the shapes in the decomposed polygon.
//	 */
//	public static Body areaToBody(PhysicsLayer physics, java.awt.Shape s, double fineness)
//	{
//		/*
//		 * Create the body to hold our simple polygons.
//		 */
//		BodyDef bd = new BodyDef();
//		Body b = physics.getWorld().createBody(bd);
//
//		ShapeDef[] shapes = areaToShapes(physics, s, fineness);
//
//		for (ShapeDef sd : shapes)
//		{
//			b.createShape(sd);
//		}
//		return b;
//	}
}
