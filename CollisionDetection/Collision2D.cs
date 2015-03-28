using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Physics2D.Util;
using Physics2D.ObjectShape;
using Physics2D.PhysicalObject;
using Physics2D.Constraint;

namespace Physics2D.CollisionDetection
{
    /** Collision2D does narrow phase collision detection. It loops through a list
 * of pairs of objects that the broad phase collision detector decided could
 * possibly be colliding.
 */
public class Collision2D
    {
    // If points are within this tolerance, categorize them as colliding
    private const double tolerance = 1.5;
    private const double parallelTolerance = 0.001;
    private PhysicsState physicsState;
    private ConstraintEngine constraintEngine;
    private Bag collidingList;
        
    // Constants for return value of collision tests 
    private const int FOUND_FEATURES = 1; // Closest features found, but no collision
    private const int ADDED_RESPONSE = 2; // Collision found and handled
    private const int PENETRATION = 3; // Objects are interpenetrating
        
    private const double ZERO_VEL = 0;
        
    public Collision2D()
        {
        physicsState = PhysicsState.getInstance();
        constraintEngine = ConstraintEngine.getInstance();
        collidingList = new Bag();
        }
        
    /** Loop through the ActiveList and perform exact collision detection on the
     * object pairs. If collisions are found, add collision responses.
     */ 
    public Bag testCollisions(HashSet<CollisionPair> activeList)
        {
        collidingList.clear();
                
        // Test the active objects for collisions
        foreach (CollisionPair pair in activeList)
            {
            if (!pair.noCollision && !constraintEngine.testNoCollisions(pair.c1, pair.c2))
                testNarrowPhase(pair);  
            }
                
        return collidingList;
        }
        
    ////////////////////////////////////////////////////
    // NARROW PHASE TESTING
    ////////////////////////////////////////////////////
    private void testNarrowPhase(CollisionPair pair)
        {
        // do the correct test based on the shapes of the objects
        Shape s1 = pair.c1.getShape();
        Shape s2 = pair.c2.getShape();

        if (s1 is Circle && s2 is Circle)
            testNarrowPhaseCircleCircle(pair);
        else if (s1 is Polygon && s2 is Polygon)
            testNarrowPhasePolyPoly(pair);
        else if (s1 is Polygon && s2 is Circle
            || s1 is Circle && s2 is Polygon)
            testNarrowPhasePolyCircle(pair);
        else
            throw new Exception("Unknown Shape!");
        }

    // Test two circles for collision.
    private bool testNarrowPhaseCircleCircle(CollisionPair pair)
        {
        Double2D ray = pair.c1.getPosition().subtract(pair.c2.getPosition()); 
        double dist = ray.length();
        double radius1 = ((Circle)pair.c1.getShape()).getRadius();
        double radius2 = ((Circle)pair.c2.getShape()).getRadius();
        
        if (dist < (radius1 + radius2 + tolerance))
            {
            // normal points from 2 to one
            pair.normal = ray.normalize();
            pair.relVel = pair.c1.getVelocity().subtract(pair.c2.getVelocity()).dot(pair.normal);
                        
            pair.colPoint2 = pair.normal.multiply(radius2);
            Double2D globalPoint = pair.colPoint2.add(pair.c2.getPosition());
            pair.colPoint1 = globalPoint.subtract(pair.c1.getPosition());
                        
            if (pair.relVel <= ZERO_VEL) // make sure objects aren't separating
                collidingList.add(pair);
            return true;
            }
        else
            return false;
        }

    // Test two polygons for collision. If they are interpenetrating, search back in
    // time (over the last timestep) to find where they collided.
    private bool testNarrowPhasePolyPoly(CollisionPair pair)
        {
        int result = testPolyPoly(pair, false); 
        if (result == PENETRATION)
            {
            // Need to do a binary search back in time to find the collision point
            double lowerBound = 0;
            double upperBound = 1;
                        
            // This stops after 6 tries (((((1/2)/2)/2)/2)/2 = 0.03125)
            while (result != ADDED_RESPONSE && upperBound - lowerBound >= .03125)
                {
                double currentPercent = lowerBound + (upperBound - lowerBound) / 2;
                                
                pair.c1.resetLastPose();
                pair.c1.updatePose(currentPercent);
                pair.c2.resetLastPose();
                pair.c2.updatePose(currentPercent);
                                
                // See if they are colliding
                result = testPolyPoly(pair, true);
                                
                // Reset the bounds based on the result
                if (result == PENETRATION)
                    upperBound = currentPercent; // move away
                else if (result == FOUND_FEATURES)
                    lowerBound = currentPercent; // move closer
                }
                        
            // restore the previous positions of objects
            pair.c1.restorePose();
            pair.c2.restorePose();
                        
            if (result == ADDED_RESPONSE)
                return true;
            else
                {
                // As a last resort, treat the polygon as a circle since we don't
                // want things passing through walls if we can avoid it
                if (pair.c1 is StationaryObject2D)
                    {
                    Polygon sav = (Polygon)pair.c2.getShape();
                    Circle circ = new Circle(Math.Max(sav.getMaxXDistanceFromCenter(), sav.getMaxYDistanceFromCenter()));
                    ((MobileObject2D)pair.c2).setShape(circ, ((MobileObject2D)pair.c2).getMass());
                                        
                    result = testPolyCircle(pair, true);
                                        
                    // Put the rectangle back
                    ((MobileObject2D)pair.c2).setShape(sav, ((MobileObject2D)pair.c2).getMass());
                                        
                    if (result != PENETRATION)
                        return true;
                    }
                else if (pair.c2 is StationaryObject2D)
                    {
                    Polygon sav = (Polygon)pair.c1.getShape();
                    Circle circ = new Circle(Math.Max(sav.getMaxXDistanceFromCenter(), sav.getMaxYDistanceFromCenter()));
                    ((MobileObject2D)pair.c1).setShape(circ, ((MobileObject2D)pair.c1).getMass());
                                        
                    result = testPolyCircle(pair, true);
                                        
                    // Put the rectangle back
                    ((MobileObject2D)pair.c1).setShape(sav, ((MobileObject2D)pair.c1).getMass());
                                        
                    if (result != PENETRATION)
                        return true;
                    }
                                
                // Don't check this pair again until they separate according to
                // the BroadPhase collision detector. At that point, this activePair
                // instance will be thrown away.
                pair.noCollision = true;
                return false;
                }
            }
        else
            return (result == ADDED_RESPONSE);
        }
        
    // Test a polygon and a circle for collision. If they are interpenetrating, search back in
    // time (over the last timestep) to find where they collided.
    private bool testNarrowPhasePolyCircle(CollisionPair pair)
        {
        int result = testPolyCircle(pair, false); 
        if (result == PENETRATION)
            {
            double lowerBound = 0;
            double upperBound = 1;
                        
            // This stops after 6 tries (((((1/2)/2)/2)/2)/2 = 0.03125)
            while (result != ADDED_RESPONSE && upperBound - lowerBound >= .03125)
                {
                double currentPercent = lowerBound + (upperBound - lowerBound) / 2;
                                
                // Set their pose to where they would have been at this time
                pair.c1.resetLastPose();
                pair.c1.updatePose(currentPercent);
                pair.c2.resetLastPose();
                pair.c2.updatePose(currentPercent);
                                
                // See if they are colliding
                result = testPolyCircle(pair, false);
                                
                // Reset the bounds based on the result
                if (result == PENETRATION)
                    upperBound = currentPercent; // move away
                else if (result == FOUND_FEATURES)
                    lowerBound = currentPercent; // move closer
                }
                        
            // restore the previous positions of objects
            pair.c1.restorePose();
            pair.c2.restorePose();
                        
            if (result == ADDED_RESPONSE)
                return true;
            else
                {
                // Don't check this pair again until they separate according to
                // the BroadPhase collision detector. At that point, this activePair
                // instance will be thrown away.
                pair.noCollision = true;
                return false;
                }
            }
        else
            return (result == ADDED_RESPONSE);
        }
        
    ///////////////////////////////////////////////////////////
    // Narrow phase collision detection for poly-poly and poly-circle.
    // These use Voronoi regions to determine the closest feature pair
    // between two objects and track that feature pair. This is very similar
    // to the Lin-Canny algorithm. See http://www.merl.com/reports/docs/TR97-23.pdf
    // for more information about Lin-Canny and other collision detection 
    // techniques
    ///////////////////////////////////////////////////////////
        
    // Tests to see if vertex2 falls into the Voronoi Region formed by 
    // rays 1 and 2 emanating from vertex1
    // PRECONDITION: leftRay and rightRay must be normalized
    private bool testVR(Double2D vertex1, Double2D leftRay, Double2D rightRay, Double2D vertex2, bool inclusive)
        {
        // Get a vector from vertex 1 to vertex 2
        Double2D connector = vertex2.subtract(vertex1);
                
        // project connector onto the ray
        double proj1 = leftRay.dot(connector);
        double proj2 = rightRay.dot(connector);
                
        if (inclusive && proj1 >= 0 && proj2 >= 0)
            return true;
        else if (!inclusive && proj1 > 0 && proj2 > 0)
            return true;
        else
            return false;
        }

    // Find and track the closest feature pair between two polygons
    // ActivePair stores the previous closest features (if any) for these
    // two polygons
        
    private int testPolyPoly(CollisionPair pair, bool searchingBack)
        {
        PhysicalObject2D collidePoly1 = pair.c1; 
        PhysicalObject2D collidePoly2 = pair.c2;
                
        Polygon shapePoly1 = (Polygon)collidePoly1.getShape();
        Polygon shapePoly2 = (Polygon)collidePoly2.getShape();

        // Get the vertices and edges of the polygons
        Double2D[] vertices1 = shapePoly1.getVertices();
        Double2D[] vertices2 = shapePoly2.getVertices();
                
        Double2D[] edges1 = shapePoly1.getEdges();
        Double2D[] edges2 = shapePoly2.getEdges();
                
        Double2D[] normals1 = shapePoly1.getNormals();
        Double2D[] normals2 = shapePoly2.getNormals();
                
        double dist = 0;
        bool foundFeatures = false;
        
        // Loop clockwise through the vertices and edges of both polygons to
        // test if they are the closest feature. Ideally, since things don't change
        // much between checks, the closest features are going to be the one that 
        // were closest last time, so start the search with them. Edges are indexed
        // by their left vertex (looking out from the center of the polygon)
        int curFeat1;
        int curFeat2;
        
        curFeat1 = pair.closestFeature1 != null ? pair.closestFeature1.Value : 0;
                
        // The vertices and edges of polygon 1
        for (int counter1 = 0; counter1 < vertices1.Length && !foundFeatures; counter1++)
            {
            curFeat2 = pair.closestFeature2 != null ? pair.closestFeature2.Value : 0;
                        
            // The vertices and edges of polygon 2
            for (int counter2 = 0; counter2 < vertices2.Length && !foundFeatures; counter2++)
                {
                int nextFeat1 = (curFeat1 + 1) % vertices1.Length;
                int nextFeat2 = (curFeat2 + 1) % vertices2.Length;
                                
                int prevFeat1 = curFeat1 == 0 ? vertices1.Length - 1 : curFeat1 - 1;
                int prevFeat2 = curFeat2 == 0 ? vertices1.Length - 1 : curFeat2 - 1;
                                
                // Now see if we can find two points that are in each other's Voronoi Regions
                // If we have that, then we have the nearest features of the two polygons
                                
                // EDGE vs. EDGE
                // first see if the edges are parallel and facing each other
                double dp = normals1[curFeat1].dot(normals2[curFeat2]);
                if (dp >= (-1 - parallelTolerance) && dp <= (-1 + parallelTolerance))
                    {
                    Double2D leftVertex = null; // looking from behind edge1
                    Double2D rightVertex = null;
                                                                                
                    // Find the left collision vertex
                    if (testVR(vertices1[curFeat1], normals1[curFeat1], edges1[curFeat1], vertices2[nextFeat2], true)
                        && testVR(vertices1[nextFeat1], edges1[curFeat1].multiply(-1), normals1[curFeat1], vertices2[nextFeat2], true))
                        {
                        leftVertex = vertices2[nextFeat2];
                        }
                    else if (testVR(vertices2[curFeat2], normals2[curFeat2], edges2[curFeat2], vertices1[curFeat1], true)
                        && testVR(vertices2[nextFeat2], edges2[curFeat2].multiply(-1), normals2[curFeat2], vertices1[curFeat1], true))
                        {
                        leftVertex = vertices1[curFeat1];
                        }
                                        
                    // If there is no left vertex there is no collision
                    if (leftVertex != null)
                        {
                        // Now find the right vertex
                        if (testVR(vertices2[curFeat2], normals2[curFeat2], edges2[curFeat2], vertices1[nextFeat1], true)
                            && testVR(vertices2[nextFeat2], edges2[curFeat2].multiply(-1), normals2[curFeat2], vertices1[nextFeat1], true))
                            {
                            rightVertex = vertices1[nextFeat1];
                            }
                        else if (testVR(vertices1[curFeat1], normals1[curFeat1], edges1[curFeat1], vertices2[curFeat2], true)
                            && testVR(vertices1[nextFeat1], edges1[curFeat1].multiply(-1), normals1[curFeat1], vertices2[curFeat2], true))
                            {
                            rightVertex = vertices2[curFeat2];
                            }
                        }
                                                
                    if (leftVertex != null && rightVertex != null)
                        {
                        pair.closestFeature1 = new int?(curFeat1);
                        pair.closestFeature2 = new int?(curFeat2);
                                                
                        foundFeatures = true;
                                                
                        // Normal needs to point from 2 to 1
                        pair.normal = normals2[curFeat2];
                                                
                        // Find the distance between the two
                        dist = vertices1[curFeat1].subtract(vertices2[curFeat2]).dot(pair.normal);
                                                
                        // Find the collision points
                        Double2D colPoint = rightVertex.add((leftVertex.subtract(rightVertex)).multiply(0.5));
                        pair.colPoint1 = colPoint.subtract(collidePoly1.getPosition());
                        pair.colPoint2 = colPoint.subtract(collidePoly2.getPosition());
                        }
                    }       
                                
                                
                if (!foundFeatures)
                    {
                    // VERTEX1 vs. VERTEX2
                    // The Voronoi region of a vertex falls between the normal to the edge
                    // on the left and the normal of the edge on the right
                    if (testVR(vertices1[curFeat1], normals1[prevFeat1], normals1[curFeat1], vertices2[curFeat2], false)
                        && testVR(vertices2[curFeat2], normals2[prevFeat2], normals2[curFeat2], vertices1[curFeat1], false))
                        {
                        // Found the closest features
                        foundFeatures = true;
                        pair.closestFeature1 = new int?(curFeat1);
                        pair.closestFeature2 = new int?(curFeat2);
                                                
                        dist = vertices1[curFeat1].subtract(vertices2[curFeat2]).length();
                        pair.colPoint1 = vertices1[curFeat1].subtract(collidePoly1.getPosition());
                        pair.colPoint2 = vertices1[curFeat1].subtract(collidePoly2.getPosition());
                        pair.normal = ((collidePoly1.getPosition()).subtract(collidePoly2.getPosition())).normalize();
                        }
                    }
                                
                // VERTEX1 vs. EDGE2
                // The Voronoi region of an edge is just its normal extending out from both 
                // vertices
                if (!foundFeatures)
                    {
                    // Find the point on edge2 that is closest to vertices1[curFeat1]
                    // by getting a vector from vertices2[curFeat2] to vertices1[curFeat1]
                    // and projecting it onto edge2
                    Double2D vecOther = vertices1[curFeat1].subtract(vertices2[curFeat2]);
                    double proj = vecOther.dot(edges2[curFeat2]);
                    Double2D edgePoint = vertices2[curFeat2].add(edges2[curFeat2].multiply(proj));
                                        
                    // See if this point lies in vertices1[curFeat1]'s VR
                    if (testVR(vertices1[curFeat1], normals1[prevFeat1], normals1[curFeat1], edgePoint, false))
                        {
                        // Now see if vertices1[curFeat1] lies in edge2's VR
                        if (testVR(vertices2[curFeat2], normals2[curFeat2], edges2[curFeat2], vertices1[curFeat1], true)
                            && testVR(vertices2[nextFeat2], edges2[curFeat2].multiply(-1), normals2[curFeat2], vertices1[curFeat1], true))
                            {
                            foundFeatures = true;
                            pair.closestFeature1 = new int?(curFeat1);
                            pair.closestFeature2 = new int?(curFeat2);
                                                        
                            dist = vertices1[curFeat1].subtract(edgePoint).length();
                            pair.colPoint1 = vertices1[curFeat1].subtract(collidePoly1.getPosition());
                            pair.colPoint2 = vertices1[curFeat1].subtract(collidePoly2.getPosition());
                                                        
                            pair.normal = normals2[curFeat2];
                            }
                        }       
                    }
                                
                // VERTEX2 vs. EDGE1
                if (!foundFeatures)
                    {
                    // try vertex2 and edges1[curFeat1] - get a vector from vertices1[curFeat1] to vertex2
                    // and project it onto edge1
                    Double2D vecOther = vertices2[curFeat2].subtract(vertices1[curFeat1]);
                    double proj = vecOther.dot(edges1[curFeat1]);
                    Double2D edgePoint = vertices1[curFeat1].add(edges1[curFeat1].multiply(proj));
                                        
                    // See if this point lies in vertex2's VR
                    if (testVR(vertices2[curFeat2], normals2[prevFeat2], normals2[curFeat2], edgePoint, false))
                        {
                        // Now see if vertex2 lies in edge1's VR
                        if (testVR(vertices1[curFeat1], normals1[curFeat1], edges1[curFeat1], vertices2[curFeat2], true)
                            && testVR(vertices1[nextFeat1], edges1[curFeat1].multiply(-1), normals1[curFeat1], vertices2[curFeat2], true))
                            {
                            foundFeatures = true;
                            pair.closestFeature1 = new int?(curFeat1);
                            pair.closestFeature2 = new int?(curFeat2);

                            dist = vertices2[curFeat2].subtract(edgePoint).length();
                            pair.colPoint1 = vertices2[curFeat2].subtract(collidePoly1.getPosition());
                            pair.colPoint2 = vertices2[curFeat2].subtract(collidePoly2.getPosition());
                                                        
                            // Normal needs to point from 2 to 1
                            pair.normal = normals1[curFeat1].multiply(-1);
                            }
                        }       
                    }
                                
                // Increment curFeat2, looping around the polygon
                curFeat2 = (curFeat2 + 1) % vertices2.Length;
                }
            curFeat1 = (curFeat1 + 1) % vertices1.Length;
            }
                
        // Add response if features are less than tolerance from each other
        if (foundFeatures && dist < tolerance)
            {       
            // Get the velocities of the collision points
            // vPoint = vBody + angVel * radius rotated by 90 degrees
            Double2D velPoly1 = collidePoly1.getVelocity().add(pair.colPoint1.rotate(Angle.halfPI).multiply(collidePoly1.getAngularVelocity()));
            Double2D velPoly2 = collidePoly2.getVelocity().add(pair.colPoint2.rotate(Angle.halfPI).multiply(collidePoly2.getAngularVelocity()));

            // Calculate the relative velocities of the collision points
            Double2D relVel = velPoly1.subtract(velPoly2);
            double relVelNorm = relVel.dot(pair.normal);
                        
            // make sure objects are separating
            if (relVelNorm <= ZERO_VEL)
                {
                pair.relVel = relVelNorm;
                collidingList.add(pair);
                return ADDED_RESPONSE;
                }
            else if (searchingBack)
                {
                // Likely, we have gone back too far, since the wrong set of points 
                // are closest see if we can apply the force to the center of the
                // objects as a last resort just to get them away from each other
                return FOUND_FEATURES;
                }
            }
                
        if (!foundFeatures)
            return PENETRATION;
                        
        else
            return FOUND_FEATURES;
        }
        
    // Find and track the closest feature of a polygon to a circle
    // ActivePair stores the previous closest 
    // feature (if one exists) for the polygon
    private int testPolyCircle(CollisionPair pair, bool alwaysAddResponse)
        {
        bool reversed;
        PhysicalObject2D collideCircle;
        PhysicalObject2D collidePoly;
        if (pair.c1.getShape() is Polygon)
            {
            collideCircle = pair.c2;
            collidePoly = pair.c1;
            reversed = true;
            }
        else
            {
            collideCircle = pair.c1;
            collidePoly = pair.c2;
            reversed = false;
            }
                
        Polygon shapePoly = (Polygon)collidePoly.getShape();
        Circle shapeCircle = (Circle)collideCircle.getShape();

        // Get the vertices and edges of the polygons
        Double2D[] vertices = shapePoly.getVertices();
        Double2D[] edges = shapePoly.getEdges();
        Double2D[] normals = shapePoly.getNormals();
                
        double dist = 0;
        bool foundFeatures = false;
        
        // Loop clockwise through the vertices and edges of the polygon to
        // test if they are the closest to the circle. Ideally, since things don't change
        // much between checks, the closest features are going to be the one that 
        // were closest last time, so start the search with them. Edges are indexed
        // by their left vertex (looking out from the center of the polygon)
        int curFeat = pair.closestFeature1 != null ? pair.closestFeature1.Value : 0;
        
        // The vertices and edges of polygon 1
        for (int counter = 0; counter < vertices.Length && !foundFeatures; counter++)
            {
            int prevFeat = curFeat == 0 ? vertices.Length - 1 : curFeat - 1;
            int nextFeat = (curFeat + 1) % vertices.Length;
                        
            // Since the circle is equal in all directions, just see if the circle's center
            // falls into the current feature's VR.
                        
            // VERTEX vs. CIRCLE 
            // The Voronoi region of a vertex falls between the normal to the edge
            // on the left and the normal of the edge on the right
            if (testVR(vertices[curFeat], normals[prevFeat], normals[curFeat], collideCircle.getPosition(), false))
                {
                // Found the closest features
                foundFeatures = true;
                pair.closestFeature1 = new int?(curFeat);
                                
                if (reversed)
                    {
                    // normal should point from circle to poly
                    pair.normal = vertices[curFeat].subtract(collideCircle.getPosition());
                    dist = pair.normal.length();
                    pair.colPoint1 = vertices[curFeat].subtract(collidePoly.getPosition());
                    pair.colPoint2 = vertices[curFeat].subtract(collideCircle.getPosition());
                    }
                else
                    {
                    // normal should point from poly to circle
                    pair.normal = collideCircle.getPosition().subtract(vertices[curFeat]);
                    dist = pair.normal.length();
                    pair.colPoint2 = vertices[curFeat].subtract(collidePoly.getPosition());
                    pair.colPoint1 = vertices[curFeat].subtract(collideCircle.getPosition());
                    }
                }
                                
            // EDGE vs. CIRCLE
            // The Voronoi region of an edge is just its normal extending out from both 
            // vertices
            if (!foundFeatures)
                {
                // Find the point on edge2 that is closest to vertex1
                // by getting a vector from vertex2 to vertex1
                // and projecting it onto edge2
                Double2D vecOther = collideCircle.getPosition().subtract(vertices[curFeat]);
                double proj = vecOther.dot(edges[curFeat]);
                Double2D edgePoint = vertices[curFeat].add(edges[curFeat].multiply(proj));
                                
                // Now see if the circle lies in the edge's VR
                if (testVR(vertices[curFeat], normals[curFeat], edges[curFeat], collideCircle.getPosition(), true)
                    && testVR(vertices[nextFeat], edges[curFeat].multiply(-1), normals[curFeat], collideCircle.getPosition(), true))
                    {
                    foundFeatures = true;
                    pair.closestFeature1 = new int?(curFeat);
                                        
                    dist = collideCircle.getPosition().subtract(edgePoint).length();
                                        
                    if (reversed)
                        {
                        pair.colPoint1 = edgePoint.subtract(collidePoly.getPosition());
                        pair.colPoint2 = edgePoint.subtract(collideCircle.getPosition());
                        pair.normal = new Double2D(-normals[curFeat].x, -normals[curFeat].y);
                        }
                    else
                        {
                        pair.colPoint2 = edgePoint.subtract(collidePoly.getPosition());
                        pair.colPoint1 = edgePoint.subtract(collideCircle.getPosition());
                        pair.normal = normals[curFeat];
                        }
                    }
                }       
            curFeat = (curFeat + 1) % vertices.Length;
            }
                
        // Add response if features are less than tolerance from each other
        if (foundFeatures && ((dist < (shapeCircle.getRadius() + tolerance)) || alwaysAddResponse))
            {
            // Get the velocities of the collision points
            // vPoint = vBody + angVel * radius rotated by 90 degrees
            Double2D velPoly = collidePoly.getVelocity().add(pair.colPoint1.rotate(Angle.halfPI).multiply(collidePoly.getAngularVelocity()));
            Double2D velCircle = collideCircle.getVelocity();

            // Calculate the relative velocities of the collision points
            Double2D relVel;
            double relVelNorm;
                        
            if (reversed)
                relVel = velPoly.subtract(velCircle);
            else
                relVel = velCircle.subtract(velPoly);
                        
            relVelNorm = relVel.dot(pair.normal);
                        
            // Make sure objects aren't separating
            if (relVelNorm <= ZERO_VEL)
                {
                pair.relVel = relVelNorm;
                collidingList.add(pair);
                return ADDED_RESPONSE;
                }
            }
                
        if (!foundFeatures)
            return PENETRATION;
        else
            return FOUND_FEATURES;
        }
    }
}
