/*
 * file: polygon2d.h
 *
 * Created: 1 Apr, 2022
 * Author : Aditya Sagi
 */

#pragma once

#include <vector>
#include <numeric>

#include "utils/vector_op.h"

namespace utils
{

class Polygon2d 
{
  public:

    /* Constructor.
     *
     * Note: The vertices of the polygon must be listed
     * in order, either CW or CCW.
     * The specified polygon MUST be convex.
     */
    Polygon2d(std::vector<Vector2d> vertices) : vertices_(vertices)
  {}

    /*
     * Find the area covered by the polygon.
     * The area is computed by breaking the polygon (convex polygon) into
     * non-overlapping triangles and summing the areas of all the triangles.
     *
     * The decomposition of the polygon into triangles is done by choosing 
     * the vertices, 1-2-3, 1-3-4, 1-4-5 etc.
     *
     * Note: It is expected that the given polygon is convex and that all 
     * the vertices are specified in order (either CW or CCW).
     *
     * \ret Area of the polygon.
     */ 
    double getArea()
    {
      int num_vertices = vertices_.size();
      double area = 0.;

      // Area of a single point or a line is zero.
      if(num_vertices <= 2)
        return 0.;

      for(auto i=0; i<num_vertices-2; i++)
      {
        area += getTriangleArea_(vertices_[0], vertices_[i+1], vertices_[i+2]);
      }

      return area;
    }

    /*
     * Find the centroid of the polygon.
     *
     * ret Vector2d pointing to the centroid of the polygon
     */
    Vector2d getCentroid()
    {
      return std::accumulate(vertices_.begin(), vertices_.end(), Vector2d{0,0})/vertices_.size();  
    }


    /*
     * Find if the specified point lies inside the polygon.
     *
     * \arg[in] p: Point to check for inclusion in polygon.
     *
     * \ret True if point lies inside (or on the edges of) the polygon.
     *      False otherwise.
     */
    bool pointInPolygon(Vector2d p)
    {
      int num_vertices = vertices_.size();
      double area = 0.;

      if(num_vertices == 0)
      {
        return false;
      }
      else if(num_vertices == 1)
      {
        // If the vertex is same as point
        // then return true, else return false.
        return (p == vertices_[0]);
      }
      else if(num_vertices == 2)
      {
        // If the point to check lies on the line joining the two
        // vertices of the polygon, then the sum of the lengths of 
        // two sides equals the length of the third side.
        double s1 = (p - vertices_[0]).norm();
        double s2 = (p - vertices_[1]).norm();
        double s3 = (vertices_[1] - vertices_[0]).norm();
        return ((s1 + s2 - s3) < 1e-8);
      }

      for(auto i=0; i<num_vertices; i++)
      {
        int vertex_a = (i+0) % num_vertices;
        int vertex_b = (i+1) % num_vertices;

        area += getTriangleArea_(p, vertices_[vertex_a], vertices_[vertex_b]);
      }

      // If the area of the polygon is same as the 
      // sum of the areas of all the triangles formed
      // by the point of interest and all the edges of
      // the polygon, then the point lies inside the
      // polygon.
      if (abs (area - getArea()) < 1e-8)
        return true;

      return false;
    }

    /*
     * Find the stability margin.
     *
     * Stability margin is defined as the smallest distance between a point and one of the sides
     * of the polygon. This is measured when the point is inside the polygon.
     *
     * In this implementation the stability margin will be provided as a positive real number
     * when the point is inside the polygon and -1 when the point is outside the polygon.
     *
     * \arg[in] point: Point of interest for which the stability margin should be determined.
     *
     * \ret stability margin as a double variables.
     */
    double getStabilityMargin(Vector2d point)
    {
      double stability_margin = 0;
      int num_vertices = 0;

      if(!pointInPolygon(point))
        return -1.;

      // Perpendicular distance between a line and a point
      // can be obtained using the area of the triangle as
      // follow:
      // area = 0.5 * base * height
      // height = 2 * area / base
      // The triangle is formed by the point of interest and
      // two other points (which are the vertices of the polygon).

      if (num_vertices == 0)
        return -1;
      else if (num_vertices == 1)
        stability_margin = 0;
      else if (num_vertices == 2)
        stability_margin = 0;
      else
      {
        std::vector<double> distance;
        distance.resize(num_vertices);

        for(auto i=0; i<num_vertices -1; i++)
        {
          Vector2d vertex_a = vertices_[i % num_vertices];
          Vector2d vertex_b = vertices_[(i+1) % num_vertices];

          double base = (vertex_a - vertex_b).norm();
          double area = getTriangleArea_(point, vertex_a, vertex_b);
          double height = 2 * area / base;

          distance[i] = height;

        }

        std::sort(distance.begin(), distance.end());
        stability_margin = distance[0];
      }

      return stability_margin;
    }

  private:
    std::vector<Vector2d> vertices_;

    /*
     * Find the area of a triangle defined by the three vertices p1, p2 and p3.
     *
     * \param[in] p1: Vertex of the triangle.
     *
     * \param[in] p2: Vertex of the triangle.
     *
     * \param[in] p3: Vertex of the triangle.
     *
     * \ret Returns the area of the triangle.
     */
    double getTriangleArea_(Vector2d p1, Vector2d p2, Vector2d p3)
    {
      double term1 = p1[0]*(p2[1] - p3[1]);
      double term2 = p2[0]*(p3[1] - p1[1]);
      double term3 = p3[0]*(p1[1] - p2[1]);

      return abs(0.5*(term1 + term2 + term3));
    }
};

} // END namespace utils
