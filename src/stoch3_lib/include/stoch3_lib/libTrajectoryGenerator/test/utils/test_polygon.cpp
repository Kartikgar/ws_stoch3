#include <iostream>
#include <cmath>

#include "utils/polygon2d.h"


/* Call this macro to run the function and declare the results */
#define TEST(func) \
  if( func() ){std::cout<<"Test: " << #func << " passed." << std::endl << std::endl;} \
  else {std::cout<<"Test: " << #func << " failed." << std::endl << std::endl;}

using namespace utils;

bool test_polygon_area()
{
  std::cout << "TEST: " << "test_polygon_area" << std::endl;

  // Test1: Area of isosceles triangle
  {
    Vector2d p1{0, 5};
    Vector2d p2{-5, 0};
    Vector2d p3{5, 0};
    Polygon2d p({p1, p2, p3});
    double area = p.getArea();

    if( abs(area - 25.) > 1e-8)
      return false;
  }
  std::cout << "Test " << 1 << " pass." << std::endl;
  
  // Test2: Area of right triangle
  {
    Vector2d p1{0, 5};
    Vector2d p2{0, 0};
    Vector2d p3{5, 0};
    Polygon2d p({p1, p2, p3});
    double area = p.getArea();

    if( abs(area - 12.5) > 1e-8)
      return false;
  }
  std::cout << "Test " << 2 << " pass." << std::endl;

  // Test3: Area of square
  {
    Vector2d p1{5, 5};
    Vector2d p2{5, -5};
    Vector2d p3{-5, -5};
    Vector2d p4{-5, 5};
    Polygon2d p({p1, p2, p3, p4});
    double area = p.getArea();

    if( abs(area - 100.) > 1e-8)
      return false;
  }
  std::cout << "Test " << 3 << " pass." << std::endl;


  // Test4: Area of pentagon
  {
    Vector2d p1{0, 5};
    Vector2d p2{5, 0};
    Vector2d p3{5, -5};
    Vector2d p4{-5, -5};
    Vector2d p5{-5, 0};

    Polygon2d p({p1, p2, p3, p4, p5});
    double area = p.getArea();

    if( abs(area - 75.) > 1e-8)
      return false;
  }
  std::cout << "Test " << 4 << " pass." << std::endl;

  // Test5: Area of line
  {
    Vector2d p1{-5, -5};
    Vector2d p2{-5, 0};

    Polygon2d p({p1, p2});
    double area = p.getArea();

    if( abs(area - 0.) > 1e-8)
      return false;
  }
  std::cout << "Test " << 5 << " pass." << std::endl;

  // Test5: Area of point
  {
    Vector2d p1{-5, -5};

    Polygon2d p({p1});
    double area = p.getArea();

    if( abs(area - 0.) > 1e-8)
      return false;
  }
  std::cout << "Test " << 6 << " pass." << std::endl;

  return true;
}


bool test_polygon_centroid()
{
  std::cout << "TEST: " << "test_polygon_centroid" << std::endl;

  // Test1: Centroid of point
  {
    Vector2d p1{0, 5};
    Polygon2d p({p1});
    Vector2d p_c = p.getCentroid();

    if((p_c - p1).norm() > 1e-8)
      return false;
  }
  std::cout << "Test " << 1 << " pass." << std::endl;
  
  // Test2: Centroid of line
  {
    Vector2d p1{0, 5};
    Vector2d p2{0, -5};

    Polygon2d p({p1, p2});
    Vector2d p_c = p.getCentroid();

    if((p_c - Vector2d{0, 0}).norm() > 1e-8)
      return false;
  }
  std::cout << "Test " << 2 << " pass." << std::endl;

  // Test3: Centroid of triangle
  {
    Vector2d p1{0, 3};
    Vector2d p2{0, -3};
    Vector2d p3{3, 0};

    Polygon2d p({p1, p2, p3});
    Vector2d p_c = p.getCentroid();

    if((p_c - Vector2d{1, 0}).norm() > 1e-8)
      return false;
  }
  std::cout << "Test " << 3 << " pass." << std::endl;

  // Test4: Centroid of rhombus
  {
    Vector2d p1{0, 3};
    Vector2d p2{0, -3};
    Vector2d p3{3, 0};
    Vector2d p4{-3, 0};

    Polygon2d p({p1, p2, p3, p4});
    Vector2d p_c = p.getCentroid();

    if((p_c - Vector2d{0, 0}).norm() > 1e-8)
      return false;
  }
  std::cout << "Test " << 4 << " pass." << std::endl;


  return true;
}


bool test_point_inclusion()
{
  std::cout << "TEST: " << "test_point_inclusion" << std::endl;

  // Test1: Point in polygon with 1 vertex
  { 
    Vector2d p1{0, 3};
    Polygon2d p({p1});
    Vector2d p_in{0, 3};
    bool point_in_polygon = p.pointInPolygon(p_in);
    if(point_in_polygon == false)
      return false;
  }
  std::cout << "Test " << 1 << " pass." << std::endl;

  // Test2: Point in polygon with 1 vertex
  { 
    Vector2d p1{0, 3};
    Polygon2d p({p1});
    Vector2d p_in{1, 3};
    bool point_in_polygon = p.pointInPolygon(p_in);
    if(point_in_polygon == true)
      return false;
  }
  std::cout << "Test " << 2 << " pass." << std::endl;


  // Test3: Point in polygon with 2 vertices
  { 
    Vector2d p1{0, 0};
    Vector2d p2{0, 3};
    Polygon2d p({p1, p2});
    Vector2d p_in{0, 2.5};
    bool point_in_polygon = p.pointInPolygon(p_in);
    if(point_in_polygon == false)
      return false;
  }
  std::cout << "Test " << 3 << " pass." << std::endl;
  
  // Test4: Point in polygon with 2 vertices
  { 
    Vector2d p1{0, 0};
    Vector2d p2{0, 3};
    Polygon2d p({p1, p2});
    Vector2d p_in{0, 5};
    bool point_in_polygon = p.pointInPolygon(p_in);
    if(point_in_polygon == true)
      return false;
  }
  std::cout << "Test " << 4  << " pass." << std::endl;

  // Test5: Point in polygon with 2 vertices
  { 
    Vector2d p1{0, 0};
    Vector2d p2{0, 3};
    Polygon2d p({p1, p2});
    Vector2d p_in{0, 3};
    bool point_in_polygon = p.pointInPolygon(p_in);
    if(point_in_polygon == false)
      return false;
  }
  std::cout << "Test " << 5 << " pass." << std::endl;
  
  // Test6: Point in polygon with 3 vertices
  { 
    Vector2d p1{3, 0};
    Vector2d p2{0, 3};
    Vector2d p3{0, -3};

    Polygon2d p({p1, p2, p3});
    Vector2d p_in{1, 0};
    bool point_in_polygon = p.pointInPolygon(p_in);
    if(point_in_polygon == false)
      return false;
  }
  std::cout << "Test " << 6 << " pass." << std::endl;

  // Test7: Point outside polygon with 3 vertices
  { 
    Vector2d p1{3, 0};
    Vector2d p2{0, 3};
    Vector2d p3{0, -3};

    Polygon2d p({p1, p2, p3});
    Vector2d p_in{10, 0};
    bool point_in_polygon = p.pointInPolygon(p_in);
    if(point_in_polygon == true)
      return false;
  }
  std::cout << "Test " << 7 << " pass." << std::endl;

  // Test8: Point on edge of polygon with 3 vertices
  { 
    Vector2d p1{3, 0};
    Vector2d p2{0, 3};
    Vector2d p3{0, -3};

    Polygon2d p({p1, p2, p3});
    Vector2d p_in{0, 0};
    bool point_in_polygon = p.pointInPolygon(p_in);
    if(point_in_polygon == false)
      return false;
  }
  std::cout << "Test " << 8 << " pass." << std::endl;
  
  // Test9: Point on vertex of polygon with 3 vertices
  { 
    Vector2d p1{3, 0};
    Vector2d p2{0, 3};
    Vector2d p3{0, -3};

    Polygon2d p({p1, p2, p3});
    Vector2d p_in{3, 0};
    bool point_in_polygon = p.pointInPolygon(p_in);
    if(point_in_polygon == false)
      return false;
  }
  std::cout << "Test " << 9 << " pass." << std::endl;

  return true;
}

bool test_stability_margin()
{
  std::cout << "TEST: " << "test_stability_margin" << std::endl;

  // Test1: Polygon with one vertex
  { 
    Vector2d p1{3, 0};
    Polygon2d p({p1});
    
    Vector2d p_in{3, 0};
    double stability_margin = p.getStabilityMargin(p_in);
    if((stability_margin - 0) > 1e-8)
      return false;
  }
  std::cout << "Test " << 1 << " pass." << std::endl;

  // Test2: Polygon with two vertices
  { 
    Vector2d p1{3, 0};
    Vector2d p2{-3, 0};
    Polygon2d p({p1, p2});
    
    Vector2d p_in{3, 3};
    double stability_margin = p.getStabilityMargin(p_in);
    if((stability_margin - 0) > 1e-8)
      return false;
  }
  std::cout << "Test " << 2 << " pass." << std::endl;

  // Test3: Polygon with three vertices
  { 
    Vector2d p1{3, 0};
    Vector2d p2{-3, 0};
    Vector2d p3{0, 5};
    Polygon2d p({p1, p2, p3});
    
    Vector2d p_in{0, 1};
    double stability_margin = p.getStabilityMargin(p_in);
    if((stability_margin - 1) > 1e-8)
      return false;
  }
  std::cout << "Test " << 3 << " pass." << std::endl;

  // Test3: Polygon with three vertices
  { 
    Vector2d p1{ 3,  3};
    Vector2d p2{ 3, -3};
    Vector2d p3{-3,  3};
    Vector2d p4{-3, -3};
    Polygon2d p({p1, p2, p3, p4});
    
    Vector2d p_in{0, 1};
    double stability_margin = p.getStabilityMargin(p_in);
    if((stability_margin - 2) > 1e-8)
      return false;
  }
  std::cout << "Test " << 3 << " pass." << std::endl;

  // Test4: Polygon with three vertices
  { 
    Vector2d p1{ 3,  3};
    Vector2d p2{ 3, -3};
    Vector2d p3{-3,  3};
    Vector2d p4{-3, -3};
    Polygon2d p({p1, p2, p3, p4});
    
    Vector2d p_in{0, -1};
    double stability_margin = p.getStabilityMargin(p_in);
    if((stability_margin - 2) > 1e-8)
      return false;
  }
  std::cout << "Test " << 4 << " pass." << std::endl;
  
  // Test5: Polygon with three vertices
  { 
    Vector2d p1{ 3,  3};
    Vector2d p2{ 3, -3};
    Vector2d p3{-3,  3};
    Vector2d p4{-3, -3};
    Polygon2d p({p1, p2, p3, p4});
    
    Vector2d p_in{2.5, 1};
    double stability_margin = p.getStabilityMargin(p_in);
    if((stability_margin - 0.5) > 1e-8)
      return false;
  }
  std::cout << "Test " << 5 << " pass." << std::endl;
  
  // Test6: Polygon with three vertices
  { 
    Vector2d p1{ 3,  3};
    Vector2d p2{ 3, -3};
    Vector2d p3{-3,  3};
    Vector2d p4{-3, -3};
    Polygon2d p({p1, p2, p3, p4});
    
    Vector2d p_in{5, 1};
    double stability_margin = p.getStabilityMargin(p_in);
    if((stability_margin - (-1)) > 1e-8)
      return false;
  }
  std::cout << "Test " << 6 << " pass." << std::endl;

  return true;
}


int main(int argc, char** argv)
{
  TEST(test_polygon_area);
  TEST(test_polygon_centroid);
  TEST(test_point_inclusion);
  TEST(test_stability_margin);

  return 0;

}
