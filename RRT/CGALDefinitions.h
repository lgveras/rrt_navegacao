#pragma once
#include <CGAL/Polygon_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/Polytope_distance_d.h>
#include <CGAL/Polytope_distance_d_traits_2.h>
#include <CGAL/squared_distance_2.h>
#include <CGAL/intersections.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/create_offset_polygons_2.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/enum.h>
#include <boost/shared_ptr.hpp>

//typedef CGAL::Exact_predicates_exact_constructions_kernel CGALKernel;
typedef CGAL::Simple_cartesian<double> CGALKernel;
typedef CGAL::Exact_predicates_inexact_constructions_kernel InexactKernel;
//typedef CGAL::Exact_predicates_exact_constructions_kernel K;

typedef CGAL::Polygon_2<CGALKernel> Polygon_2;
typedef CGALKernel::Point_2 Point_2;
typedef CGALKernel::Segment_2 Segment_2;
typedef CGALKernel::Circle_2 Circle_2;

typedef Polygon_2::Vertex_iterator VertexIterator;
typedef Polygon_2::Edge_const_iterator EdgeIterator;
typedef Polygon_2::Vertex_iterator VertexIterator;

typedef CGALKernel::Intersect_2 Intersect_2;

typedef CGAL::Polytope_distance_d_traits_2<CGALKernel> Traits;
typedef CGAL::Polytope_distance_d<Traits> PolytopeDistance;
typedef CGAL::Polygon_with_holes_2<CGALKernel> Polygon_with_holes_2;

typedef CGAL::Straight_skeleton_2<InexactKernel> StraightSkeleton;
typedef boost::shared_ptr<StraightSkeleton> StraightSkeletonPtr;
