#ifndef __VERTICES_INFO_H__
#define __VERTICES_INFO_H__

#include <vector>
#include <Eigen/Core>
#include <boost/geometry.hpp> 
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/numeric/conversion/bounds.hpp>
#include "boost/tuple/tuple.hpp"
#include <boost/foreach.hpp>

namespace computational_geometry{
  
  //typedef boost::tuple<double, double> point_t;
  typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point_t;
  typedef boost::geometry::model::polygon<point_t, true, false> polygon_t;
  
  class VerticesInfo{
  private:
    polygon_t m_poly;
    polygon_t m_hull;
    
    std::vector<Eigen::Vector3d> m_vertices;
    std::vector<Eigen::Vector3d> m_vertices_hull;
    
    double pointToSegment(Eigen::Vector3d& closest_point,
                          const Eigen::Vector3d& p,
                          const Eigen::Vector3d& p1,
                          const Eigen::Vector3d& p2,
                          double eps = 1.0e-8)
    {
      const Eigen::Vector3d ab(p2 - p1);
      const Eigen::Vector3d ac(p - p1);
      
      if( ab.x()*ab.x()+ab.y()*ab.y() < eps*eps ){
        closest_point = p1;
        return 0.0;
      }
      
      const double alpha  = (ab.x()*ac.x()+ab.y()*ac.y()) / (ab.x()*ab.x()+ab.y()*ab.y());
      const double dir_sign = (ab.x()*ac.y() - ab.y()*ac.x()) > 0.0 ? 1.0 : -1.0;
      
      if( alpha <= 0 ){
        closest_point = p1;
        return dir_sign * sqrt(ac.x()*ac.x() + ac.y()*ac.y());
      }
      else if( alpha >= 1.0 ){
        closest_point = p2;
        double bcx = p.x() - p2.x();
        double bcy = p.y() - p2.y();
        return dir_sign * sqrt(bcx*bcx + bcy*bcy);
      }
      else{
         const Eigen::Vector3d m(p1 + alpha * ab);
        closest_point = m;
        double cmx = m.x() - p.x();
        double cmy = m.y() - p.y();
        return dir_sign * sqrt(cmx*cmx + cmy*cmy);
      }
    }
    
  public:
    VerticesInfo(void) {
      m_vertices.clear();
      m_vertices_hull.clear();
    }

    VerticesInfo(const std::vector<Eigen::Vector3d>& vertices) {
      m_vertices = vertices;
      m_vertices_hull.clear();
      makeConvexHull();
    }    
    
    virtual ~VerticesInfo(){}
    
    void clear(void){
      m_vertices.clear();
      m_vertices_hull.clear();
    }
    
    bool empty(void){
      return m_vertices.empty();
    }
    
    int size(void){
      return (int)m_vertices.size();
    }
    
    int numberOfEffectiveVertices(void) const {
      return (int)m_vertices_hull.size();
    }
    
    void addVertex(const Eigen::Vector3d& v){
      m_vertices.push_back(v);
    }
    
    void addVertex(double x, double y, double z){
      addVertex(Eigen::Vector3d(x, y, z));
    }
    
    void addVertices(const std::vector<Eigen::Vector3d>& vertices){
      m_vertices = vertices;
      makeConvexHull();
    }
    
    const Eigen::Vector3d& getVertex(int nVertices) const {
      return m_vertices[nVertices];
    }
    
    const Eigen::Vector3d& getEffectiveVertex(int index) const {
      return m_vertices_hull[index];
    }
    
    int makeConvexHull()
    {
      //std::cout << "size of vertices = " << m_vertices.size() << std::endl;
      
      m_poly.clear();
      m_hull.clear();
      for( auto v : m_vertices ){
        m_poly.outer().push_back(point_t(v.x(), v.y()));
        //std::cout << "poly=" << m_poly.outer().back().get<0>() << "," << m_poly.outer().back().get<1>() << std::endl;
      }
      boost::geometry::convex_hull(m_poly, m_hull);
      
      //std::cout << "size of hull = " << m_hull.outer().size() << std::endl;
      
      m_vertices_hull.clear();
      for( auto h : m_hull.outer() )
        m_vertices_hull.push_back(Eigen::Vector3d(h.get<0>(), h.get<1>(), m_vertices[0].z()));
      return (int)m_vertices_hull.size();
    }
    
    /*! @brief compute closest point of convex hull to target point
     * @param[out] closest_point 
     * @param[in]  pos target point
     * @param[in]  v   set of vertex
     * @return distance between target point and closest point
     */
    double getClosestPoint(Eigen::Vector3d& closest_point,
                           const Eigen::Vector3d& check_point)
    {
      if( m_vertices_hull.empty() )
        makeConvexHull();
      
      double dmin = 1.0e30;
      Eigen::Vector3d c_point;
      for( unsigned int i = 0 ; i < m_vertices_hull.size()-1 ; i++ ){
        double d = fabs(pointToSegment(c_point, check_point,
                                       m_vertices_hull[i], m_vertices_hull[i+1]));
        
        if( dmin > d ){
          dmin = d;
          closest_point = c_point;
        }
      }
      {
        double d = fabs(pointToSegment(c_point, check_point,
                                       m_vertices_hull.back(), m_vertices_hull.front()));
        
        if( dmin > d ){
          dmin = d;
          closest_point = c_point;
        }
      }
      return dmin;
    }
    
    /*! @brief whether if convex hull include target point
     * @param[in] check_point target position(x,y)
     * @return true inside
     * @return false outside
     */
    bool checkConvexHullInclusion(const Eigen::Vector3d& check_point){
      point_t pos(check_point.x(), check_point.y());
      return boost::geometry::within(pos, m_hull);
    }
    
  };
  
}

#endif // __VERTICES_INFO_H__
