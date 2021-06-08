#ifndef __SUPPORT_POLYGON_MANAGER__
#define __SUPPORT_POLYGON_MANAGER__

#include <unordered_map>
#include "VerticesInfo.h"

namespace computational_geometry{
  
  template<class T> class SupportPolygonManager {
  protected:
    VerticesInfo wVertices_all_;
    std::unordered_map<T, bool> isContact_;
    std::unordered_map<T, VerticesInfo> wVertices_;
    
    void makeConvexHull(const T& node);
    
  public:
    SupportPolygonManager();
    
    std::vector<bool>& isContact(){ return isContact_; }
    
    bool getContact(const T& node){ return isContact_[node]; }
    //void setContact(const T& node, bool contact){ isContact_[node] = contact; }
    
    void clear();
    
    void addVertices(const T& node, const std::vector<Eigen::Vector3d>& v, bool contact);
    
    bool checkAllConvexHullInclusion(Eigen::Vector3d& check_point);
    
    bool checkConvexHullInclusion(const T& node, Eigen::Vector3d& check_point);
    
    void getGlobalAllSupportPolygon();
    
    double getClosestPointInAllSupportPolygon(Eigen::Vector3d& closest_point,
                                              Eigen::Vector3d& check_point);
    
    double getClosestPointInSupportPolygon(const T& node,
                                           Eigen::Vector3d& closest_point,
                                           Eigen::Vector3d& check_point);
    
    bool checkAllSupportPolygon(Eigen::Vector3d& check_point, double& distance_min);
    
    bool checkAllSupportPolygon(Eigen::Vector3d& check_point);
    
    bool checkSupportPolygon(const T& node, Eigen::Vector3d& check_point,
                             double& distance_min);
    
    bool checkSupportPolygon(const T& node, Eigen::Vector3d& check_point);
    
  };
  
  template<class T>
    SupportPolygonManager<T>::SupportPolygonManager()
  {
    clear();
  }
  
  template<class T>
    void SupportPolygonManager<T>::clear()
  {
    isContact_.clear();
    wVertices_.clear();
    wVertices_all_.clear();
  }

  template<class T>
    void SupportPolygonManager<T>::addVertices(const T& node,
                                               const std::vector<Eigen::Vector3d>& v,
                                               bool contact)
  {
    VerticesInfo vInfo(v);
    wVertices_[node] = vInfo;
    isContact_[node] = contact;
  }

  template<class T>
    void SupportPolygonManager<T>::makeConvexHull(const T& node)
  {
    wVertices_[node].makeConvexHull();
  }
  
  template<class T>
    void SupportPolygonManager<T>::getGlobalAllSupportPolygon()
  {
    wVertices_all_.clear();
    for( auto & wV : wVertices_ ){
      if( isContact_[wV.first] )
      {
        for( int n = 0 ; n < wV.second.size() ; n++ )
        {
          wVertices_all_.addVertex(wV.second.getVertex(n));
        }
      }
    }
    if( !wVertices_all_.empty() )
      wVertices_all_.makeConvexHull();
  }

  template<class T>
    bool SupportPolygonManager<T>::checkAllConvexHullInclusion(Eigen::Vector3d& check_point)
  {
    return wVertices_all_.checkConvexHullInclusion(check_point);
  }

  template<class T>
    bool SupportPolygonManager<T>::checkConvexHullInclusion(const T& node,
                                                           Eigen::Vector3d& check_point)
  {
    return wVertices_[node].checkConvexHullInclusion(check_point);
  }

  template<class T>
    double SupportPolygonManager<T>::getClosestPointInAllSupportPolygon(Eigen::Vector3d& closest_point,
                                                                        Eigen::Vector3d& check_point)
  {
    return wVertices_all_.getClosestPoint(closest_point, check_point);
  }

  template<class T>
    double SupportPolygonManager<T>::getClosestPointInSupportPolygon(const T& node,
                                                                     Eigen::Vector3d& closest_point,
                                                                     Eigen::Vector3d& check_point)
  {
    return wVertices_[node].getClosestPoint(closest_point, check_point);
  }

  template<class T>
    bool SupportPolygonManager<T>::checkAllSupportPolygon(Eigen::Vector3d& check_point,
                                                          double& distance_min)
  {
    if( !wVertices_all_.checkConvexHullInclusion(check_point) ){
      Eigen::Vector3d closest_point;
      distance_min = wVertices_all_.getClosestPoint(closest_point, check_point);
      check_point = closest_point;
      return false;
    }
    
    return true;
  }

  template<class T>
    bool SupportPolygonManager<T>::checkAllSupportPolygon(Eigen::Vector3d& check_point)
  {
    double distance_min = std::numeric_limits<double>::max();
    return checkAllSupportPolygon(check_point, distance_min);
  }
  
  template<class T>
    bool SupportPolygonManager<T>::checkSupportPolygon(const T& node,
                                                       Eigen::Vector3d& check_point,
                                                       double& distance_min)
  {
    if( !wVertices_[node].checkConvexHullInclusion(check_point) ){
      Eigen::Vector3d closest_point;
      distance_min = wVertices_[node].getClosestPoint(closest_point, check_point);
      check_point = closest_point;
      return false;
    }
    
    return true;
  }

  template<class T>
    bool SupportPolygonManager<T>::checkSupportPolygon(const T& node,
                                                       Eigen::Vector3d& check_point)
  {
    double distance_min = std::numeric_limits<double>::max();
    return checkSupportPolygon(node, check_point, distance_min);
  }
  
}

#endif // __SUPPORT_POLYGON_MANAGER__
