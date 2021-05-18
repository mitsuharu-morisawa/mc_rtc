#include "SupportPolygonManager.h"

using namespace computational_geometry;

SupportPolygonManager::SupportPolygonManager(void)
{
  clear();
}

void SupportPolygonManager::clear(void)
{
  m_isContact_.clear();
  wVertices_.clear();
  wVertices_all_.clear();
}

void SupportPolygonManager::push_back(std::pair<bool, const std::vector<Eigen::Vector3d> >& v)
{
  m_isContact_.push_back(v.first);
  wVertices_.addVertices(v.second);
}

void SupportPolygonManager::getGlobalSupportPolygon(const Eigen::Vector3d& p,
                                                    const Eigen::Matrix3d& R,
                                                    unsigned int node)
{
  VerticesInfo& vInfo = wVertices_[node];
  
  vInfo.clear();
  for( auto& lVertex : localVertices_[node] ){
    Eigen::Vector3d v(p + R * lVertex);
    vInfo.addVertex(v);
  }
  
  vInfo.makeConvexHull();
}

void SupportPolygonManager::getGlobalAllSupportPolygon(void)
{
  wVertices_all_.clear();
  for( unsigned int node = 0 ; node < wVertices_.size() ; node++ ){
    if( isContact_[node] ){
      for( unsigned int n = 0 ; n < wVertices_[node].size() - 1 ; n++ )
        wVertices_all_.addVertex(wVertices_[node].getVertex(n));
    }
  }
  
  if( !wVertices_all_.empty() )
    wVertices_all_.makeConvexHull();
}

int SupportPolygonManager::checkAllConvexHullInclusion(Eigen::Vector3d& check_point)
{
  return wVertices_all_.checkConvexHullInclusion(check_point);
}

int SupportPolygonManager::checkConvexHullInclusion(int n_node,
                                                    Eigen::Vector3d& check_point)
{
  return wVertices_[n_node].checkConvexHullInclusion(check_point);
}

double SupportPolygonManager::getClosestPointInAllSupportPolygon(Eigen::Vector3d& closest_point,
                                                                 Eigen::Vector3d& check_point)
{
  return wVertices_all_.getClosestPoint(closest_point, check_point);
}

double SupportPolygonManager::getClosestPointInSupportPolygon(int n_node,
                                                              Eigen::Vector3d& closest_point,
                                                              Eigen::Vector3d& check_point)
{
  return wVertices_[n_node].getClosestPoint(closest_point, check_point);
}

bool SupportPolygonManager::checkAllSupportPolygon(Eigen::Vector3d& check_point,
                                                   double& distance_min)
{
  if( wVertices_all_.checkConvexHullInclusion(check_point) < 0 ){
    Eigen::Vector3d closest_point;
    distance_min = wVertices_all_.getClosestPoint(closest_point, check_point);
    check_point = closest_point;
    return false;
  }
  
  return true;
}

bool SupportPolygonManager::checkAllSupportPolygon(Eigen::Vector3d& check_point)
{
  if( wVertices_all_.checkConvexHullInclusion(check_point) < 0 ){
    Eigen::Vector3d closest_point;
    wVertices_all_.getClosestPoint(closest_point, check_point);
    check_point = closest_point;
    return false;
  }
  
  return true;
}

bool SupportPolygonManager::checkSupportPolygon(int n_node,
                                                Eigen::Vector3d& check_point,
                                                double& distance_min)
{
  if( wVertices_[n_node].checkConvexHullInclusion(check_point) < 0 ){
    Eigen::Vector3d closest_point;
    distance_min = wVertices_[n_node].getClosestPoint(closest_point, check_point);
    check_point = closest_point;
    return false;
  }
  
  return true;
}

bool SupportPolygonManager::checkSupportPolygon(int n_node,
                                                Eigen::Vector3d& check_point)
{
  if( wVertices_[n_node].checkConvexHullInclusion(check_point) < 0 ){
    Eigen::Vector3d closest_point;
    wVertices_[n_node].getClosestPoint(closest_point, check_point);
    check_point = closest_point;
    return false;
  }
  
  return true;
}

int SupportPolygonManager::
getIntersectionPointInAllSupportPolygon(Eigen::Vector3d& intersection_point,
                                        const Eigen::Vector3d& p,
                                        const Eigen::Vector3d& dir)
{
  return wVertices_all_.getIntersectionPoint(intersection_point, p, dir);
}
