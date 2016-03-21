
#ifndef _KDTREE_KDTREE_ANN_H_
#define _KDTREE_KDTREE_ANN_H_

#include <truck_detector/kdtree/kdtree.h>
#include <ANN/ANN.h>
#include <boost/thread/mutex.hpp>

namespace kdtree
{
  class KdTreeANN : public KdTree
  {
    public:
      /** \brief Constructor for KdTree.
        * \param points the ROS point cloud data array
        */
      KdTreeANN (const sensor_msgs::PointCloud &points)
      {
	ann_kdtree_ = NULL; // to avoid a bad delete in the destructor
	epsilon_ = 0.0; // default error bound value
	dim_ = 3; // default number of dimensions (xyz = 3)
	bucket_size_ = std::min (30, (int)points.points.size()); // default bucket size value

	// allocate enough data
	nr_points_ = convertCloudToArray (points);
	if (nr_points_ == 0)
	{
	  ROS_ERROR ("[KdTreeANN] Could not create kD-tree for %d points!", nr_points_);
	  return;
	}

        // create the kdtree representation
        m_lock_.lock ();
        ann_kdtree_ = new ANNkd_tree (points_, nr_points_, dim_, bucket_size_);
        m_lock_.unlock ();
      }

      /** \brief Search for all the nearest neighbors of the query point in a given radius.
       * \param index the index in \a points representing the query point
       * \param radius the radius of the sphere bounding all of p_q's neighbors
       * \param k_indices the resultant point indices
       * \param k_distances the resultant point distances
       * \param max_nn if given, bounds the maximum returned neighbors to this value
       */
      virtual inline bool radiusSearch (int index, double radius, std::vector<int> &k_indices, std::vector<float> &k_distances, int max_nn = INT_MAX)
      {
	radius *= radius;

	m_lock_.lock();
	int neighbors_in_radius = ann_kdtree_->annkFRSearch (points_[index], radius, 0, NULL, NULL, epsilon_);
	m_lock_.unlock();

	if (neighbors_in_radius == 0)
	  return false;

	if (neighbors_in_radius > max_nn)
	  neighbors_in_radius = max_nn;

	k_indices.resize (neighbors_in_radius);
	k_distances.resize (neighbors_in_radius);

        m_lock_.lock ();
        ann_kdtree_->annkFRSearch (points_[index], radius, neighbors_in_radius, &k_indices[0], &k_distances[0], epsilon_);
        m_lock_.unlock ();

	return true;
      } //@function radiusSearch


    private:
      int convertCloudToArray (const sensor_msgs::PointCloud &cloud);


      boost::mutex m_lock_;

      /** \brief The ANN kdtree object */
      ANNkd_tree* ann_kdtree_;

      /** \brief Internal tree bucket size */
      double bucket_size_;

      /** \brief Internal pointer to data */
      ANNpointArray points_;

      /** \brief Number of points in the tree */
      int nr_points_;
      /** \brief Tree dimensionality (i.e. the number of dimensions per point) */
      int dim_;

  }; //@class KdTreeANN 

} //@namespace kdtree

#endif //_KDTREE_KDTREE_ANN_H_
