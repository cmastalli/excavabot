
/** \author Carlos Mastalli */
/** \description This code is based on paper: Atkeson, C., Moore, A. and Schaal,
                 S. "Locally Weighted Regression". Which this using a weighted
                 function that is a Gaussian kernel. */
                 

#ifndef LWR_H_
#define LWR_H_

// system includes
#include <vector>
#include <string>

// ros includes
#include <ros/ros.h>

#include <Eigen/Eigen>

// local includes
#include <lwr/Model.h>

namespace lwr
{

class LocallyWeightedRegression
{
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*! Constructor
     */
    LocallyWeightedRegression();

    /*! Destructor
     */
    ~LocallyWeightedRegression();

    /*! Initialize the LWR model from several methods: using the parameters from the parameters server in parameter_namespace or from message
     * @return True on success, false on failure
     */
    bool initialize(ros::NodeHandle& node_handle);

    bool initFromMessage(const lwr::Model& model);

    bool initializeFromDisc(const std::string directory_name);

    /*! Writes LWR model to message
     * @param model
     * @return
     */
    bool writeToMessage(lwr::Model& model);

     /*! Writes the LWR model to file
     * @param directory_name (input) Directory name in which the LWR model is stored.
     * @return True on success, false on failure
     */
    bool writeToDisc(const std::string directory_name);

    /*! Learn weights of LWR method
     * @param x
     * @param y
     * @return True on success, false on failure
     */
    bool learnWeights(const Eigen::VectorXd& x_input_vector,
                      const Eigen::VectorXd& y_target_vector);

    /*! Predict the data based in a LWR
     * @param x
     * @param y
     * @return True on success, false on failure
     */
    bool predict(const double x_query,
                 double& y_prediction);

    /*! Generate basis function matrix
     * @param x_input_vector
     * @param basis_function_matrix
     * @return True on success, false on failure
     */
    bool generateBasisFunctionMatrix(const Eigen::VectorXd& x_input_vector,
                                     Eigen::MatrixXd& basis_function_matrix);

    /*! Gets the theta vector
     * @param thetas
     * @return True on success, false on failure
     */
    bool getThetas(Eigen::VectorXd& thetas);

    /*! Sets the theta vector
     * @param thetas
     * @return True on success, false on failure
     */
    bool setThetas(const Eigen::VectorXd& thetas);

    /*! Updates the theta vectors (thetas += delta_thetas)
     * @param delta_thetas
     * @return True on success, false on failure
     */
    bool updateThetas(const Eigen::VectorXd& delta_thetas);

    /*! Gets widths and centers of LWR model
     * @param widths
     * @param centers
     * @return True on success, false on failure
     */
    bool getWidthsAndCenters(Eigen::VectorXd& widths,
                             Eigen::VectorXd& centers);

    /*! Returns a string containing relevant information of the LWR model
     */
    std::string getInfoString();

    /*! Gets the number of receptive fields
     * @param num_rfs
     * @return True on success, false on failure
     */
    bool getNumRFS(int& num_rfs);

    std::string getPathName();


  private:

    ros::NodeHandle node_handle_;

    /*! Get the Gaussian kernel
     * @param x_input
     * @param center_index
     */
    double getGaussianKernel(const double x_input,
                             const int center_index);

    /*! Indicates whether the LWR model is initialized
     * @return True on success, false on failure
     */
    bool initialized_;

    std::string path_name_;

    double can_sys_cutoff_;

    double rfs_width_boundary_;

    /*! Number of receptive fields used in this LWR model
     */
    int num_rfs_;

    /*! Determines whether gaussians are exponentially spaced (true) or equally spaced (false)
     *  exponential scale is used to deal with the nonlinearity of the
     *  phase variable of the canonical system of a DMP
     */
    bool exponentially_spaced_;

    /*! Centers of the receptive fields
     */
    Eigen::VectorXd centers_;

    /*! Bandwidth used for each local model
     */
    Eigen::VectorXd widths_;

    /*! Slopes of the local linear approximations
     */
    Eigen::VectorXd thetas_;

    /*! Offsets of the local linear approximations. (currently not implemented)
     */
    Eigen::VectorXd offsets_;
};

}
#endif /* LWR_H_ */
