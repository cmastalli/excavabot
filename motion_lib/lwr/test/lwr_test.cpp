
// ros includes
#include <ros/ros.h>
#include <ros/package.h>

#include <Eigen/Eigen>

#include <iostream>
#include <fstream>

//#include <boost/filesystem.hpp>

// local include
#include <lwr/lwr.h>

using namespace Eigen;
using namespace lwr;

class LocallyWeightedRegressionTest
{

  public:

    LocallyWeightedRegressionTest(ros::NodeHandle& node_handle);

    ~LocallyWeightedRegressionTest();

    bool initialize();

    bool runLocallyWeightedRegressionTest();


  private:

    ros::NodeHandle node_handle_, node_handle_lwr_;

    double testFunction(const double test_x);

    bool initialized_;

    std::string data_directory_name_;

    double rfs_width_boundary_;

    int num_rfs_;

    double can_sys_cutoff_;

    int num_data_learn_, num_data_query_;

    double mse_prediction_error_threshold_;

    double mse_rescaling_error_threshold_;


}; // class LocallyWeightedRegressionTest

LocallyWeightedRegressionTest::LocallyWeightedRegressionTest(ros::NodeHandle& node_handle) :
    node_handle_(node_handle), node_handle_lwr_(node_handle, "lwr"),
    initialized_(false)
{
}

LocallyWeightedRegressionTest::~LocallyWeightedRegressionTest()
{
}

bool LocallyWeightedRegressionTest::initialize()
{
  std::string ns = node_handle_.getNamespace();
  ROS_INFO("The LWR test namespace is: %s", ns.c_str());

  // get number of data to learn
  if (!node_handle_.getParam("num_data_learn", num_data_learn_))
  {
    ROS_FATAL("Could not found num_data_learn from parameter server.");
    return false;
  }

  // get number of data query
  if (!node_handle_.getParam("num_data_query", num_data_query_))
  {
    ROS_FATAL("Could not found num_data_query from parameter server.");
    return false;
  }

  // get mse prediction error threshold
  if (!node_handle_.getParam("mse_prediction_error_threshold", mse_prediction_error_threshold_))
  {
    ROS_FATAL("Could not found mse_prediction_error_threshold from parameter server.");
    return false;
  }

  // get mse rescaling error threshold
  if (!node_handle_.getParam("mse_rescaling_error_threshold", mse_rescaling_error_threshold_))
  {
    ROS_FATAL("Could not found mse_rescaling_error_threshold from parameter server.");
    return false;
  }

  if (!node_handle_lwr_.getParam("num_rfs", num_rfs_))
  {
    ROS_FATAL("Could not found num_rfs from parameter server.");
    return false;
  }


/*
  // create directoy if it doesn't exist
  boost::filesystem::create_directories(library_directory_name_);
  boost::filesystem::create_directories(data_directory_name_);
*/
  initialized_ = true;
  return initialized_;
}

bool LocallyWeightedRegressionTest::runLocallyWeightedRegressionTest()
{
  LocallyWeightedRegression lwr;
  if (!lwr.initialize(node_handle_))
  {
    ROS_ERROR("Could not initialized LWR model");
    return false;
  }

  // generate input vector
  VectorXd x_test = VectorXd::Zero(num_data_learn_);
  x_test(0) = 0;
  double dx = static_cast<double> (1.0) / (x_test.size() - 1);
  for(int i = 1; i < x_test.size(); i++)
  {
    x_test(i) =  x_test(i - 1) + dx;
  }

  // generate target vector
  VectorXd y_test = VectorXd::Zero(num_data_learn_);
  for(int i = 1; i < x_test.size(); i++)
  {
    y_test(i) = testFunction(x_test(i));
  }

  // learn weights
  if (!lwr.learnWeights(x_test, y_test))
  {
    ROS_ERROR("Could not learn weigths");
    return false;
  }

  // generate query input vector
  VectorXd xq_test = VectorXd::Zero(num_data_query_);
  xq_test(0) = 0;
  dx = static_cast<double> (1.0) / (xq_test.size() - 1);
  for(int i = 1; i < xq_test.size(); i++)
  {
    xq_test(i) = xq_test(i - 1) + dx;
  }

  // get predictions
  VectorXd yp_test = VectorXd::Zero(xq_test.size());
  for(int i = 0; i < xq_test.size(); i++)
  {
    if (!lwr.predict(xq_test(i), yp_test(i)))
    {
      ROS_ERROR("Could not predict from LWR model");
      return false;
    }
  }

  // compute mean squared error
  double mse = 0;
  for(int i = 0; i < xq_test.size(); i++)
  {
    mse += pow(testFunction(xq_test(i)) - yp_test(i), 2);
  }
  if (mse > mse_prediction_error_threshold_)
  {
    ROS_ERROR("MSE of the prediction (%f) is large than the threshold (%f)", mse, mse_prediction_error_threshold_);
    return false;
  }

  MatrixXd basis_function_matrix = MatrixXd::Zero(x_test.size(), num_rfs_);
  if (!lwr.generateBasisFunctionMatrix(x_test, basis_function_matrix))
  {
    ROS_ERROR("Could not get basis function matrix");
    return false;  
  }

  data_directory_name_ = lwr.getPathName();
  std::ofstream outfile;
  // write x_test
  outfile.open(std::string(data_directory_name_ + std::string("x_test.txt")).c_str());
  outfile << x_test;
  outfile.close();

  // write y_test
  outfile.open(std::string(data_directory_name_ + std::string("y_test.txt")).c_str());
  outfile << y_test;
  outfile.close();

  // write xq_test
  outfile.open(std::string(data_directory_name_ + std::string("xq_test.txt")).c_str());
  outfile << xq_test;
  outfile.close();

  // write yp_test
  outfile.open(std::string(data_directory_name_ + std::string("yp_test.txt")).c_str());
  outfile << yp_test;
  outfile.close();

  outfile.open(std::string(data_directory_name_ + std::string("basis_function_matrix.txt")).c_str());
  outfile << basis_function_matrix;
  outfile.close();

  if (!lwr.writeToDisc(data_directory_name_))
  {
    ROS_ERROR("Could not write LWR model to file");
    return false;
  }

  ROS_INFO_STREAM(lwr.getInfoString());

  // create new LWR model and read previous one from disc
  LocallyWeightedRegression lwr_copy;
  if (!lwr_copy.initializeFromDisc(data_directory_name_))
  {
    ROS_ERROR("Could not read LWR model from file");
    return false;
  }

  // get predictions
  VectorXd yp_test_copy = VectorXd::Zero(xq_test.size());
  for(int i = 0; i < xq_test.size(); i++)
  {
    if (!lwr_copy.predict(xq_test(i), yp_test_copy(i)))
    {
       ROS_ERROR("Could not predict from LWR model");
       return false;
    }
  }

  outfile.open(std::string(data_directory_name_ + std::string("yp_test_copy.txt")).c_str());
  outfile << yp_test_copy;
  outfile.close();

  ROS_INFO_STREAM(lwr_copy.getInfoString());

  return true;
}

double LocallyWeightedRegressionTest::testFunction(const double test_x)
{
  return -exp(pow(test_x - 0.5, 2)) * pow(sin(0.5 * test_x), 3);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "lwr_tests");

  ros::NodeHandle node_handle("/lwr_test/test_parameters");
  LocallyWeightedRegressionTest lwr_test(node_handle);
  if (!lwr_test.initialize())
    ROS_FATAL("Could not initialize the LWR test node.");

  if (!lwr_test.runLocallyWeightedRegressionTest())
    ROS_FATAL("Could not run the LWR test.");
}
