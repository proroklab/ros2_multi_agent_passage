#include <fstream>
#include <iostream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "freyja_msgs/msg/reference_state.hpp"
#include "freyja_msgs/msg/current_state.hpp"

#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/Splines>

#define CSV_NCOLS 3

using std::placeholders::_1;

typedef freyja_msgs::msg::ReferenceState RefState;
typedef freyja_msgs::msg::CurrentState CurState;
typedef Eigen::Spline<double, 2> Spline2d;

class ReferenceManager : public rclcpp::Node
{
  Eigen::Matrix<double, Eigen::Dynamic, 2> pepd_;
  Eigen::Matrix<double, Eigen::Dynamic, 2> vevd_;
  Eigen::VectorXd tvec_;
  Eigen::RowVector2d pepd_centre_;
  Eigen::RowVector2d pepd_scaler_;

  Spline2d pepd_spline_;
  int takeoff_recv_;
  bool go_recv_;
  float TMAX_;

  double fixed_pn_, fixed_yaw_;

  public:
    ReferenceManager();

    void load_trajectory( const std::string& );
    void interpolate_trajectory();
    void scale_trajectory();
    void refine_trajectory();

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr init_sub_;
    void initCallback( const std_msgs::msg::UInt8::ConstSharedPtr );

    rclcpp::Subscription<CurState>::SharedPtr cs_sub_;
    void csCallback( const CurState::ConstSharedPtr );

    rclcpp::Publisher<RefState>::SharedPtr refstate_pub_;
    rclcpp::TimerBase::SharedPtr ref_timer_;
    void sendReference();
};

ReferenceManager::ReferenceManager() : Node( "reference_state_node" )
{
  declare_parameter<std::string>( "csv_file", "trajectory.csv" );
  std::string fname;
  get_parameter( "csv_file", fname );

  pepd_centre_ << 0.0, 1.2;
  pepd_scaler_ << 1.8, 1.5;

  fixed_pn_ = -0.5;
  fixed_yaw_ = 1.57;

  TMAX_ = 40.0;
  float trajectory_period = 1.0/30.0;

  load_trajectory( fname );
  scale_trajectory();
  refine_trajectory();

  go_recv_ = false;
  takeoff_recv_ = 0;
  init_sub_ = create_subscription <std_msgs::msg::UInt8> ("start_mission", 1,
                          std::bind( &ReferenceManager::initCallback, this, _1) );
  cs_sub_ = create_subscription <CurState> ("current_state", 1,
                          std::bind( &ReferenceManager::csCallback, this, _1) );

  refstate_pub_ = create_publisher <RefState> ( "reference_state", 1 );
  ref_timer_ = rclcpp::create_timer( this, get_clock(), std::chrono::duration<float>(trajectory_period),
                           std::bind( &ReferenceManager::sendReference, this ) );
}

void ReferenceManager::initCallback( const std_msgs::msg::UInt8::ConstSharedPtr msg )
{
  if( (msg->data == 1 || msg->data == 2) )
    takeoff_recv_ = msg->data;
  if( !go_recv_ && msg->data == 3 )
    go_recv_ = true;
}

void ReferenceManager::csCallback( const CurState::ConstSharedPtr msg )
{
  static bool home_captured = false;
  if( !home_captured )
  {
    fixed_pn_ = msg->state_vector[0];
    fixed_yaw_ = msg->state_vector[8];
    std::cout << "Home captured!" << std::endl;
    home_captured = true;
  }
}
void ReferenceManager::load_trajectory( const std::string &fname )
{
  std::cout << "Looking for file: " << fname << std::endl;
  Eigen::Matrix<double, Eigen::Dynamic, CSV_NCOLS> file_data;

  std::ifstream f( fname );
  std::string csvrow, csvtoken;

  // read the header
  f >> csvrow;
  while( f.good() )
  {
    f >> csvrow;
    std::stringstream csvrow_ss( csvrow );
    std::vector<double> row_vec;
    while( std::getline( csvrow_ss, csvtoken, ',') )
      row_vec.push_back( std::stod(csvtoken) );
    file_data.conservativeResize( file_data.rows()+1, Eigen::NoChange );
    file_data.bottomRows<1>() = Eigen::Map<Eigen::Matrix<double, 1, CSV_NCOLS>>(row_vec.data());
  }

  std::cout << "Done loading trajectory! Snapshot:\n";
  std::cout << file_data.topRows<5>() << std::endl;
  std::cout << "Total points (rows): " << file_data.rows() << std::endl; 

  Eigen::MatrixXd pos_and_time = file_data.rightCols<3>();
  pepd_ = pos_and_time.leftCols<2>();
  tvec_ = pos_and_time.rightCols<1>();
  vevd_ = pepd_;
  //tvec_.setLinSpaced( pepd_.rows(), 0, TMAX_ );
}

void ReferenceManager::interpolate_trajectory()
{
  //pepd_spline_ = Eigen::SplineFitting<Spline2d>::Interpolate( pepd_, 2 );
  //std::cout << "Done interpolating. Random sample: " << pepd_spline_ (0.5) << std::endl;
}

void ReferenceManager::scale_trajectory()
{
  // adjust drawing to world scale
  Eigen::RowVector2d half_point;
  half_point << 0.5, 0.5;
  pepd_ = pepd_.rowwise() - half_point;                     // shift drawing [0..1] to [-0.5..0.5]
  pepd_ = pepd_.array().rowwise() * pepd_scaler_.array();   // scale axes separately
  pepd_ = pepd_.rowwise() + pepd_centre_;                   // recenter around some point

  // adjust time to actual time
  tvec_ *= TMAX_;
}

void ReferenceManager::refine_trajectory()
{
  /* Construct velocity components from trajectory */
  Eigen::RowVectorXd fcoeff(5);
  fcoeff << 0.11, 0.15, 0.4, 0.22, 0.12 ;
  //fcoeff << 0.2, 0.2, 0.2, 0.2, 0.2 ;
  int nrows = pepd_.rows();
  Eigen::VectorXd ve(nrows), vd(nrows);
  Eigen::VectorXd dt(nrows);
  dt << tvec_.tail(nrows-1) - tvec_.head(nrows-1), 1.0;
  if( (dt.array() < 0).any() )
    std::cout << "[ERROR]: tvec is not monotonically increasing." << std::endl;
  dt = dt.cwiseMax( 0.01 );
  std::cout << "Sample dt: " << dt.head<10>().transpose() << std::endl;
  
  ve << pepd_.col(0).tail(nrows-1) - pepd_.col(0).head(nrows-1), 0.0;   // numeric diff
  ve = ve.cwiseQuotient( dt ).cwiseMin(2.0).cwiseMax(-2.0);             // divide and clip limits
  ve[0] = fcoeff[0]*ve[0] + fcoeff[1]*ve[1];
  for( int idx = 2; idx < nrows-2; idx++ )
    ve[idx] = fcoeff * ve.segment<5>(idx-2);                            // smoothe

  vd << pepd_.col(1).tail(nrows-1) - pepd_.col(1).head(nrows-1), 0.0;   // numeric diff
  vd = vd.cwiseQuotient( dt ).cwiseMin(1.0).cwiseMax(-1.0);             // divide and clip limits
  vd[0] = fcoeff[0]*vd[0] + fcoeff[1]*vd[1];
  for( int idx = 2; idx < nrows-2; idx++ )
    vd[idx] = fcoeff * vd.segment<5>(idx-2);                            // smoothe

  vevd_.col(0) = ve;
  vevd_.col(1) = vd;
  std::cout << "Done refining velocities!" << std::endl;
  std::cout << "Sample ve: " << ve.head<10>().transpose() << std::endl;
  std::cout << "Sample vd: " << vd.head<10>().transpose() << std::endl;
}

void ReferenceManager::sendReference()
{
  // called on a timer

  static RefState rs;
  
  if( takeoff_recv_ == 0 )
    return;

  if( !go_recv_ && takeoff_recv_ == 1 )
  {
    rs.pd -= 0.005;
    if( rs.pd < -1.2 )
      rs.pd = -1.2;
  }
  else if( !go_recv_ && takeoff_recv_ == 2 )
  {
    // hover at first point
    rs.pe = pepd_(0,0);
    rs.pd = -pepd_(0,1);
    RCLCPP_INFO_THROTTLE( get_logger(), *(get_clock()), 1200, "Waiting at first point!" );
  }
  else
  {
    static rclcpp::Time t_init = now();
    rclcpp::Time tnow = now();
    double t = (tnow - t_init).seconds();
  
    if( t < TMAX_ + 2.0 )
    {
      int idx = -1;
      (tvec_.array() - t).abs().minCoeff(&idx);
      rs.pe = pepd_(idx, 0);
      rs.pd = -pepd_(idx, 1);
      rs.ve = vevd_(idx, 0);
      rs.vd = -vevd_(idx, 1);
    }
    else
    {
      // land
      rs.pd += 0.01;
    }
  }
  // pn is fixed
  rs.pn = fixed_pn_;
  rs.yaw = fixed_yaw_;
  refstate_pub_ -> publish( rs );
}

int main( int argc, char** argv )
{
  rclcpp::init( argc, argv );
  rclcpp::spin( std::make_shared<ReferenceManager>() );
  rclcpp::shutdown();
  return 0;
}
