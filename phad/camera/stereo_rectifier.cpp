#include "phad/camera/stereo_rectifier.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <utility>
#include <variant>
#include <vector>

namespace phad::camera
{
  namespace
  {

    [[nodiscard]] CameraModelError makeError( CameraModelErrorCode code,
                                              std::string          detail )
    {
      return CameraModelError{ code, std::move( detail ) };
    }

    [[nodiscard]] cv::Mat cameraMatrix(
        const sensor::PinholeRadialTangentialParameters& model )
    {
      cv::Mat K = cv::Mat::eye( 3, 3, CV_64F );
      K.at<double>( 0, 0 ) = model.fxPixels();
      K.at<double>( 1, 1 ) = model.fyPixels();
      K.at<double>( 0, 2 ) = model.cxPixels();
      K.at<double>( 1, 2 ) = model.cyPixels();
      return K;
    }

    [[nodiscard]] cv::Mat distCoeffs(
        const sensor::PinholeRadialTangentialParameters& model )
    {
      return ( cv::Mat_<double>( 4, 1 ) << model.k1(), model.k2(), model.p1(),
               model.p2() );
    }

    [[nodiscard]] bool isGrayUint8( const sensor::Image& image )
    {
      return image.width() > 0 && image.height() > 0 &&
             image.channels() == 1 &&
             image.pixelType() == sensor::PixelType::kUint8;
    }

    [[nodiscard]] CameraModelResult<cv::Mat> toGrayMat(
        const sensor::Image& image )
    {
      if ( !isGrayUint8( image ) )
      {
        return makeError(
            CameraModelErrorCode::kOutsideModelDomain,
            "stereo rectify expects single-channel uint8 images" );
      }
      const auto pixels = image.pixels<std::uint8_t>();
      if ( !pixels.has_value() )
      {
        return makeError( CameraModelErrorCode::kOutsideModelDomain,
                          "uint8 image does not expose uint8 pixels" );
      }
      cv::Mat mat( image.height(), image.width(), CV_8UC1 );
      if ( pixels->size() != mat.total() )
      {
        return makeError(
            CameraModelErrorCode::kOutsideModelDomain,
            "image metadata does not match pixel count" );
      }
      std::copy( pixels->begin(), pixels->end(), mat.ptr<std::uint8_t>() );
      return mat;
    }

    [[nodiscard]] sensor::Image fromGrayMat( const cv::Mat& mat )
    {
      const std::size_t count = mat.total();
      std::vector<std::uint8_t> pixels( count );
      if ( mat.isContinuous() )
      {
        std::copy( mat.ptr<std::uint8_t>(),
                   mat.ptr<std::uint8_t>() + count, pixels.begin() );
      }
      else
      {
        for ( int row = 0; row < mat.rows; ++row )
        {
          const auto* begin = mat.ptr<std::uint8_t>( row );
          const auto offset = static_cast<std::ptrdiff_t>( row ) *
                              static_cast<std::ptrdiff_t>( mat.cols );
          std::copy( begin, begin + mat.cols, pixels.begin() + offset );
        }
      }
      return sensor::Image{ mat.cols, mat.rows, 1, std::move( pixels ) };
    }

  }  // namespace

  struct StereoRectifier::Impl
  {
    Impl( RectifiedStereoCalibration rectified_calibration, int width,
          int height )
        : calibration( std::move( rectified_calibration ) ),
          expected_width( width ),
          expected_height( height )
    {
    }

    RectifiedStereoCalibration calibration;
    cv::Mat                    map1_left;
    cv::Mat                    map2_left;
    cv::Mat                    map1_right;
    cv::Mat                    map2_right;
    int                        expected_width;
    int                        expected_height;
  };

  CameraModelResult<StereoRectifier> StereoRectifier::create(
      const sensor::StereoImuCalibration& calibration )
  {
    const auto& left_model  = calibration.leftCamera().modelParameters();
    const auto& right_model = calibration.rightCamera().modelParameters();
    if ( !std::holds_alternative<sensor::PinholeRadialTangentialParameters>(
             left_model ) ||
         !std::holds_alternative<sensor::PinholeRadialTangentialParameters>(
             right_model ) )
    {
      return makeError(
          CameraModelErrorCode::kOutsideModelDomain,
          "stereo rectify supports only radial-tangential models; "
          "equidistant is deferred" );
    }

    const auto& left =
        std::get<sensor::PinholeRadialTangentialParameters>( left_model );
    const auto& right =
        std::get<sensor::PinholeRadialTangentialParameters>( right_model );

    const int width = static_cast<int>( calibration.leftCamera().imageWidth() );
    const int height =
        static_cast<int>( calibration.leftCamera().imageHeight() );
    if ( width <= 0 || height <= 0 ||
         calibration.rightCamera().imageWidth() !=
             calibration.leftCamera().imageWidth() ||
         calibration.rightCamera().imageHeight() !=
             calibration.leftCamera().imageHeight() )
    {
      return makeError(
          CameraModelErrorCode::kOutsideModelDomain,
          "left and right cameras must share a positive image size" );
    }

    const Eigen::Matrix3d R_B_left =
        calibration.T_B_left_camera().rotation();
    const Eigen::Vector3d t_B_left =
        calibration.T_B_left_camera().translation();
    const Eigen::Matrix3d R_B_right =
        calibration.T_B_right_camera().rotation();
    const Eigen::Vector3d t_B_right =
        calibration.T_B_right_camera().translation();

    // OpenCV stereoRectify expects R,T mapping left-camera points into the
    // right-camera frame: p_right = R * p_left + T.
    const Eigen::Matrix3d R_right_left =
        R_B_right.transpose() * R_B_left;
    const Eigen::Vector3d t_right_left =
        R_B_right.transpose() * ( t_B_left - t_B_right );

    cv::Mat R( 3, 3, CV_64F );
    cv::Mat T( 3, 1, CV_64F );
    for ( int row = 0; row < 3; ++row )
    {
      T.at<double>( row, 0 ) = t_right_left( row );
      for ( int col = 0; col < 3; ++col )
      {
        R.at<double>( row, col ) = R_right_left( row, col );
      }
    }

    cv::Mat R1;
    cv::Mat R2;
    cv::Mat P1;
    cv::Mat P2;
    cv::Mat Q;
    try
    {
      cv::stereoRectify( cameraMatrix( left ), distCoeffs( left ),
                         cameraMatrix( right ), distCoeffs( right ),
                         cv::Size( width, height ), R, T, R1, R2, P1, P2, Q,
                         cv::CALIB_ZERO_DISPARITY, 0.0,
                         cv::Size( width, height ) );
    }
    catch ( const cv::Exception& exception )
    {
      return makeError( CameraModelErrorCode::kNumericalFailure,
                        exception.what() );
    }

    const double fx        = P1.at<double>( 0, 0 );
    const double fy        = P1.at<double>( 1, 1 );
    const double cx        = P1.at<double>( 0, 2 );
    const double cy        = P1.at<double>( 1, 2 );
    const double baseline_m =
        std::abs( P2.at<double>( 0, 3 ) / P2.at<double>( 0, 0 ) );
    if ( !std::isfinite( fx ) || !std::isfinite( fy ) ||
         !std::isfinite( cx ) || !std::isfinite( cy ) ||
         !std::isfinite( baseline_m ) || baseline_m <= 0.0 )
    {
      return makeError(
          CameraModelErrorCode::kNumericalFailure,
          "stereoRectify produced non-finite or non-positive calibration" );
    }

    // R1 maps unrectified left points into the rectified left frame.
    // T_left_left_rect has R = R1^T, t = 0, so
    // T_B_left_rectified = T_B_left * T_left_left_rect.
    Eigen::Matrix3d R1_eigen;
    for ( int row = 0; row < 3; ++row )
    {
      for ( int col = 0; col < 3; ++col )
      {
        R1_eigen( row, col ) = R1.at<double>( row, col );
      }
    }
    Eigen::Matrix4d T_B_left_rect_matrix = Eigen::Matrix4d::Identity();
    T_B_left_rect_matrix.block<3, 3>( 0, 0 ) = R_B_left * R1_eigen.transpose();
    T_B_left_rect_matrix.block<3, 1>( 0, 3 ) = t_B_left;

    auto T_B_left_rectified =
        sensor::RigidTransform::create( T_B_left_rect_matrix );
    if ( !T_B_left_rectified )
    {
      return makeError(
          CameraModelErrorCode::kNumericalFailure,
          "failed to form T_B_left_rectified from stereoRectify R1" );
    }

    auto rectified = RectifiedStereoCalibration::create(
        fx, fy, cx, cy, baseline_m, width, height,
        std::move( T_B_left_rectified ).value() );
    if ( !rectified )
    {
      return makeError(
          CameraModelErrorCode::kNumericalFailure,
          "failed to build RectifiedStereoCalibration from stereoRectify" );
    }

    auto impl = std::make_unique<Impl>( std::move( rectified ).value(), width,
                                        height );
    try
    {
      cv::initUndistortRectifyMap(
          cameraMatrix( left ), distCoeffs( left ), R1, P1,
          cv::Size( width, height ), CV_16SC2, impl->map1_left,
          impl->map2_left );
      cv::initUndistortRectifyMap(
          cameraMatrix( right ), distCoeffs( right ), R2, P2,
          cv::Size( width, height ), CV_16SC2, impl->map1_right,
          impl->map2_right );
    }
    catch ( const cv::Exception& exception )
    {
      return makeError( CameraModelErrorCode::kNumericalFailure,
                        exception.what() );
    }

    return StereoRectifier( std::move( impl ) );
  }

  StereoRectifier::StereoRectifier( std::unique_ptr<Impl> impl )
      : m_impl( std::move( impl ) )
  {
  }

  StereoRectifier::~StereoRectifier() = default;

  StereoRectifier::StereoRectifier( StereoRectifier&& ) noexcept = default;

  StereoRectifier& StereoRectifier::operator=( StereoRectifier&& ) noexcept =
      default;

  const RectifiedStereoCalibration& StereoRectifier::calibration()
      const noexcept
  {
    return m_impl->calibration;
  }

  CameraModelResult<sensor::StereoFrame> StereoRectifier::rectify(
      const sensor::StereoFrame& raw_frame ) const
  {
    if ( raw_frame.left.width() != m_impl->expected_width ||
         raw_frame.left.height() != m_impl->expected_height ||
         raw_frame.right.width() != m_impl->expected_width ||
         raw_frame.right.height() != m_impl->expected_height )
    {
      return makeError(
          CameraModelErrorCode::kOutsideModelDomain,
          "raw stereo frame size does not match rectifier calibration" );
    }

    auto left_mat = toGrayMat( raw_frame.left );
    if ( !left_mat )
    {
      return left_mat.error();
    }
    auto right_mat = toGrayMat( raw_frame.right );
    if ( !right_mat )
    {
      return right_mat.error();
    }

    cv::Mat left_rectified;
    cv::Mat right_rectified;
    try
    {
      cv::remap( left_mat.value(), left_rectified, m_impl->map1_left,
                 m_impl->map2_left, cv::INTER_LINEAR, cv::BORDER_CONSTANT );
      cv::remap( right_mat.value(), right_rectified, m_impl->map1_right,
                 m_impl->map2_right, cv::INTER_LINEAR, cv::BORDER_CONSTANT );
    }
    catch ( const cv::Exception& exception )
    {
      return makeError( CameraModelErrorCode::kNumericalFailure,
                        exception.what() );
    }

    return sensor::StereoFrame{ raw_frame.timestamp,
                                fromGrayMat( left_rectified ),
                                fromGrayMat( right_rectified ) };
  }

}  // namespace phad::camera
