#include "phad/camera/stereo_rectifier.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <sstream>
#include <string_view>
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

    [[nodiscard]] CameraModelError makeCalibrationCreateError(
        const char*                     stage,
        const sensor::CalibrationError& calibration_error )
    {
      std::ostringstream detail;
      detail << "create/calibration stage " << stage << " failed at "
             << calibration_error.field_path << " (" << calibration_error.detail
             << ')';
      return makeError( CameraModelErrorCode::kNumericalFailure,
                        detail.str() );
    }

    [[nodiscard]] cv::Mat cameraMatrix(
        const sensor::PinholeRadialTangentialParameters& model )
    {
      cv::Mat K            = cv::Mat::eye( 3, 3, CV_64F );
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

    [[nodiscard]] cv::Mat cameraMatrix(
        const sensor::PinholeEquidistantParameters& model )
    {
      cv::Mat K            = cv::Mat::eye( 3, 3, CV_64F );
      K.at<double>( 0, 0 ) = model.fxPixels();
      K.at<double>( 1, 1 ) = model.fyPixels();
      K.at<double>( 0, 2 ) = model.cxPixels();
      K.at<double>( 1, 2 ) = model.cyPixels();
      return K;
    }

    [[nodiscard]] cv::Mat distCoeffs(
        const sensor::PinholeEquidistantParameters& model )
    {
      return ( cv::Mat_<double>( 4, 1 ) << model.k1(), model.k2(), model.k3(),
               model.k4() );
    }

    [[nodiscard]] std::string_view modelName(
        const sensor::CameraModelParameters& model )
    {
      if ( std::holds_alternative<
               sensor::PinholeRadialTangentialParameters>( model ) )
      {
        return "radtan";
      }
      return "equidistant";
    }

    [[nodiscard]] std::string_view pixelTypeName(
        sensor::PixelType pixel_type )
    {
      switch ( pixel_type )
      {
        case sensor::PixelType::kUint8:
          return "uint8";
        case sensor::PixelType::kUint16:
          return "uint16";
      }
      return "unknown";
    }

    [[nodiscard]] std::optional<CameraModelError> validateImage(
        const sensor::Image& image, std::string_view side, int expected_width,
        int expected_height )
    {
      if ( image.width() != expected_width ||
           image.height() != expected_height )
      {
        std::ostringstream detail;
        detail << "rectify validate stage " << side << " size expected "
               << expected_width << 'x' << expected_height << " actual "
               << image.width() << 'x' << image.height();
        return makeError( CameraModelErrorCode::kOutsideModelDomain,
                          detail.str() );
      }
      if ( image.channels() != 1 )
      {
        std::ostringstream detail;
        detail << "rectify validate stage " << side
               << " channels expected 1 actual " << image.channels();
        return makeError( CameraModelErrorCode::kOutsideModelDomain,
                          detail.str() );
      }

      const std::size_t expected_count =
          static_cast<std::size_t>( expected_width ) *
          static_cast<std::size_t>( expected_height );
      std::size_t actual_count = 0;
      switch ( image.pixelType() )
      {
        case sensor::PixelType::kUint8:
        {
          const auto pixels = image.pixels<std::uint8_t>();
          actual_count      = pixels.has_value() ? pixels->size() : 0;
          break;
        }
        case sensor::PixelType::kUint16:
        {
          const auto pixels = image.pixels<std::uint16_t>();
          actual_count      = pixels.has_value() ? pixels->size() : 0;
          break;
        }
      }
      if ( actual_count != expected_count )
      {
        std::ostringstream detail;
        detail << "rectify validate stage " << side << ' '
               << pixelTypeName( image.pixelType() )
               << " pixel count expected " << expected_count << " actual "
               << actual_count;
        return makeError( CameraModelErrorCode::kOutsideModelDomain,
                          detail.str() );
      }
      return std::nullopt;
    }

    [[nodiscard]] CameraModelResult<cv::Mat> toGrayMat(
        const sensor::Image& image, std::string_view side )
    {
      try
      {
        if ( image.pixelType() == sensor::PixelType::kUint8 )
        {
          const auto pixels = image.pixels<std::uint8_t>();
          cv::Mat    mat( image.height(), image.width(), CV_8UC1 );
          std::copy( pixels->begin(), pixels->end(), mat.ptr<std::uint8_t>() );
          return mat;
        }

        const auto pixels = image.pixels<std::uint16_t>();
        cv::Mat    mat( image.height(), image.width(), CV_16UC1 );
        std::copy( pixels->begin(), pixels->end(), mat.ptr<std::uint16_t>() );
        return mat;
      }
      catch ( const cv::Exception& exception )
      {
        return makeError(
            CameraModelErrorCode::kNumericalFailure,
            "rectify input-mat stage " + std::string( side ) +
                " OpenCV cause: " + std::string( exception.what() ) );
      }
    }

    [[nodiscard]] CameraModelResult<sensor::Image> fromRectifiedGrayMat(
        const cv::Mat& mat, std::string_view side )
    {
      try
      {
        const std::size_t         count = mat.total();
        std::vector<std::uint8_t> pixels( count );
        if ( mat.type() == CV_8UC1 )
        {
          for ( int row = 0; row < mat.rows; ++row )
          {
            const auto* source = mat.ptr<std::uint8_t>( row );
            for ( int col = 0; col < mat.cols; ++col )
            {
              pixels[ static_cast<std::size_t>( row * mat.cols + col ) ] =
                  source[ col ];
            }
          }
        }
        else if ( mat.type() == CV_16UC1 )
        {
          for ( int row = 0; row < mat.rows; ++row )
          {
            const auto* source = mat.ptr<std::uint16_t>( row );
            const auto  offset = static_cast<std::size_t>( row ) *
                                static_cast<std::size_t>( mat.cols );
            for ( int col = 0; col < mat.cols; ++col )
            {
              pixels[ offset + static_cast<std::size_t>( col ) ] =
                  static_cast<std::uint8_t>( source[ col ] >> 8 );
            }
          }
        }
        else
        {
          std::ostringstream detail;
          detail << "rectify output stage " << side
                 << " expected CV_8UC1 or CV_16UC1 actual OpenCV type "
                 << mat.type();
          return makeError( CameraModelErrorCode::kNumericalFailure,
                            detail.str() );
        }
        return sensor::Image{ mat.cols, mat.rows, 1, std::move( pixels ) };
      }
      catch ( const cv::Exception& exception )
      {
        return makeError(
            CameraModelErrorCode::kNumericalFailure,
            "rectify output stage " + std::string( side ) +
                " OpenCV cause: " + std::string( exception.what() ) );
      }
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
    if ( left_model.index() != right_model.index() )
    {
      std::ostringstream detail;
      detail << "create model pairing expected matching models actual left "
             << modelName( left_model ) << " right "
             << modelName( right_model );
      return makeError( CameraModelErrorCode::kOutsideModelDomain,
                        detail.str() );
    }

    const auto left_width   = calibration.leftCamera().imageWidth();
    const auto left_height  = calibration.leftCamera().imageHeight();
    const auto right_width  = calibration.rightCamera().imageWidth();
    const auto right_height = calibration.rightCamera().imageHeight();
    if ( left_width == 0U || left_height == 0U ||
         left_width > static_cast<std::uint32_t>(
                          std::numeric_limits<int>::max() ) ||
         left_height > static_cast<std::uint32_t>(
                           std::numeric_limits<int>::max() ) ||
         left_width != right_width || left_height != right_height )
    {
      std::ostringstream detail;
      detail << "create calibrated size expected matching positive int size "
             << left_width << 'x' << left_height << " actual right "
             << right_width << 'x' << right_height;
      return makeError( CameraModelErrorCode::kOutsideModelDomain,
                        detail.str() );
    }
    const int width  = static_cast<int>( left_width );
    const int height = static_cast<int>( left_height );

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

    cv::Mat    R;
    cv::Mat    T;
    cv::Mat    R1;
    cv::Mat    R2;
    cv::Mat    P1;
    cv::Mat    P2;
    cv::Mat    Q;
    cv::Mat    K_left;
    cv::Mat    D_left;
    cv::Mat    K_right;
    cv::Mat    D_right;
    const bool is_radtan =
        std::holds_alternative<sensor::PinholeRadialTangentialParameters>(
            left_model );
    try
    {
      R = cv::Mat( 3, 3, CV_64F );
      T = cv::Mat( 3, 1, CV_64F );
      for ( int row = 0; row < 3; ++row )
      {
        T.at<double>( row, 0 ) = t_right_left( row );
        for ( int col = 0; col < 3; ++col )
        {
          R.at<double>( row, col ) = R_right_left( row, col );
        }
      }
      if ( is_radtan )
      {
        const auto& left =
            std::get<sensor::PinholeRadialTangentialParameters>( left_model );
        const auto& right =
            std::get<sensor::PinholeRadialTangentialParameters>( right_model );
        K_left  = cameraMatrix( left );
        D_left  = distCoeffs( left );
        K_right = cameraMatrix( right );
        D_right = distCoeffs( right );
        cv::stereoRectify( K_left, D_left, K_right, D_right,
                           cv::Size( width, height ), R, T, R1, R2, P1, P2, Q,
                           cv::CALIB_ZERO_DISPARITY, 0.0,
                           cv::Size( width, height ) );
      }
      else
      {
        const auto& left =
            std::get<sensor::PinholeEquidistantParameters>( left_model );
        const auto& right =
            std::get<sensor::PinholeEquidistantParameters>( right_model );
        K_left  = cameraMatrix( left );
        D_left  = distCoeffs( left );
        K_right = cameraMatrix( right );
        D_right = distCoeffs( right );
        cv::fisheye::stereoRectify(
            K_left, D_left, K_right, D_right, cv::Size( width, height ), R, T,
            R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY,
            cv::Size( width, height ), 0.0, 1.0 );
      }
    }
    catch ( const cv::Exception& exception )
    {
      return makeError(
          CameraModelErrorCode::kNumericalFailure,
          "create stereoRectify stage OpenCV cause: " +
              std::string( exception.what() ) );
    }

    double          fx         = 0.0;
    double          fy         = 0.0;
    double          cx         = 0.0;
    double          cy         = 0.0;
    double          baseline_m = 0.0;
    Eigen::Matrix3d R1_eigen;
    try
    {
      fx = P1.at<double>( 0, 0 );
      fy = P1.at<double>( 1, 1 );
      cx = P1.at<double>( 0, 2 );
      cy = P1.at<double>( 1, 2 );
      baseline_m =
          std::abs( P2.at<double>( 0, 3 ) / P2.at<double>( 0, 0 ) );
      for ( int row = 0; row < 3; ++row )
      {
        for ( int col = 0; col < 3; ++col )
        {
          R1_eigen( row, col ) = R1.at<double>( row, col );
        }
      }
    }
    catch ( const cv::Exception& exception )
    {
      return makeError(
          CameraModelErrorCode::kNumericalFailure,
          "create calibration stage OpenCV cause: " +
              std::string( exception.what() ) );
    }
    if ( !std::isfinite( fx ) || !std::isfinite( fy ) ||
         !std::isfinite( cx ) || !std::isfinite( cy ) ||
         !std::isfinite( baseline_m ) )
    {
      return makeError(
          CameraModelErrorCode::kNumericalFailure,
          "create stereoRectify stage produced non-finite or non-positive "
          "calibration" );
    }
    if ( fx <= 0.0 || fy <= 0.0 )
    {
      return makeError(
          CameraModelErrorCode::kNumericalFailure,
          "create/calibration stage produced non-positive fx/fy" );
    }
    if ( baseline_m <= 0.0 )
    {
      return makeError(
          CameraModelErrorCode::kNumericalFailure,
          "create/calibration stage produced non-positive baseline" );
    }

    // R1 maps unrectified left points into the rectified left frame.
    // T_left_left_rect has R = R1^T, t = 0, so
    // T_B_left_rectified = T_B_left * T_left_left_rect.
    Eigen::Matrix4d T_B_left_rect_matrix     = Eigen::Matrix4d::Identity();
    T_B_left_rect_matrix.block<3, 3>( 0, 0 ) = R_B_left * R1_eigen.transpose();
    T_B_left_rect_matrix.block<3, 1>( 0, 3 ) = t_B_left;

    auto T_B_left_rectified =
        sensor::RigidTransform::create( T_B_left_rect_matrix );
    if ( !T_B_left_rectified )
    {
      return makeCalibrationCreateError( "RigidTransform::create",
                                         T_B_left_rectified.error() );
    }

    auto rectified = RectifiedStereoCalibration::create(
        fx, fy, cx, cy, baseline_m, width, height,
        std::move( T_B_left_rectified ).value() );
    if ( !rectified )
    {
      return makeCalibrationCreateError( "RectifiedStereoCalibration::create",
                                         rectified.error() );
    }

    auto impl = std::make_unique<Impl>( std::move( rectified ).value(), width,
                                        height );
    try
    {
      if ( is_radtan )
      {
        cv::initUndistortRectifyMap(
            K_left, D_left, R1, P1, cv::Size( width, height ), CV_16SC2,
            impl->map1_left, impl->map2_left );
      }
      else
      {
        cv::fisheye::initUndistortRectifyMap(
            K_left, D_left, R1, P1, cv::Size( width, height ), CV_16SC2,
            impl->map1_left, impl->map2_left );
      }
    }
    catch ( const cv::Exception& exception )
    {
      return makeError(
          CameraModelErrorCode::kNumericalFailure,
          "create rectification-map stage left OpenCV cause: " +
              std::string( exception.what() ) );
    }
    try
    {
      if ( is_radtan )
      {
        cv::initUndistortRectifyMap(
            K_right, D_right, R2, P2, cv::Size( width, height ), CV_16SC2,
            impl->map1_right, impl->map2_right );
      }
      else
      {
        cv::fisheye::initUndistortRectifyMap(
            K_right, D_right, R2, P2, cv::Size( width, height ), CV_16SC2,
            impl->map1_right, impl->map2_right );
      }
    }
    catch ( const cv::Exception& exception )
    {
      return makeError(
          CameraModelErrorCode::kNumericalFailure,
          "create rectification-map stage right OpenCV cause: " +
              std::string( exception.what() ) );
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
    if ( auto error = validateImage( raw_frame.left, "left",
                                     m_impl->expected_width,
                                     m_impl->expected_height ) )
    {
      return std::move( *error );
    }
    if ( auto error = validateImage( raw_frame.right, "right",
                                     m_impl->expected_width,
                                     m_impl->expected_height ) )
    {
      return std::move( *error );
    }
    if ( raw_frame.left.pixelType() != raw_frame.right.pixelType() )
    {
      std::ostringstream detail;
      detail << "rectify validate stage right pixel type expected "
             << pixelTypeName( raw_frame.left.pixelType() ) << " actual "
             << pixelTypeName( raw_frame.right.pixelType() );
      return makeError( CameraModelErrorCode::kOutsideModelDomain,
                        detail.str() );
    }

    auto left_mat = toGrayMat( raw_frame.left, "left" );
    if ( !left_mat )
    {
      return left_mat.error();
    }
    auto right_mat = toGrayMat( raw_frame.right, "right" );
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
    }
    catch ( const cv::Exception& exception )
    {
      return makeError(
          CameraModelErrorCode::kNumericalFailure,
          "rectify remap stage left OpenCV cause: " +
              std::string( exception.what() ) );
    }
    try
    {
      cv::remap( right_mat.value(), right_rectified, m_impl->map1_right,
                 m_impl->map2_right, cv::INTER_LINEAR, cv::BORDER_CONSTANT );
    }
    catch ( const cv::Exception& exception )
    {
      return makeError(
          CameraModelErrorCode::kNumericalFailure,
          "rectify remap stage right OpenCV cause: " +
              std::string( exception.what() ) );
    }

    auto left_output = fromRectifiedGrayMat( left_rectified, "left" );
    if ( !left_output )
    {
      return left_output.error();
    }
    auto right_output = fromRectifiedGrayMat( right_rectified, "right" );
    if ( !right_output )
    {
      return right_output.error();
    }

    return sensor::StereoFrame{
        raw_frame.timestamp, std::move( left_output ).value(),
        std::move( right_output ).value() };
  }

}  // namespace phad::camera
