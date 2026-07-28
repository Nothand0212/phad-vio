#include "phad/io/dataset/stereo_imu_dataset.hpp"

#include <cstdint>
#include <filesystem>
#include <opencv2/imgcodecs.hpp>
#include <string>
#include <utility>
#include <vector>

namespace phad::io::dataset
{
  namespace
  {

    namespace fs = std::filesystem;

    DatasetError makeError( DatasetErrorCode code, std::string sensor_id,
                            fs::path source_path, std::string field,
                            std::string cause, std::size_t index,
                            common::Timestamp timestamp )
    {
      return DatasetError{ code, std::move( sensor_id ), std::move( source_path ),
                           index, timestamp, std::move( field ),
                           std::move( cause ) };
    }

    template <typename Pixel>
    sensor::Image copyImage( const cv::Mat& decoded )
    {
      const std::size_t row_pixels =
          static_cast<std::size_t>( decoded.cols * decoded.channels() );
      std::vector<Pixel> pixels;
      pixels.reserve( row_pixels * static_cast<std::size_t>( decoded.rows ) );
      for ( int row = 0; row < decoded.rows; ++row )
      {
        const auto* begin = decoded.ptr<Pixel>( row );
        pixels.insert( pixels.end(), begin, begin + row_pixels );
      }
      return sensor::Image{ decoded.cols, decoded.rows, decoded.channels(),
                            std::move( pixels ) };
    }

    DatasetResult<sensor::Image> decodeImage(
        const fs::path& path, const std::string& sensor_id, std::size_t index,
        common::Timestamp timestamp, const sensor::ImageResolution& expected,
        sensor::PixelType expected_pixel_type )
    {
      cv::Mat decoded;
      try
      {
        decoded = cv::imread( path.string(), cv::IMREAD_UNCHANGED );
      }
      catch ( const cv::Exception& exception )
      {
        return makeError( DatasetErrorCode::kImageDecodeFailed, sensor_id, path,
                          "image", exception.what(), index, timestamp );
      }
      if ( decoded.empty() )
      {
        return makeError( DatasetErrorCode::kImageDecodeFailed, sensor_id, path,
                          "image", "OpenCV could not decode the image", index,
                          timestamp );
      }
      if ( decoded.cols != expected.width || decoded.rows != expected.height )
      {
        return makeError(
            DatasetErrorCode::kImageFormatMismatch, sensor_id, path, "resolution",
            "decoded dimensions do not match sensor calibration", index, timestamp );
      }
      if ( expected_pixel_type == sensor::PixelType::kUint8 )
      {
        if ( decoded.type() != CV_8UC1 )
        {
          return makeError( DatasetErrorCode::kImageFormatMismatch, sensor_id, path,
                            "pixel_type", "decoded image must be uint8 grayscale",
                            index, timestamp );
        }
        return copyImage<std::uint8_t>( decoded );
      }
      if ( decoded.type() != CV_16UC1 )
      {
        return makeError( DatasetErrorCode::kImageFormatMismatch, sensor_id, path,
                          "pixel_type", "decoded image must be uint16 grayscale",
                          index, timestamp );
      }
      return copyImage<std::uint16_t>( decoded );
    }

  }  // namespace

  DatasetResult<sensor::StereoFrame> StereoImuDataset::loadStereo(
      std::size_t index ) const
  {
    if ( index >= m_stereo_index.size() )
    {
      return DatasetError{ DatasetErrorCode::kIndexOutOfRange,
                           "cam0/cam1",
                           {},
                           index,
                           std::nullopt,
                           "index",
                           "stereo frame index is out of range" };
    }

    const auto& reference = m_stereo_index[ index ];
    auto        left      = decodeImage( reference.left_path, "cam0", index,
                                         reference.timestamp, m_calibration.left.resolution,
                                         m_left_pixel_type );
    if ( !left )
    {
      return left.error();
    }
    auto right = decodeImage( reference.right_path, "cam1", index,
                              reference.timestamp, m_calibration.right.resolution,
                              m_right_pixel_type );
    if ( !right )
    {
      return right.error();
    }
    return sensor::StereoFrame{ reference.timestamp, std::move( left ).value(),
                                std::move( right ).value() };
  }

}  // namespace phad::io::dataset
