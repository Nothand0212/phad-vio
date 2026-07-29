#include "phad/io/dataset/stereo_imu_dataset.hpp"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace phad::io::dataset
{
  namespace internal
  {

    class StereoImuDatasetImpl
    {
    public:
      StereoImuDatasetImpl(
          StereoImuCalibration                calibration,
          std::vector<sensor::ImuMeasurement> imu_measurements,
          std::vector<StereoFrameRef>         stereo_index,
          sensor::PixelType                   left_pixel_type,
          sensor::PixelType                   right_pixel_type )
          : m_calibration( std::move( calibration ) ),
            m_imu_measurements( std::move( imu_measurements ) ),
            m_stereo_index( std::move( stereo_index ) ),
            m_left_pixel_type( left_pixel_type ),
            m_right_pixel_type( right_pixel_type )
      {
      }

      StereoImuCalibration                m_calibration;
      std::vector<sensor::ImuMeasurement> m_imu_measurements;
      std::vector<StereoFrameRef>         m_stereo_index;
      sensor::PixelType                   m_left_pixel_type;
      sensor::PixelType                   m_right_pixel_type;
    };

  }  // namespace internal

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

    DatasetReaderError makeReaderError( const DatasetError& error,
                                        std::size_t         record_number,
                                        common::Timestamp   timestamp )
    {
      std::string       cause = error.cause;
      const std::string path  = error.source_path.string();
      if ( !path.empty() )
      {
        std::size_t position = cause.find( path );
        while ( position != std::string::npos )
        {
          cause.replace( position, path.size(), "<image>" );
          position = cause.find( path, position + 7 );
        }
      }
      const auto code =
          error.code == DatasetErrorCode::kImageFormatMismatch
              ? DatasetReaderErrorCode::kImageFormatMismatch
              : DatasetReaderErrorCode::kImageDecodeFailed;
      return DatasetReaderError{ code, error.sensor_id, timestamp,
                                 record_number, std::move( cause ) };
    }

    DatasetResult<sensor::StereoFrame> decodeStereo(
        const StereoFrameRef&       reference,
        const StereoImuCalibration& calibration,
        sensor::PixelType           left_pixel_type,
        sensor::PixelType           right_pixel_type,
        const std::string&          left_sensor_id,
        const std::string&          right_sensor_id,
        std::size_t                 record_index )
    {
      auto left = decodeImage(
          reference.left_path, left_sensor_id, record_index,
          reference.timestamp, calibration.left.resolution, left_pixel_type );
      if ( !left )
      {
        return left.error();
      }
      auto right = decodeImage(
          reference.right_path, right_sensor_id, record_index,
          reference.timestamp, calibration.right.resolution, right_pixel_type );
      if ( !right )
      {
        return right.error();
      }
      return sensor::StereoFrame{ reference.timestamp,
                                  std::move( left ).value(),
                                  std::move( right ).value() };
    }

  }  // namespace

  StereoImuDataset::StereoImuDataset(
      StereoImuCalibration                calibration,
      std::vector<sensor::ImuMeasurement> imu_measurements,
      std::vector<StereoFrameRef>         stereo_index,
      sensor::PixelType                   left_pixel_type,
      sensor::PixelType                   right_pixel_type )
      : m_impl( std::make_shared<const internal::StereoImuDatasetImpl>(
            std::move( calibration ), std::move( imu_measurements ),
            std::move( stereo_index ), left_pixel_type, right_pixel_type ) )
  {
  }

  StereoImuCalibration StereoImuDataset::calibration() const
  {
    return m_impl->m_calibration;
  }

  StereoImuDatasetSummary StereoImuDataset::summary() const noexcept
  {
    const auto summarize = []( const auto& records ) {
      DatasetStreamSummary result{ records.size(), std::nullopt, std::nullopt };
      if ( !records.empty() )
      {
        result.first_timestamp = records.front().timestamp;
        result.last_timestamp  = records.back().timestamp;
      }
      return result;
    };
    return StereoImuDatasetSummary{ summarize( m_impl->m_imu_measurements ),
                                    summarize( m_impl->m_stereo_index ) };
  }

  StereoImuDatasetReader StereoImuDataset::reader() const
  {
    return StereoImuDatasetReader{ m_impl };
  }

  StereoImuDatasetReader::StereoImuDatasetReader(
      std::shared_ptr<const internal::StereoImuDatasetImpl> impl )
      : m_impl( std::move( impl ) )
  {
  }

  DatasetReaderResult<sensor::ImuMeasurement>
  StereoImuDatasetReader::takeImu()
  {
    if ( m_terminal_error.has_value() )
    {
      return *m_terminal_error;
    }
    if ( m_next_imu_index >= m_impl->m_imu_measurements.size() )
    {
      return DatasetReaderEnd{};
    }
    return m_impl->m_imu_measurements[ m_next_imu_index++ ];
  }

  DatasetReaderResult<common::Timestamp>
  StereoImuDatasetReader::peekStereoTimestamp()
  {
    if ( m_terminal_error.has_value() )
    {
      return *m_terminal_error;
    }
    if ( m_next_stereo_index >= m_impl->m_stereo_index.size() )
    {
      return DatasetReaderEnd{};
    }
    return m_impl->m_stereo_index[ m_next_stereo_index ].timestamp;
  }

  DatasetReaderResult<sensor::StereoFrame>
  StereoImuDatasetReader::takeStereo()
  {
    if ( m_terminal_error.has_value() )
    {
      return *m_terminal_error;
    }
    if ( m_next_stereo_index >= m_impl->m_stereo_index.size() )
    {
      return DatasetReaderEnd{};
    }

    const auto&       reference     = m_impl->m_stereo_index[ m_next_stereo_index ];
    const std::size_t record_number = m_next_stereo_index + 1;
    auto              decoded       = decodeStereo(
        reference, m_impl->m_calibration, m_impl->m_left_pixel_type,
        m_impl->m_right_pixel_type, "left_camera", "right_camera",
        m_next_stereo_index );
    if ( !decoded )
    {
      m_terminal_error =
          makeReaderError( decoded.error(), record_number, reference.timestamp );
      return *m_terminal_error;
    }

    ++m_next_stereo_index;
    return std::move( decoded ).value();
  }

  std::span<const sensor::ImuMeasurement>
  StereoImuDataset::imuMeasurements() const noexcept
  {
    return m_impl->m_imu_measurements;
  }

  std::span<const StereoFrameRef> StereoImuDataset::stereoIndex() const noexcept
  {
    return m_impl->m_stereo_index;
  }

  DatasetResult<sensor::StereoFrame> StereoImuDataset::loadStereo(
      std::size_t index ) const
  {
    if ( index >= m_impl->m_stereo_index.size() )
    {
      return DatasetError{ DatasetErrorCode::kIndexOutOfRange,
                           "cam0/cam1",
                           {},
                           index,
                           std::nullopt,
                           "index",
                           "stereo frame index is out of range" };
    }

    return decodeStereo(
        m_impl->m_stereo_index[ index ], m_impl->m_calibration,
        m_impl->m_left_pixel_type, m_impl->m_right_pixel_type, "cam0", "cam1",
        index );
  }

}  // namespace phad::io::dataset
