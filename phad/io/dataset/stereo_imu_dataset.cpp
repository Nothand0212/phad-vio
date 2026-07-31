#include "phad/io/dataset/stereo_imu_dataset.hpp"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "phad/io/dataset/dataset_error.hpp"
#include "phad/io/dataset/internal/stereo_imu_dataset_builder.hpp"

namespace phad::io::dataset
{
  namespace internal
  {

    class StereoImuDatasetImpl
    {
    public:
      StereoImuDatasetImpl(
          sensor::StereoImuCalibration         calibration,
          std::vector<sensor::ImuMeasurement>  imu_measurements,
          std::vector<ImageFrameManifestEntry> left_manifest,
          std::vector<ImageFrameManifestEntry> right_manifest,
          sensor::PixelType                    left_pixel_type,
          sensor::PixelType                    right_pixel_type )
          : m_calibration( std::move( calibration ) ),
            m_imu_measurements( std::move( imu_measurements ) ),
            m_left_manifest( std::move( left_manifest ) ),
            m_right_manifest( std::move( right_manifest ) ),
            m_left_pixel_type( left_pixel_type ),
            m_right_pixel_type( right_pixel_type )
      {
      }

      sensor::StereoImuCalibration         m_calibration;
      std::vector<sensor::ImuMeasurement>  m_imu_measurements;
      std::vector<ImageFrameManifestEntry> m_left_manifest;
      std::vector<ImageFrameManifestEntry> m_right_manifest;
      sensor::PixelType                    m_left_pixel_type;
      sensor::PixelType                    m_right_pixel_type;
    };

    class StereoImuDatasetReaderImpl
    {
    public:
      explicit StereoImuDatasetReaderImpl(
          std::shared_ptr<const StereoImuDatasetImpl> dataset )
          : m_dataset( std::move( dataset ) )
      {
      }

      std::shared_ptr<const StereoImuDatasetImpl> m_dataset;
      std::size_t                                 m_next_imu_index   = 0;
      std::size_t                                 m_next_left_index  = 0;
      std::size_t                                 m_next_right_index = 0;
      std::optional<DatasetReaderError>           m_terminal_left;
      std::optional<DatasetReaderError>           m_terminal_right;
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
        common::Timestamp timestamp, std::uint32_t expected_width,
        std::uint32_t     expected_height,
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
      if ( decoded.cols < 0 || decoded.rows < 0 ||
           static_cast<std::uint32_t>( decoded.cols ) != expected_width ||
           static_cast<std::uint32_t>( decoded.rows ) != expected_height )
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

    [[nodiscard]] bool isLeft( sensor::CameraId camera ) noexcept
    {
      return camera == sensor::CameraId::kLeft;
    }

  }  // namespace

  StereoImuDataset internal::StereoImuDatasetBuilder::build(
      sensor::StereoImuCalibration         calibration,
      std::vector<sensor::ImuMeasurement>  imu_measurements,
      std::vector<ImageFrameManifestEntry> left_manifest,
      std::vector<ImageFrameManifestEntry> right_manifest,
      sensor::PixelType                    left_pixel_type,
      sensor::PixelType                    right_pixel_type )
  {
    return StereoImuDataset{ std::make_shared<const StereoImuDatasetImpl>(
        std::move( calibration ), std::move( imu_measurements ),
        std::move( left_manifest ), std::move( right_manifest ),
        left_pixel_type, right_pixel_type ) };
  }

  StereoImuDataset::StereoImuDataset(
      std::shared_ptr<const internal::StereoImuDatasetImpl> impl )
      : m_impl( std::move( impl ) )
  {
  }

  sensor::StereoImuCalibration StereoImuDataset::calibration() const
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
                                    summarize( m_impl->m_left_manifest ),
                                    summarize( m_impl->m_right_manifest ) };
  }

  std::size_t StereoImuDataset::exactTimestampIntersectionCount()
      const noexcept
  {
    const auto& left  = m_impl->m_left_manifest;
    const auto& right = m_impl->m_right_manifest;
    std::size_t i     = 0;
    std::size_t j     = 0;
    std::size_t count = 0;
    while ( i < left.size() && j < right.size() )
    {
      if ( left[ i ].timestamp == right[ j ].timestamp )
      {
        ++count;
        ++i;
        ++j;
      }
      else if ( left[ i ].timestamp < right[ j ].timestamp )
      {
        ++i;
      }
      else
      {
        ++j;
      }
    }
    return count;
  }

  StereoImuDatasetReader StereoImuDataset::reader() const
  {
    return StereoImuDatasetReader{ m_impl };
  }

  StereoImuDatasetReader::StereoImuDatasetReader(
      std::shared_ptr<const internal::StereoImuDatasetImpl> impl )
      : m_impl( std::make_unique<internal::StereoImuDatasetReaderImpl>(
            std::move( impl ) ) )
  {
  }

  StereoImuDatasetReader::~StereoImuDatasetReader() = default;

  StereoImuDatasetReader::StereoImuDatasetReader(
      StereoImuDatasetReader&& ) noexcept = default;

  StereoImuDatasetReader& StereoImuDatasetReader::operator=(
      StereoImuDatasetReader&& ) noexcept = default;

  DatasetReaderResult<sensor::ImuMeasurement>
  StereoImuDatasetReader::takeImu()
  {
    if ( m_impl->m_next_imu_index >=
         m_impl->m_dataset->m_imu_measurements.size() )
    {
      return DatasetReaderEnd{};
    }
    return m_impl
        ->m_dataset->m_imu_measurements[ m_impl->m_next_imu_index++ ];
  }

  DatasetReaderResult<common::Timestamp>
  StereoImuDatasetReader::peekImageTimestamp( sensor::CameraId camera )
  {
    const bool  left = isLeft( camera );
    const auto& terminal =
        left ? m_impl->m_terminal_left : m_impl->m_terminal_right;
    if ( terminal.has_value() )
    {
      return *terminal;
    }
    const auto& manifest =
        left ? m_impl->m_dataset->m_left_manifest
             : m_impl->m_dataset->m_right_manifest;
    const std::size_t index =
        left ? m_impl->m_next_left_index : m_impl->m_next_right_index;
    if ( index >= manifest.size() )
    {
      return DatasetReaderEnd{};
    }
    return manifest[ index ].timestamp;
  }

  DatasetReaderResult<sensor::ImageFrameEvent>
  StereoImuDatasetReader::takeImage( sensor::CameraId camera )
  {
    const bool left = isLeft( camera );
    auto&      terminal =
        left ? m_impl->m_terminal_left : m_impl->m_terminal_right;
    if ( terminal.has_value() )
    {
      return *terminal;
    }

    const auto& manifest =
        left ? m_impl->m_dataset->m_left_manifest
             : m_impl->m_dataset->m_right_manifest;
    std::size_t& index =
        left ? m_impl->m_next_left_index : m_impl->m_next_right_index;
    if ( index >= manifest.size() )
    {
      return DatasetReaderEnd{};
    }

    const auto&       reference     = manifest[ index ];
    const std::size_t record_number = index + 1;
    const auto&       calibration   = m_impl->m_dataset->m_calibration;
    const auto        decoded       = decodeImage(
        reference.image_path, left ? "left_camera" : "right_camera", index,
        reference.timestamp,
        left ? calibration.leftCamera().imageWidth()
                          : calibration.rightCamera().imageWidth(),
        left ? calibration.leftCamera().imageHeight()
                          : calibration.rightCamera().imageHeight(),
        left ? m_impl->m_dataset->m_left_pixel_type
                          : m_impl->m_dataset->m_right_pixel_type );
    if ( !decoded )
    {
      terminal =
          makeReaderError( decoded.error(), record_number, reference.timestamp );
      return *terminal;
    }

    ++index;
    return sensor::ImageFrameEvent{ camera, reference.timestamp,
                                    std::move( decoded ).value() };
  }

}  // namespace phad::io::dataset
