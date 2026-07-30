#include "phad/viz/image_window.hpp"

#include <algorithm>
#include <opencv2/highgui.hpp>
#include <utility>

namespace phad::viz
{

  namespace
  {

    constexpr int kEscapeKey = 27;
    constexpr int kQuitKey   = 'q';

  }  // namespace

  ImageWindow::ImageWindow( std::string name ) : m_name( std::move( name ) )
  {
    cv::namedWindow( m_name, cv::WINDOW_AUTOSIZE );
  }

  ImageWindow::~ImageWindow()
  {
    try
    {
      cv::destroyWindow( m_name );
    }
    catch ( const cv::Exception& )
    {
      // 窗口可能已被用户关闭；析构不报告这一情况。
    }
  }

  void ImageWindow::show( const cv::Mat& canvas )
  {
    cv::imshow( m_name, canvas );
  }

  bool ImageWindow::pump( int wait_ms )
  {
    const int key = cv::waitKey( std::max( wait_ms, 1 ) );
    if ( key == kEscapeKey || key == kQuitKey )
    {
      return false;
    }
    return isOpen();
  }

  bool ImageWindow::isOpen() const
  {
    return cv::getWindowProperty( m_name, cv::WND_PROP_VISIBLE ) >= 1.0;
  }

}  // namespace phad::viz
