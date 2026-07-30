#pragma once

#include <opencv2/core.hpp>
#include <string>

/**
 * @file image_window.hpp
 * @brief 单个显示窗口的生命周期与退出检测。
 *
 * 回放循环需要在等待下一帧的同时保持窗口响应，并区分「用户要求退出」与
 * 「数据流结束」。窗口把 OpenCV 的 highgui 调用集中到一处，调用方只面对
 * 「显示一帧」和「泵一段时间事件，是否继续」两个动作。
 */

namespace phad::viz
{

  class ImageWindow
  {
  public:
    explicit ImageWindow( std::string name );
    ~ImageWindow();

    ImageWindow( const ImageWindow& )            = delete;
    ImageWindow& operator=( const ImageWindow& ) = delete;
    ImageWindow( ImageWindow&& )                 = delete;
    ImageWindow& operator=( ImageWindow&& )      = delete;

    void show( const cv::Mat& canvas );

    /**
     * @brief 处理至多 wait_ms 毫秒的 GUI 事件。
     * @return 用户按下 q 或 Esc、或窗口已被关闭时返回 false。
     *
     * wait_ms 会被抬到至少 1 ms：OpenCV 把 0 解释为无限等待按键。
     */
    [[nodiscard]] bool pump( int wait_ms );

    [[nodiscard]] bool isOpen() const;

  private:
    std::string m_name;
  };

}  // namespace phad::viz
