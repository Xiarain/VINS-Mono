/**
 * File: demo_brief.cpp
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: demo application of DLoopDetector
 * License: see the LICENSE.txt file
 */

// ----------------------------------------------------------------------------
#include "loop_closure.h"

LoopClosure::LoopClosure(const char *_voc_file, int _image_w, int _image_h)
:demo(_voc_file,_image_w, _image_h), IMAGE_W(_image_w), IMAGE_H(_image_h)
{
    printf(" loop closure init finish\n");
}

/**
 * @brief 初始化相机模型
 * @param calib_file 相机模型的名称
 */
void LoopClosure::initCameraModel(const std::string &calib_file)
{
      demo.initCameraModel(calib_file);
}

/**
 * @brief 开始闭环检测
 * @param keys 3D特征点位置
 * @param descriptors 描述子
 * @param cur_pts 当前帧
 * @param old_pts
 * @param old_index 匹配上的关键帧序号
 * @return
 */
bool LoopClosure::startLoopClosure(std::vector<cv::KeyPoint> &keys, std::vector<BRIEF::bitset> &descriptors,
                                   std::vector<cv::Point2f> &cur_pts,
                                   std::vector<cv::Point2f> &old_pts,
                                   int &old_index)
{
  try 
  {
    bool loop_succ = false;
    loop_succ = demo.run("BRIEF", keys, descriptors, cur_pts, old_pts, old_index);
    return loop_succ;
  }
  catch(const std::string &ex)
  {
    cout << "Error: " << ex << endl;
    return false;
  }
}

void LoopClosure::eraseIndex(std::vector<int> &erase_index)
{
  demo.eraseIndex(erase_index);
}