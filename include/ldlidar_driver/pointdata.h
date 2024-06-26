/**
 * @file pointdata.h
 * @author LDRobot (support@ldrobot.com)
 * @brief  lidar point data structure
 *         This code is only applicable to LDROBOT products
 * sold by Shenzhen LDROBOT Co., LTD
 * @version 0.1
 * @date 2021-11-09
 * Licensed under the MIT License (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License in the file LICENSE
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * @copyright Copyright (c) 2021  SHENZHEN LDROBOT CO., LTD. All rights
 * reserved.
 *
 */
#ifndef _POINT_DATA_H_
#define _POINT_DATA_H_

#include <stdint.h>
#include <string.h>
#include <math.h>

#include <cstring>
#include <vector>
#include <algorithm>
#include <iostream>


namespace ldlidar {

#define ANGLE_TO_RADIAN(angle) ((angle)*3141.5926/180000) // degress unit transform to radin
#define RADIAN_TO_ANGLE(angle) ((angle)*180000/3141.5926) // radin uint transform to degress 

enum class LDType {
  LD_SSL20_L,
  LD_SSL20_N,
  LD_SSL20_P,
  LD_07N,
  LD_NO_TYPE
};

struct PointData {
  //极坐标表示方式
  float angle;
  uint16_t distance;
  uint8_t intensity;
  //直角坐标表示方式
  double x;
  double y;
  PointData(float angle, uint16_t distance, uint8_t intensity , double x = 0, double y = 0) {
    this->angle = angle;
    this->distance = distance;
    this->intensity = intensity;
    this->x = x;
    this->y = y;
  }
  PointData() {}
  friend std::ostream& operator<<(std::ostream &os , const PointData &data) {
    os << data.angle << " "<< data.distance << " " << (int)data.intensity << " "<<data.x << " "<<data.y;
    return  os;
  }
};

typedef std::vector<PointData> Points2D;

} // namespace ldlidar 

#endif // _POINT_DATA_H_

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/