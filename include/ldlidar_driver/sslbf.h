/**
 * @file sslbf.h
 * @author LDRobot (support@ldrobot.com)
 * @brief  LiDAR noise data filter processing App
 *         This code is only applicable to LDROBOT LiDAR
 * products sold by Shenzhen LDROBOT Co., LTD
 * @version 0.1
 * @date 2022-05-27
 *
 * @copyright Copyright (c) 2021  SHENZHEN LDROBOT CO., LTD. All rights
 * reserved. Licensed under the MIT License (the "License"); you may not use
 * this file except in compliance with the License. You may obtain a copy of the
 * License in the file LICENSE Unless required by applicable law or agreed to in
 * writing, software distributed under the License is distributed on an "AS IS"
 * BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 * implied. See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef __Sslbf_H_
#define __Sslbf_H_

#include "pointdata.h"

namespace ldlidar {

class Sslbf {
private:
  double fov_angle_;
  double total_point_number_;
  
  Sslbf(const Sslbf &) = delete;
  Sslbf &operator=(const Sslbf &) = delete;
public:
  Sslbf(double fov_angle, double total_point_number);
  ~Sslbf();
  Points2D OutlierFilter(Points2D& tmp) const;
};

}

#endif

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/