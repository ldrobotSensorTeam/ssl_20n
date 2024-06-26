/**
 * @file trnet.cpp
 * @author LDRobot (support@ldrobot.com)
 * @brief  LiDAR data protocol processing App
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
#ifndef _TR_NET_H
#define _TR_NET_H

#include "pointdata.h"

namespace ldlidar {


class TransData {
 public:
  TransData();
	~TransData();
	bool TransformSignlePoint(uint32_t* coe, uint8_t coe_size, 
	  uint16_t dist, int n,int center_kb, uint8_t confidence, Points2D& dst);
	bool TransformSignlePoint(uint32_t* coe, uint8_t coe_size, 
		uint16_t dist, int n,int center_kb, uint8_t confidence, Points2D& dst,uint8_t angle_offset);
};

}

#endif // _TR_NET_H

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/