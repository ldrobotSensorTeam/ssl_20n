/**
 * @file main.cpp
 * @author LDRobot (support@ldrobot.com)
 * @brief  main process App
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
#include "rtrnet.h"

#define isTrigMode 0

int main(int argc, char **argv) {

	if (argc != 4) {
		std::cout << "please input:" << std::endl;
		std::cout << "./ldlidar_ssl <product_type1> <product_type2> <port_name>" << std::endl;
		std::cout << "example: " << std::endl;
		std::cout << "./ldlidar_ssl SSL20N SSL20L /dev/ttyUSB0" << std::endl;
		exit(EXIT_FAILURE);
	}
	
	// 1. 激光雷达数据处理类实例化
	ldlidar::RTRNet* lidar_comm = new ldlidar::RTRNet();
	// 2. 串口应用类实例化
	ldlidar::CmdInterfaceLinux* cmd_port = new ldlidar::CmdInterfaceLinux();

	std::cout << "[ldrobot] SDK Pack Version is " << lidar_comm->GetSdkPackVersionNumber() << std::endl;

	ldlidar::LDType type1;
	ldlidar::LDType type2;
	std::string product_type1(argv[1]);
	std::string product_type2(argv[2]);
	std::string port_name(argv[3]);
	if ("SSL20L" == product_type1) {
		type1 = ldlidar::LDType::LD_SSL20_L;
	}else if ("SSL20N" == product_type1) {
		type1 = ldlidar::LDType::LD_SSL20_N;
	}else if ("SSL20P" == product_type1) {
		type1 = ldlidar::LDType::LD_SSL20_P;
	}else if ("LD07N" == product_type1) {
		type1 = ldlidar::LDType::LD_07N;
	}else {
		type1 = ldlidar::LDType::LD_NO_TYPE;
	}
	if ("SSL20L" == product_type2) {
		type2 = ldlidar::LDType::LD_SSL20_L;
	}else if ("SSL20N" == product_type2) {
		type2 = ldlidar::LDType::LD_SSL20_N;
	}else if ("SSL20P" == product_type2) {
		type2 = ldlidar::LDType::LD_SSL20_P;
	}else if ("LD07N" == product_type2) {
		type2 = ldlidar::LDType::LD_07N;
	}else {
		type2 = ldlidar::LDType::LD_NO_TYPE;
	}
	// 3. 设置产品型号
	if (!lidar_comm->SetProductType(ldlidar::LDType::LD_SSL20_N)) {
		std::cout << "[ldrobot] set product type is fail" << std::endl;
		exit(EXIT_FAILURE);
	}
	// 4. 打开激光雷达对应的串口设备
	if(!cmd_port->Open(port_name, ldlidar::STDBaudRateType::BAUD_921600)) {
		std::cout << "[ldrobot] open " << port_name << " device is fail" << std::endl;
		exit(EXIT_FAILURE);
	} else {
		std::cout << "[ldrobot] open " << port_name << " device is success" << std::endl;
	}
	// 5. 设置串口设备读数据处理回调
	cmd_port->SetReadCallback(std::bind(&ldlidar::RTRNet::CommReadCallback, lidar_comm, std::placeholders::_1, std::placeholders::_2));
	// if (!lidar_comm->SendCmd(cmd_port, 0, ldlidar::PACK_RESET_SYSTEM)) {
	// 		std::cout << "[ldrobot] send to PACK_CONFIG_ADDRESS cmd fail" << std::endl;
	// 		exit(EXIT_FAILURE);
	// }

	// 10. 发送停止激光雷达扫描命令
	if (!lidar_comm->SendCmd(cmd_port, LIDAR_DEVICE_01, ldlidar::PACK_STOP)) {
		std::cout << "[ldrobot] [DEVICE_01] send to PACK_STOP_DISTANCE_TRANS cmd fail" << std::endl;
	} else {
		std::cout << "[ldrobot] [DEVICE_01] send to PACK_STOP_DISTANCE_TRANS cmd success" << std::endl;
	}
	sleep(1);
	if (!lidar_comm->SendCmd(cmd_port, LIDAR_DEVICE_02, ldlidar::PACK_STOP)) {
		std::cout << "[ldrobot] [DEVICE_02] send to PACK_STOP_DISTANCE_TRANS cmd fail" << std::endl;
	} else {
		std::cout << "[ldrobot] [DEVICE_02] send to PACK_STOP_DISTANCE_TRANS cmd success" << std::endl;
	}
	sleep(1);

	// 6. 发送命令并获取激光雷达参数
	int error = 0;
	while (!lidar_comm->IsParametersReady()) {
		if (!lidar_comm->SendCmd(cmd_port, 0, ldlidar::PACK_CONFIG_ADDRESS)) {
			std::cout << "[ldrobot] send to PACK_CONFIG_ADDRESS cmd fail" << std::endl;
			exit(EXIT_FAILURE);
		}
		sleep(1);
		if (!lidar_comm->SendCmd(cmd_port, LIDAR_DEVICE_01, ldlidar::PACK_GET_COE)) {
			std::cout << "[ldrobot] [DEVICE_01] send to PACK_GET_COE cmd fail" << std::endl;
			exit(EXIT_FAILURE);
		}
		sleep(1);
		if (!lidar_comm->SendCmd(cmd_port, LIDAR_DEVICE_02, ldlidar::PACK_GET_COE)) {
			std::cout << "[ldrobot] [DEVICE_02] send to PACK_GET_COE cmd fail" << std::endl;
			exit(EXIT_FAILURE);
		}
		sleep(1);
		if (!lidar_comm->SendCmd(cmd_port, LIDAR_DEVICE_01, ldlidar::PACK_VIDEO_SIZE)) {
			std::cout << "[ldrobot] [DEVICE_01] send to PACK_VIDEO_SIZE cmd fail" << std::endl;
			exit(EXIT_FAILURE);
		}
		sleep(1);
		if (!lidar_comm->SendCmd(cmd_port, LIDAR_DEVICE_02, ldlidar::PACK_VIDEO_SIZE)) {
			std::cout << "[ldrobot] [DEVICE_02] send to PACK_VIDEO_SIZE cmd fail" << std::endl;
			exit(EXIT_FAILURE);
		}
		sleep(1);

		error++;
		if (error > 2) { /* Exit if the number of errors is more than 2*/
			std::cout << "[ldrobot] Error: get ssl20 lidar parameters fail" << std::endl;
			exit(EXIT_FAILURE);
		}
	}
	lidar_comm->ResetParametersReady();
	std::cout << "[ldrobot] get ssl20 lidar parameters success" << std::endl;

	//插值距离模式输出
	uint8_t send_parbuf = 0;
	send_parbuf = 0;			//1：开启插值模式，0：关闭插值模式
	if (!lidar_comm->sendbytes(cmd_port, LIDAR_DEVICE_01, ldlidar::PACK_COE_INTERPOLATION, &send_parbuf, 1, 0)) {
		std::cout << "[ldrobot] [DEVICE_01] send to PACK_COE_INTERPOLATION cmd fail" << std::endl;
		exit(EXIT_FAILURE);
	} else {
		std::cout << "[ldrobot] [DEVICE_01] send to PACK_COE_INTERPOLATION cmd success" << std::endl;
	}
	if (!lidar_comm->sendbytes(cmd_port, LIDAR_DEVICE_02, ldlidar::PACK_COE_INTERPOLATION, &send_parbuf, 1, 0)) {
		std::cout << "[ldrobot] [DEVICE_02] send to PACK_COE_INTERPOLATION cmd fail" << std::endl;
		exit(EXIT_FAILURE);
	} else {
		std::cout << "[ldrobot] [DEVICE_02] send to PACK_COE_INTERPOLATION cmd success" << std::endl;
	}
	lidar_comm->SetDistanceModule(send_parbuf);
	sleep(1);


	if (type1== ldlidar::LDType::LD_SSL20_N) {
		uint8_t send_parbuf[256] = {0};
		uint32_t GroundDistance = 180;
		strcpy((char *)send_parbuf, "ground distance");
		memcpy(&send_parbuf[PACK_STRING_LEN], (uint8_t *)&GroundDistance, 4);
		if (!lidar_comm->sendWords(cmd_port, LIDAR_DEVICE_01, ldlidar::PACK_SET_GROUND_DISTANCE, (uint32_t *)send_parbuf, 4 + PACK_STRING_LEN, 0)) {
			std::cout << "[ldrobot] [DEVICE_01] send to PACK_SET_GROUND_DISTANCE cmd fail" << std::endl;
			exit(EXIT_FAILURE);
		}
	}

	if (type2== ldlidar::LDType::LD_SSL20_N) {
		uint8_t send_parbuf[256] = {0};
		uint32_t GroundDistance = 180;
		strcpy((char *)send_parbuf, "ground distance");
		memcpy(&send_parbuf[PACK_STRING_LEN], (uint8_t *)&GroundDistance, 4);
		if (!lidar_comm->sendWords(cmd_port, LIDAR_DEVICE_02, ldlidar::PACK_SET_GROUND_DISTANCE, (uint32_t *)send_parbuf, 4 + PACK_STRING_LEN, 0)) {
			std::cout << "[ldrobot] [DEVICE_02] send to PACK_SET_GROUND_DISTANCE cmd fail" << std::endl;
			exit(EXIT_FAILURE);
		}
	}

	// 7. 发送启动激光雷达扫描命令
		//级联软件级联同步模式
		sleep(1);
		if (!lidar_comm->SendCmd(cmd_port, LIDAR_DEVICE_01, ldlidar::PACK_GET_DISTANCE)) {
			std::cout << "[ldrobot] [DEVICE_01] send to PACK_GET_DISTANCE cmd fail" << std::endl;
		} else {
			std::cout << "[ldrobot] [DEVICE_01] send to PACK_GET_DISTANCE cmd success" << std::endl;
		}
		sleep(1);
		if (!lidar_comm->SendCmd(cmd_port, LIDAR_DEVICE_02, ldlidar::PACK_GET_DISTANCE)) {
			std::cout << "[ldrobot] [DEVICE_02] send to PACK_GET_DISTANCE cmd fail" << std::endl;
		} else {
			std::cout << "[ldrobot] [DEVICE_02] send to PACK_GET_DISTANCE cmd success" << std::endl;
		}
		sleep(1);

	auto last_time = std::chrono::steady_clock::now();
	while (1) {
		if (lidar_comm->IsFrameReady()) { // 8. 判断激光扫描数据是否就绪
			lidar_comm->ResetFrameReady();// 清除就绪标志
			last_time = std::chrono::steady_clock::now();
			// 9. 获取级联激光扫描数据
      uint8_t device_addr_id = lidar_comm->GetDeviceAddress();
      switch (device_addr_id) {
        case LIDAR_DEVICE_01: {
          ldlidar::Points2D laser_scan = lidar_comm->GetLaserScanData();
          std::cout << "[ldrobot] pub laser1" << std::endl;
          for (auto point : laser_scan) {
            std::cout << "[ldrobot] [1] angle: " << point.angle << " "
                      << " distance(m): " << point.distance << " "
                      << " intensity: " << (int)point.intensity << std::endl;
          }
          break;
        }
        case LIDAR_DEVICE_02: {
          ldlidar::Points2D laser_scan = lidar_comm->GetLaserScanData();
          std::cout << "[ldrobot] pub laser2" << std::endl;
          for (auto point : laser_scan) {
            std::cout << "[ldrobot] [2] angle: " << point.angle << " "
                      << " distance(m): " << point.distance << " "
                      << " intensity: " << (int)point.intensity << std::endl;
          }
          break;
        }
        default:
          break;
      }
		}

		if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-last_time).count() > 1000) { 
			// 数据发布超时或者串口设备拔出处理
			std::cout << "[ldrobot] lidar pub data is time out, please check lidar device" << std::endl;
			// exit(EXIT_FAILURE);
		}

		usleep(5*1000);
	}

	// 10. 发送停止激光雷达扫描命令
	if (!lidar_comm->SendCmd(cmd_port, LIDAR_DEVICE_01, ldlidar::PACK_STOP)) {
		std::cout << "[ldrobot] [DEVICE_01] send to PACK_STOP_DISTANCE_TRANS cmd fail" << std::endl;
	} else {
		std::cout << "[ldrobot] [DEVICE_01] send to PACK_STOP_DISTANCE_TRANS cmd success" << std::endl;
	}
	sleep(1);
	if (!lidar_comm->SendCmd(cmd_port, LIDAR_DEVICE_02, ldlidar::PACK_STOP)) {
		std::cout << "[ldrobot] [DEVICE_02] send to PACK_STOP_DISTANCE_TRANS cmd fail" << std::endl;
	} else {
		std::cout << "[ldrobot] [DEVICE_02] send to PACK_STOP_DISTANCE_TRANS cmd success" << std::endl;
	}
	sleep(1);

	// 11. 关闭串口设备
	cmd_port->Close();

	delete lidar_comm;
	lidar_comm = nullptr;
	delete cmd_port;
	cmd_port = nullptr;

	return 0;
}


/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/