/**
 * @file cmd_interface_linux.h
 * @author LDRobot (support@ldrobot.com)
 * @brief  linux serial port App
 * @version 0.2
 * @date 2022-05-27
 *
 * @copyright Copyright (c) 2021  SHENZHEN LDROBOT CO., LTD. All rights
 * reserved.
 * Licensed under the MIT License (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License in the file LICENSE
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef __LINUX_SERIAL_PORT_H__
#define __LINUX_SERIAL_PORT_H__

#include <inttypes.h>
#include <sys/file.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <string.h>
#include <memory.h>
#include <iostream>

#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <functional>
#include <string>
#include <condition_variable>

namespace ldlidar {

enum class STDBaudRateType {
  BAUD_57600,   /** 57600 bps*/
  BAUD_115200,  /** 115200 bps*/  
  BAUD_230400,  /** 230400 bps*/ 
  BAUD_460800,  /** 460800 bps*/
  BAUD_500000,  /** 500000 bps*/
  BAUD_576000,  /** 576000 bps*/
  BAUD_921600,   /** 921600 bps*/
  BAUD_1000000,  /** 1000000 bps*/
  BAUD_1152000,  /** 1152000 bps*/
  BAUD_1500000,  /** 1500000 bps*/
  BAUD_2000000,  /** 2000000 bps*/
  BAUD_2500000,  /** 2500000 bps*/
  BAUD_3000000,  /** 3000000 bps*/
  BAUD_3500000,  /** 3500000 bps*/
  BAUD_4000000,  /** 4000000 bps*/
};

class CmdInterfaceLinux {   
public:
  CmdInterfaceLinux();
  ~CmdInterfaceLinux();

  /**
   * @brief open serial or usb device, crate recevie date thread handle and set communication baudrate.
   * @param [in]
   *  @param port_name:  this varible type is std::string, input system mounted device file, eg: "/dev/ttyUSB0"
   *  @param baudrate:  this vairble type is ldlidar::STDBaudRateType, baudrate Enum,input value refer to "enum class STDBaudRateType", 
   *      eg: ldlidar::STDBaudRateType::BAUD_115200
   * @retval value is true, open is success.
   *         value is false, open is failed.
  */
  bool Open(std::string& port_name, STDBaudRateType baudrate);
  bool Close();
  bool ReadFromIO(uint8_t *rx_buf, uint32_t rx_buf_len, uint32_t *rx_len);
  bool WriteToIo(const uint8_t *tx_buf, uint32_t tx_buf_len, uint32_t *tx_len);
	void SetReadCallback(std::function<void(const char *, size_t length)> callback) { read_callback_ = callback; }
	bool IsOpened() { return is_cmd_opened_.load(); };

private:
  std::thread *rx_thread_;
  long long rx_count_;
  int32_t com_handle_;
  std::atomic<bool> is_cmd_opened_, rx_thread_exit_flag_;
  std::function<void(const char *, size_t length)> read_callback_;

  static void RxThreadProc(void *param);
};

}

#endif
/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/