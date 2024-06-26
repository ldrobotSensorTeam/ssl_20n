/**
 * @file rtrnet.cpp
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
#include "rtrnet.h"

namespace ldlidar {

RTRNet::RTRNet(): sdk_pack_version_number_("v1.2.1"),
  product_type_number_(LDType::LD_NO_TYPE),
  frame_ready_(false),
  parameters_ready_(false),
  fov_angle_(0),
  total_point_number_(0),
  tr_data_(nullptr), 
  parse_data_len_(0),
  distance_module(0),
  is_filter_(false) {

}

RTRNet::~RTRNet() {

}

void RTRNet::CommReadCallback(const char *byte, size_t len) {
  this->UnpackData((uint8_t *)byte, len);
}
//data analysis 
bool RTRNet::Pack(const TRData &in, std::vector<uint8_t> &out) {
  out.resize(EXTRA_LEN + in.data.size());
  uint8_t *p = out.data();
  *(uint32_t *)p = LEADING_CODE;
  p += 4;
  *p++ = in.device_address;
  *p++ = in.pack_id;
  *(uint16_t *)p = in.chunk_offset;
  p += 2;
  *(uint16_t *)p = in.data.size();
  p += 2;
  std::memcpy(p, in.data.data(), in.data.size());
  uint8_t checksum = CalCheckSum(out.data() + 4, out.size() - 5);
  
  out.back() = checksum;

  return true;
}

bool RTRNet::FindLeadingCode(const uint8_t *buff) {
  uint32_t code = *(uint32_t *)buff;
  
  return (code == LEADING_CODE);
}
//check data sum
uint8_t RTRNet::CalCheckSum(const uint8_t *data, uint16_t len) {
  uint8_t checksum = 0;

  for (uint16_t i = 0; i < len; i++) {
    checksum += *data++;
  }

  return checksum;
}

//unpack the pack 
const TRData *RTRNet::Unpack(const uint8_t *data, uint32_t len) {
  if (data == nullptr || len < EXTRA_LEN) {
    return nullptr;
  }

  const uint8_t *p = data;
  uint32_t code = *(uint32_t *)data;

  if (code != LEADING_CODE) {
    return nullptr;
  }

  p += 8;
  uint16_t data_len = *(uint16_t *)p;

  if (data_len > (len - EXTRA_LEN)) {
    return nullptr;
  }

  p += 2;

  uint8_t checksum = CalCheckSum(data + 4, 6 + data_len);

  p += data_len;

  if (checksum == *p) {
    p = data;
    p += 4;
    tr_unpack_data_.device_address = *p++;
    tr_unpack_data_.pack_id = *p++;
    tr_unpack_data_.chunk_offset = *(uint16_t *)p;
    p += 2;
    if (tr_unpack_data_.data.size() < data_len) {
      tr_unpack_data_.data.resize(data_len);
    }
    p += 2;
    std::memcpy(tr_unpack_data_.data.data(), p, data_len);
    parse_data_len_ = data_len + EXTRA_LEN;

    return &tr_unpack_data_;
  }

  return nullptr;
}

bool RTRNet::AnalysisTRNetByte(uint8_t byte) {
  static enum {
    HEADER1,
    HEADER2,
    HEADER3,
    HEADER4,
    LENS,
    DATA,
  } state = HEADER1;
  static uint16_t count = 0;
  static uint8_t tmp[500] = {0};
  static uint16_t pkg_count = 0;

  switch (state) {
    case HEADER1:
      if (byte == 0xAA) {
        tmp[count++] = byte;
        state = HEADER2;
      }
      break;
    case HEADER2:
      if (byte == 0xAA) {
        tmp[count++] = byte;
        state = HEADER3;
      } else {
        state = HEADER1;
        count = 0;
      }
      break;
    case HEADER3:
      if (byte == 0xAA) {
        tmp[count++] = byte;
        state = HEADER4;
      } else {
        state = HEADER1;
        count = 0;
      }
      break;
    case HEADER4:
      if (byte == 0xAA) {
        tmp[count++] = byte;
        state = LENS;
      } else {
        state = HEADER1;
        count = 0;
      }
      break;
    case LENS:
      tmp[count++] = byte;
      if (count == 10) {       
        uint16_t data_lens_val = ((tmp[9] << 8) | tmp[8]);
        uint16_t check_len = 324;
        if (GetDistanceModule()) 
        {
          check_len = 324 + 40;
        }
        if (data_lens_val > check_len) {
          state = HEADER1;
          count = 0;
        } else {
          pkg_count = data_lens_val + 11;
          state = DATA;
        }
      }
      break;
    case DATA:
      tmp[count++] = byte;
      if (count >= pkg_count) {
        state = HEADER1;
        count = 0;
        tr_data_ = Unpack(tmp, pkg_count);
        
        if ((tr_data_ != nullptr)) {        
          return true;
        } else {
          return false;
        }
      }
      break;
    default:
      break;
  }

  return false; 
}

uint8_t RTRNet::ParseAddr2Index(uint8_t address) {
  uint8_t index = 0;
  // sensor address suppose to be 0x1 | 0x1<<1 | 0x1<<2 |...
  if (address == 0x0) {
    return 0;
  } 

  while ((address&0x1) == 0) {
    index++;
    address = address >> 1;
  }

  return (index >= SENSOR_MAX_NUM) ? (SENSOR_MAX_NUM - 1) : index;
}

bool RTRNet::UnpackData(const uint8_t *data, uint32_t len) {
  
  for (uint32_t i = 0; i < len; i++) {
    if (AnalysisTRNetByte(data[i])) {
      switch (tr_data_->pack_id) {
        case PACK_GET_DISTANCE: {
          // std::cout << "[ldrobot] get PACK_GET_DISTANCE " << std::endl; 
          if (tr_data_->device_address == THIS_DEVICE_ADDREESS) {
            break;
          }
          SetDeviceAddress(tr_data_->device_address);
          uint8_t index = ParseAddr2Index(tr_data_->device_address);
          memcpy(coe_, coe[index],sizeof(coe_));
          Transform(tr_data_);
          break;
        }
        case PACK_GET_COE: {
          std::cout << "[ldrobot] get PACK_GET_COE " << std::endl; 
          if (tr_data_->device_address == THIS_DEVICE_ADDREESS) {
            break;
          }
          uint8_t index = ParseAddr2Index(tr_data_->device_address);
          memcpy(coe[index], tr_data_->data.data(),sizeof(coe_));
          break;
        }
        case PACK_VIDEO_SIZE: {
          std::cout << "[ldrobot] get PACK_VIDEO_SIZE " << std::endl;
          if (tr_data_->device_address == THIS_DEVICE_ADDREESS) {
            break;
          }
          uint8_t index = ParseAddr2Index(tr_data_->device_address);
          coe_u_[index] = *(uint16_t *)(tr_data_->data.data());
          coe_v_[index] = *(uint16_t *)(tr_data_->data.data() + 2);
          // std::cout << "[ldrobot] Picture pixels: " << coe_u_ << "*" << coe_v_ << std::endl;
          SetParametersReady();
          break;
        }
        case PACK_ACK: {
          
          uint8_t cmd = *tr_data_->data.data();
          uint8_t ack = *(tr_data_->data.data()+1);
          std::cout << "[ldrobot] ACK RSP addreess: "<< int(tr_data_->device_address) << std::endl;
          std::cout << "[ldrobot] ACK RSP cmd: "<< (int)cmd << " ,ack:" << (int)ack << std::endl;
        }

        default: {
          break;
        }
      }
    }
  }

  return true;
}

bool RTRNet::Transform(const TRData *tr_data) {
  Points2D tmp,filter_tmp;
  /*Packet length minus 4-byte timestamp*/
  int data_amount = tr_data->data.size() - 4; 
  int n = 0;
  TransData trans_data;

  
  if (GetDistanceModule()) {        //角度插值模式
      data_amount = data_amount - 40;  //插值模式多40个字节
  }

  for (int i = 0; i < data_amount; i += 2, n++) {
    /*Acquired distance information data*/
    uint16_t value = *(uint16_t *)(tr_data->data.data() + i + 4);
    uint16_t confidence = 0;
    uint16_t center_dis = 0;
    uint8_t  center_kb = 0;

    uint8_t angle_offset = 0;   //计算角度字节偏移量
    uint8_t angle_offset_value = 0;
    uint8_t angle_interpolation_value = 0;

    if (GetDistanceModule()) {        //角度插值模式
      angle_offset = n / 4;            //计算角度字节偏移量
      angle_offset_value = *(uint8_t *)(tr_data->data.data() + angle_offset + 324);
      angle_interpolation_value = 0;
    }

    //get the distance,center_value,cofidence
    if(GetProductType() == LDType::LD_SSL20_P){
      confidence = (((value >> 11) & 0x003) << 6);  //confidence [12:10]   the value need to multiply by 64
      center_dis = (value & 0x7ff);                 //distance   [10:0]    same as   
    }
    else{
      confidence = (((value >> 10) & 0x007) << 5);  //confidence [12:10]   the value need to multiply by 32
      center_dis = (value & 0x3ff);                 //distance   [9:0]    same as      
    }

    center_kb  = ((value >> 13) & 0x007);         //kb range   [15:13]  high FOV,need this,because of the camera distortion
    if (center_dis > 0) {
      if (GetDistanceModule()) {          //角度插值模式
        angle_interpolation_value = ((angle_offset_value >> (i % 8)) & 0x03 );   //获取角度插值
        
        if (!trans_data.TransformSignlePoint(coe_, 35, center_dis, n * 2, center_kb, confidence, tmp,angle_interpolation_value)) {
          std::cout << "[ldrobot] trans data error" << std::endl;
        }

      } else {
        if (!trans_data.TransformSignlePoint(coe_, 35, center_dis, n * 2,center_kb, confidence, tmp)) {
          std::cout << "[ldrobot] trans data error" << std::endl;
        }
      }
    }
  }
  Sslbf outlier_point(fov_angle_, total_point_number_);
  if (GetFilter()) {
    filter_tmp = outlier_point.OutlierFilter(tmp);
  } else {
    filter_tmp = tmp;
  }
    switch (GetDeviceAddress()) {
    case LIDAR_DEVICE_01:
      SetLaserScanData(filter_tmp);
      SetFrameReady();
      break;
    case LIDAR_DEVICE_02:
      SetLaserScanData(filter_tmp);
      SetFrameReady();
      break;
    case LIDAR_DEVICE_03:
      SetLaserScanData(filter_tmp);
      SetFrameReady();
      break;
    default:
      break;
  }

  return true;
}

std::string RTRNet::GetSdkPackVersionNumber(void) {
  return sdk_pack_version_number_;
}

bool RTRNet::SetProductType(LDType type_number) {
  product_type_number_ = type_number;
  switch (type_number) {
    case LDType::LD_SSL20_L:
      fov_angle_ = 110;
      total_point_number_ = 160;
      return true;
      break;
    case LDType::LD_SSL20_N:
      fov_angle_ = 115;
      total_point_number_ = 160;
      return true;
      break;
    case LDType::LD_SSL20_P:
      fov_angle_ = 115;
      total_point_number_ = 160;
      return true;
      break;
    case LDType::LD_07N:
      fov_angle_ = 110;
      total_point_number_ = 160;
      return true;
      break;
    default:
      return false;
      break;
  }
}

LDType RTRNet::GetProductType(void) {
  return product_type_number_;
}

double RTRNet::GetFovAngleVal(void) {
  return fov_angle_;
}

double RTRNet::GetTotalPointNumberVal(void) {
  return total_point_number_;
}

bool RTRNet::SendCmd(CmdInterfaceLinux* port, uint8_t address, uint8_t id) {
  
  std::vector<uint8_t> out;
  TRData out_data;
  uint32_t len = 0;
  out_data.device_address = address;
  out_data.pack_id = id;
  out_data.chunk_offset = 0;
  Pack(out_data, out);

  if (port->WriteToIo((const uint8_t *)out.data(), out.size(), &len)) {
    return true;
  } else {
    return false;
  }

}

bool RTRNet::sendbytes(CmdInterfaceLinux* port, uint8_t address, uint8_t id, uint8_t *data, uint32_t len, uint16_t offset) {
  
  std::vector<uint8_t> out;
  TRData out_data;
  out_data.device_address = address;
  out_data.pack_id = id;
  out_data.chunk_offset = offset;
  out_data.data.resize(len);
  memcpy(out_data.data.data(), data, len);
  Pack(out_data, out);

  if (port->WriteToIo((const uint8_t *)out.data(), out.size(), &len)) {
    return true;
  } else {
    return false;
  }
}

bool RTRNet::sendWords(CmdInterfaceLinux* port, uint8_t address, uint8_t id, uint32_t *data, uint32_t len, uint16_t offset) {
  
  std::vector<uint8_t> out;
  TRData out_data;
  out_data.device_address = address;
  out_data.pack_id = id;
  out_data.chunk_offset = offset;
  out_data.data.resize(len);
  memcpy(out_data.data.data(), data, len);
  Pack(out_data, out);

  if (port->WriteToIo((const uint8_t *)out.data(), out.size(), &len)) {
    return true;
  } else {
    return false;
  }
}

bool RTRNet::SendStopDistanceTransmitCmd(CmdInterfaceLinux* port) {
  uint32_t len = 0;
  uint8_t tx_buf[11] = {0xaa,0xaa,0xaa,0xaa,0x01,0x0f,0x00,0x00,0x00,0x00,0x10};
  if (port->WriteToIo(tx_buf,sizeof(tx_buf),&len)) {
    return true;
  } else {
    return false;
  }
}

void RTRNet::ResetFrameReady(void) {
  std::lock_guard<std::mutex> lg(mutex_lock1_);
  frame_ready_ = false;
}

void RTRNet::SetFrameReady(void) {
  std::lock_guard<std::mutex> lg(mutex_lock1_);
  frame_ready_ = true;
}

bool RTRNet::IsFrameReady(void) {
  std::lock_guard<std::mutex> lg(mutex_lock1_);
  return frame_ready_; 
} 

void RTRNet::ResetParametersReady(void) {
  std::lock_guard<std::mutex> lg(mutex_lock2_);
  parameters_ready_ = false;
}

void RTRNet::SetParametersReady(void) {
  std::lock_guard<std::mutex> lg(mutex_lock2_);
  parameters_ready_ = true;
}

bool RTRNet::IsParametersReady(void) {
  std::lock_guard<std::mutex> lg(mutex_lock2_);
  return parameters_ready_; 
}

Points2D RTRNet::GetLaserScanData(void) {
  std::lock_guard<std::mutex> lg(mutex_lock3_);
  return laser_scan_data_;
}

void RTRNet::SetLaserScanData(Points2D& src) {
  std::lock_guard<std::mutex> lg(mutex_lock3_);
  laser_scan_data_ = src;
}

uint8_t RTRNet::GetDeviceAddress(void) {
  std::lock_guard<std::mutex> lg(mutex_lock4_);
  return device_address_; 
}

void RTRNet::SetDeviceAddress(uint8_t dev_address) {
  std::lock_guard<std::mutex> lg(mutex_lock4_);
  device_address_ = dev_address;
}

void RTRNet::SetDistanceModule(uint8_t data) {
  
  distance_module = data;
}

uint8_t RTRNet::GetDistanceModule(void) {
  
  return distance_module;
}
void RTRNet::SetFilter(bool filter) {
  
  is_filter_ = filter;
}

bool RTRNet::GetFilter(void) {
  
  return is_filter_;
}

}

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/