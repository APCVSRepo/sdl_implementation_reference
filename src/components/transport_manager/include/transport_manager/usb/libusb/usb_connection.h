/*
 * Copyright (c) 2013-2014, Ford Motor Company
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of the Ford Motor Company nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SRC_COMPONENTS_TRANSPORT_MANAGER_INCLUDE_TRANSPORT_MANAGER_USB_LIBUSB_USB_CONNECTION_H_
#define SRC_COMPONENTS_TRANSPORT_MANAGER_INCLUDE_TRANSPORT_MANAGER_USB_LIBUSB_USB_CONNECTION_H_

#include <list>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "utils/lock.h"

#include "transport_manager/transport_adapter/transport_adapter_controller.h"
#include "transport_manager/transport_adapter/connection.h"
#include "transport_manager/usb/common.h"

namespace transport_manager {
namespace transport_adapter {

#ifndef REMOVEQUEUE
#define REMOVEQUEUE
#endif

#ifdef REMOVEQUEUE
#define MAXBUFFERLEN 2097152 //2M
#define EACHFRAMEDATALEN 655360 //640k
const uint8_t PROTOCOL_HEADER_V2_SIZE = 12;

enum {
    DATA_ERROR = -2,
    DATA_LESS = -1, 
    DATA_UNSTREAM = 0,
    DATA_CONTROLSTREAM = 1,
    DATA_STREAM = 2
};

enum ServiceType {
  kAudio = 0x0A,
  kMobileNav = 0x0B
};

enum {
  FRAME_DATA_HEART_BEAT = 0x00,
  FRAME_DATA_START_SERVICE = 0x01,
  FRAME_DATA_START_SERVICE_ACK = 0x02,
  FRAME_DATA_START_SERVICE_NACK = 0x03,
  FRAME_DATA_END_SERVICE = 0x04,
  FRAME_DATA_END_SERVICE_ACK = 0x05,
  FRAME_DATA_END_SERVICE_NACK = 0x06,
  FRAME_DATA_SERVICE_DATA_ACK = 0xFE,
  FRAME_DATA_HEART_BEAT_ACK = 0xFF,
  FRAME_DATA_SINGLE = 0x00,
  FRAME_DATA_FIRST = 0x00,
  FRAME_DATA_LAST_CONSECUTIVE = 0x00,
  FRAME_DATA_MAX_CONSECUTIVE = 0xFF,
  FRAME_DATA_MAX_VALUE = 0xFF
};

enum {
  FRAME_TYPE_CONTROL = 0x00,
  FRAME_TYPE_SINGLE = 0x01,
  FRAME_TYPE_FIRST = 0x02,
  FRAME_TYPE_CONSECUTIVE = 0x03,
  FRAME_TYPE_MAX_VALUE = 0x07
};

enum {
  PROTOCOL_VERSION_1 = 0x01,
  PROTOCOL_VERSION_2 = 0x02,
  PROTOCOL_VERSION_3 = 0x03,
  PROTOCOL_VERSION_4 = 0x04,
  PROTOCOL_VERSION_MAX = 0x0F
};

typedef struct{
    uint8_t version;
    uint8_t frameType;
    uint8_t serviceType;
    uint8_t frameData;
    uint32_t messageId;
    uint32_t dataSize;
}Frame;

typedef struct{
    unsigned char sbuffer[EACHFRAMEDATALEN];
    int nbufferlen;
}FRAMEDATALIST;
#endif

class UsbConnection : public Connection {
 public:
  UsbConnection(const DeviceUID& device_uid,
                const ApplicationHandle& app_handle,
                TransportAdapterController* controller,
                const UsbHandlerSptr usb_handler,
                PlatformUsbDevice* device);
  bool Init();
  virtual ~UsbConnection();

 protected:
  virtual TransportAdapter::Error SendData(
      ::protocol_handler::RawMessagePtr message);
  virtual TransportAdapter::Error Disconnect();

 private:
  void PopOutMessage();
  bool PostInTransfer();
  bool PostOutTransfer();
  void OnInTransfer(struct libusb_transfer*);
  void OnOutTransfer(struct libusb_transfer*);
  void Finalise();
  void AbortConnection();
  bool FindEndpoints();

  const DeviceUID device_uid_;
  const ApplicationHandle app_handle_;
  TransportAdapterController* controller_;
  UsbHandlerSptr usb_handler_;
  libusb_device* libusb_device_;
  libusb_device_handle* device_handle_;
  uint8_t in_endpoint_;
  uint16_t in_endpoint_max_packet_size_;
  uint8_t out_endpoint_;
  uint16_t out_endpoint_max_packet_size_;
  unsigned char* in_buffer_;
  libusb_transfer* in_transfer_;
  libusb_transfer* out_transfer_;

  std::list<protocol_handler::RawMessagePtr> out_messages_;
  protocol_handler::RawMessagePtr current_out_message_;
  sync_primitives::Lock out_messages_mutex_;
  size_t bytes_sent_;
  bool disconnecting_;
  bool waiting_in_transfer_cancel_;
  bool waiting_out_transfer_cancel_;
  friend void InTransferCallback(struct libusb_transfer*);
  friend void OutTransferCallback(struct libusb_transfer*);

#ifdef REMOVEQUEUE
private:
  class SocketConnectionDelegate : public threads::ThreadDelegate {
   public:
    explicit SocketConnectionDelegate(UsbConnection* connection);
    void threadMain() OVERRIDE;
    void exitThreadMain() OVERRIDE;

   private:
    UsbConnection* connection_;
  };
  void threadMain();
  int deserialize(unsigned char *in_buffer,int nrecvlen,Frame &frame,int &nheadlen);
  int StartStreamer(int nrecvlen);
  bool WriteDataToQueue(unsigned char *buffer, int nbufferlen);
  int mystrstr(unsigned char *buffer, int nbufferlen);
  uint32_t read_be_uint32(const uint8_t* const data);

  threads::Thread* thread_;
  std::list<FRAMEDATALIST>framemessage_;
  FRAMEDATALIST current_frame_message_;
  bool is_video_start;
  bool is_audio_start;
  int nvideoremainbufferlen;
  unsigned char *svideowritebuffer;
  int nvideoreadywritelen;
  int nvideoreadyscanlen;
  int nvideoreadyscannum;
  sync_primitives::Lock mobile_messages_mutex_;
  sync_primitives::ConditionalVariable mobile_messages_mutex_condition_;
#endif
};
}  // namespace transport_adapter
}  // namespace transport_manager
#endif  // SRC_COMPONENTS_TRANSPORT_MANAGER_INCLUDE_TRANSPORT_MANAGER_USB_LIBUSB_USB_CONNECTION_H_
