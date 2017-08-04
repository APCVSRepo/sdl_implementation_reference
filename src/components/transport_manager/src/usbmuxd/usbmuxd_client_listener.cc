/*
 *
 * Copyright (c) 2013, Ford Motor Company
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

#include "transport_manager/usbmuxd/usbmuxd_client_listener.h"

#include <memory.h>
#include <signal.h>
#include <errno.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/sysctl.h>
#include <sys/socket.h>
#ifdef __linux__
#include <linux/tcp.h>
#else  // __linux__
#include <sys/time.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netinet/tcp_var.h>
#endif  // __linux__

#include <sstream>

#include "utils/logger.h"
#include "utils/threads/thread.h"
#include "transport_manager/transport_adapter/transport_adapter_controller.h"
#include "transport_manager/usbmuxd/usbmuxd_device.h"
#include "transport_manager/usbmuxd/usbmuxd_socket_connection.h"


#ifdef __CPLUSPLUS
extern "C"{
#endif
#include "usbmuxd-proto.h"
#include "usbmuxd.h"
#ifdef __CPLUSPLUS
}
#endif


namespace transport_manager {
namespace transport_adapter {

CREATE_LOGGERPTR_GLOBAL(logger_, "TransportManager")

UsbmuxdClientListener::UsbmuxdClientListener(TransportAdapterController* controller,
                                     const uint16_t port,
                                     const bool enable_keepalive)
    : port_(port)
    , enable_keepalive_(enable_keepalive)
    , controller_(controller)
    , thread_(0)
    , socket_(-1)
    , thread_stop_requested_(false) {
  thread_ = threads::CreateThread("UsbmuxdClientListener",
                                  new ListeningThreadDelegate(this));
}

TransportAdapter::Error UsbmuxdClientListener::Init() {
  LOG4CXX_AUTO_TRACE(logger_);
  thread_stop_requested_ = false;

  socket_ = socket(AF_INET, SOCK_STREAM, 0);
  if (-1 == socket_) {
    LOG4CXX_ERROR_WITH_ERRNO(logger_, "Failed to create socket");
    return TransportAdapter::FAIL;
  }

  sockaddr_in server_address = {0};
  server_address.sin_family = AF_INET;
  server_address.sin_port = htons(port_);
  server_address.sin_addr.s_addr = INADDR_ANY;

  int optval = 1;
  setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));

  if (bind(socket_,
           reinterpret_cast<sockaddr*>(&server_address),
           sizeof(server_address)) != 0) {
    LOG4CXX_ERROR_WITH_ERRNO(logger_, "bind() failed");
    return TransportAdapter::FAIL;
  }

  const int kBacklog = 128;
  if (0 != listen(socket_, kBacklog)) {
    LOG4CXX_ERROR_WITH_ERRNO(logger_, "listen() failed");
    return TransportAdapter::FAIL;
  }
  return TransportAdapter::OK;
}

void UsbmuxdClientListener::Terminate() {
  LOG4CXX_AUTO_TRACE(logger_);
  if (socket_ == -1) {
    LOG4CXX_WARN(logger_, "Socket has been closed");
    return;
  }
  if (shutdown(socket_, SHUT_RDWR) != 0) {
    LOG4CXX_ERROR_WITH_ERRNO(logger_, "Failed to shutdown socket");
  }
  if (close(socket_) != 0) {
    LOG4CXX_ERROR_WITH_ERRNO(logger_, "Failed to close socket");
  }
  socket_ = -1;
}

bool UsbmuxdClientListener::IsInitialised() const {
  return thread_;
}

UsbmuxdClientListener::~UsbmuxdClientListener() {
  LOG4CXX_AUTO_TRACE(logger_);
  StopListening();
  delete thread_->delegate();
  threads::DeleteThread(thread_);
  Terminate();
}

void UsbmuxdClientListener::Loop() {
  LOG4CXX_AUTO_TRACE(logger_);
  int devlist_count = 0,ndevicenum = 0;
  usbmuxd_device_info_t device_info ;
  //libusbmuxd_set_debug_level(3) ;
  time_t nlasttime = 0,nnowtime = 0;
  nlasttime = nnowtime = time(NULL);
  while (!thread_stop_requested_) {
	nnowtime = time(NULL);
	memset(&device_info,0,sizeof(usbmuxd_device_info_t));
	usbmuxd_device_info_t *devicelist = NULL;	
	devlist_count = usbmuxd_get_device_list(&devicelist);
	if((devlist_count < 1) && ((nnowtime - nlasttime) > 1)){
	  nlasttime = nnowtime;
	  LOG4CXX_ERROR(logger_,"Failed to get usbmuxd device \n" << devlist_count);	
          usleep(100);
	  continue;
	}
	else{
	  nlasttime = nnowtime;
	}
	
	std::vector<DeviceUID> DeviceList;
	for(ndevicenum = 0;ndevicenum < devlist_count;ndevicenum ++){				
 	  int appport = 20001;
   	  device_info =  devicelist[ndevicenum];
   	  char uid[100] = "";
   	  char device_name[100] = "";

	  for(;appport < 20004;appport ++){
		  sprintf(uid,"%d",appport);
		  memcpy(uid+strlen(uid),device_info.udid,strlen(device_info.udid));
		  sprintf(device_name,"%s",uid);
		  DeviceUID devi;
		  devi = uid;
		  DeviceList.push_back(devi);
		  if(controller_->IsSameDevice(uid)){
			continue;
		  }

		  UsbmuxdDevice* Usbmuxd_device = new UsbmuxdDevice(uid, device_name);
		  DeviceSptr device = controller_->AddDevice(Usbmuxd_device);  
		  const int apphandle = device_info.handle;
		  const ApplicationHandle app_handle = Usbmuxd_device->AddIncomingApplication(apphandle);
		  Usbmuxd_device = static_cast<UsbmuxdDevice*>(device.get());
		  UsbmuxdSocketConnection* connection(new UsbmuxdSocketConnection(
			device->unique_device_id(), app_handle, controller_,appport));
		  const TransportAdapter::Error error = connection->Start();
		  if (error != TransportAdapter::OK) {
			delete connection;
		  }
	  }
	}
	controller_->RemoveUnFindDevice(DeviceList);
	usbmuxd_device_list_free(&devicelist);
	devicelist = NULL;
	usleep(100);
  }
}

void UsbmuxdClientListener::StopLoop() {
  LOG4CXX_AUTO_TRACE(logger_);
  thread_stop_requested_ = true;
  // We need to connect to the listening socket to unblock accept() call
  int byesocket = socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in server_address = {0};
  server_address.sin_family = AF_INET;
  server_address.sin_port = htons(port_);
  server_address.sin_addr.s_addr = INADDR_ANY;
  connect(byesocket,
          reinterpret_cast<sockaddr*>(&server_address),
          sizeof(server_address));
  shutdown(byesocket, SHUT_RDWR);
  close(byesocket);
}

TransportAdapter::Error UsbmuxdClientListener::StartListening() {
  LOG4CXX_AUTO_TRACE(logger_);
  if (thread_->is_running()) {
    LOG4CXX_WARN(
        logger_,
        "TransportAdapter::BAD_STATE. Listener has already been started");
    return TransportAdapter::BAD_STATE;
  }

  if (!thread_->start()) {
    LOG4CXX_ERROR(logger_, "Usbmuxd client listener thread start failed");
    return TransportAdapter::FAIL;
  }
  LOG4CXX_INFO(logger_, "Usbmuxd client listener has started successfully");
  return TransportAdapter::OK;
}

void UsbmuxdClientListener::ListeningThreadDelegate::exitThreadMain() {
  parent_->StopLoop();
}

void UsbmuxdClientListener::ListeningThreadDelegate::threadMain() {
  parent_->Loop();
}

UsbmuxdClientListener::ListeningThreadDelegate::ListeningThreadDelegate(
    UsbmuxdClientListener* parent)
    : parent_(parent) {}

TransportAdapter::Error UsbmuxdClientListener::StopListening() {
  LOG4CXX_AUTO_TRACE(logger_);
  if (!thread_->is_running()) {
    LOG4CXX_DEBUG(logger_, "UsbmuxdClientListener is not running now");
    return TransportAdapter::BAD_STATE;
  }

  thread_->join();

  LOG4CXX_INFO(logger_, "Usbmuxd client listener has stopped successfully");
  return TransportAdapter::OK;
}

}  // namespace transport_adapter
}  // namespace transport_manager
