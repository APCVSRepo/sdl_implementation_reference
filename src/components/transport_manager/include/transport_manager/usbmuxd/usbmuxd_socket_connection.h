/*
 * \file Usbmuxd_socket_connection.h
 * \brief UsbmuxdSocketConnection class header file.
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

#ifndef SRC_COMPONENTS_TRANSPORT_MANAGER_INCLUDE_TRANSPORT_MANAGER_USBMUXD_USBMUXD_SOCKET_CONNECTION_H_
#define SRC_COMPONENTS_TRANSPORT_MANAGER_INCLUDE_TRANSPORT_MANAGER_USBMUXD_USBMUXD_SOCKET_CONNECTION_H_

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>

#include "transport_manager/transport_adapter/threaded_socket_connection.h"


namespace transport_manager {
namespace transport_adapter {
	
#define TIMEOUT_ONECONNECT 60*60*24*30//s
#define RECVBUFFERLEN 1048576
class TransportAdapterController;

/**
 * @brief Class responsible for communication over Usbmuxd sockets.
 */
class UsbmuxdSocketConnection : public Connection {
 public:
  /**
   * @brief Constructor.
   *
   * @param device_uid Device unique identifier.
   * @param app_handle Handle of application.
   * @param controller Pointer to the Usbmuxd device adapter controller.
   */
  UsbmuxdSocketConnection(const DeviceUID& device_uid,
										   const ApplicationHandle& app_handle,
										   TransportAdapterController* controller,
										   const int appport);

  /**
   * @brief Destructor.
   */
  virtual ~UsbmuxdSocketConnection();

 protected:
  /**
   * @brief
   */
 virtual bool Establish(ConnectError** error);

public:
 /**
  * @brief Send data frame.
  *
  * @param message Smart pointer to the raw message.
  *
  * @return Error Information about possible reason of sending data failure.
  */
 TransportAdapter::Error SendData(::protocol_handler::RawMessagePtr message);

 /**
  * @brief Disconnect the current connection.
  *
  * @return Error Information about possible reason of Disconnect failure.
  */
 TransportAdapter::Error Disconnect();

 /**
  * @brief Start thread creation.
  *
  * @return Information about possible reason of thread creation error.
  */
 TransportAdapter::Error Start();

 /**
  * @brief Checks is queue with frames to send empty or not.
  *
  * @return Information about queue is empty or not.
  */
 bool IsFramesToSendQueueEmpty() const;

 /**
  * @brief Set variable that hold socket No.
  */
 void set_socket(int socket) {
   socket_ = socket;
 }

 int usbmuxd_send_data(int connect_sfd,uint8_t* sendsrcdata,int nsrcsendlen);
 int usbmuxd_recv_data(int connect_sfd,char* data,int datasize);
 int usbmuxd_recv_head(int connect_sfd); 
 bool explainrecvdata() ;
protected:


 /**
  * @brief Return pointer to the device adapter controller.
  */
 TransportAdapterController* controller() {
   return controller_;
 }

 /**
  * @brief Return device unique identifier.
  */
 DeviceUID device_handle() const {
   return device_uid_;
 }

 /**
  * @brief Return handle of application.
  */
 ApplicationHandle application_handle() const {
   return app_handle_;
 }

private:
 class UsbmuxdSocketConnectionDelegate : public threads::ThreadDelegate {
  public:
   explicit UsbmuxdSocketConnectionDelegate(UsbmuxdSocketConnection* connection);
   void threadMain() OVERRIDE;
   void exitThreadMain() OVERRIDE;

  private:
   UsbmuxdSocketConnection* connection_;
 };

 int read_fd_;
 int write_fd_;
 char recv_buffer[RECVBUFFERLEN];
 char recv_buffer_nextframe[RECVBUFFERLEN];
 char srealsenddata[RECVBUFFERLEN];
 int recvbufferlen;
 int recvbufferlen_nextframe;
 void threadMain();
 void Transmit();
 void Finalize();
 TransportAdapter::Error Notify() const;
 bool Receive();
 bool Send();
 void Abort();

 TransportAdapterController* controller_;
 /**
  * @brief Frames that must be sent to remote device.
  **/
 typedef std::queue<protocol_handler::RawMessagePtr> FrameQueue;
 FrameQueue frames_to_send_;
 mutable sync_primitives::Lock frames_to_send_mutex_;
 int socket_;
 const int myport;
 bool terminate_flag_;
 bool unexpected_disconnect_;
 const DeviceUID device_uid_;
 const ApplicationHandle app_handle_;
 threads::Thread* thread_;
};
/**
  * @brief Class responsible for communication over sockets that originated by
  * server.
  */
 class UsbmuxdServerOiginatedSocketConnection : public UsbmuxdSocketConnection {
  public:
   /**
	* @brief Constructor.
	*
	* @param device_uid Device unique identifier.
	* @param app_handle Handle of application.
	* @param controller Pointer to the device adapter controller.
	*/
   UsbmuxdServerOiginatedSocketConnection(const DeviceUID& device_uid,
									  const ApplicationHandle& app_handle,
									  TransportAdapterController* controller); 
   /**
	* @brief Destructor.
	*/
   virtual ~UsbmuxdServerOiginatedSocketConnection();
 
  protected:
   /**
	* @brief
	*/
   virtual bool Establish(ConnectError** error);
};

}  // namespace transport_adapter
}  // namespace transport_manager

#endif  // SRC_COMPONENTS_TRANSPORT_MANAGER_INCLUDE_TRANSPORT_MANAGER_Usbmuxd_Usbmuxd_SOCKET_CONNECTION_H_
