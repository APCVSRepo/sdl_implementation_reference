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
#include "transport_manager/usbmuxd/usbmuxd_socket_connection.h"

#include <algorithm>
#include <fcntl.h>
#include <memory.h>
#include <signal.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#include "utils/logger.h"
#include "utils/threads/thread.h"
#include "transport_manager/usbmuxd/usbmuxd_device.h"
#include "transport_manager/transport_adapter/transport_adapter_controller.h"

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
	
struct usbmuxd_header_real {
	uint32_t headerframe;	//&&&&
	uint32_t version;	// protocol version 
	uint32_t message;	// message type 
	uint32_t tag;		// responses to this query will echo back this tag	
	uint32_t length;	
	
} __attribute__((__packed__));

UsbmuxdSocketConnection::UsbmuxdSocketConnection(const DeviceUID& device_uid,
                                         const ApplicationHandle& app_handle,
                                         TransportAdapterController* controller,
                                         const int appport)
		: read_fd_(-1)
		, write_fd_(-1)
		, controller_(controller)
		, frames_to_send_()
		, frames_to_send_mutex_()
		, socket_(-1)
		, myport(appport)
		, terminate_flag_(false)
		, unexpected_disconnect_(false)
		, device_uid_(device_uid)
		, app_handle_(app_handle)
		, thread_(NULL) {
		
	  const std::string thread_name = std::string("Socket ") + device_handle();
	  thread_ = threads::CreateThread(thread_name.c_str(),
									  new UsbmuxdSocketConnectionDelegate(this));
}
		
UsbmuxdSocketConnection::~UsbmuxdSocketConnection() {
	LOG4CXX_AUTO_TRACE(logger_);
	Disconnect();
	thread_->join();
	delete thread_->delegate();
	threads::DeleteThread(thread_);

	if (-1 != read_fd_) {
		close(read_fd_);
	}
	if (-1 != write_fd_) {
		close(write_fd_);
	}
}
void UsbmuxdSocketConnection::Abort() {
	LOG4CXX_AUTO_TRACE(logger_);
	unexpected_disconnect_ = true;
	terminate_flag_ = true;
}

TransportAdapter::Error UsbmuxdSocketConnection::Start() {
	LOG4CXX_AUTO_TRACE(logger_);
	int fds[2];
	const int pipe_ret = pipe(fds);
	if (0 == pipe_ret) {
		LOG4CXX_DEBUG(logger_, "pipe created");
		read_fd_ = fds[0];
		write_fd_ = fds[1];
	} 
	else {
		LOG4CXX_ERROR(logger_, "pipe creation failed");
		return TransportAdapter::FAIL;
	}
	const int fcntl_ret = fcntl(read_fd_, F_SETFL, fcntl(read_fd_, F_GETFL) | O_NONBLOCK);
	if (0 != fcntl_ret) {
		LOG4CXX_ERROR(logger_, "fcntl failed");
		return TransportAdapter::FAIL;
	}

	if (!thread_->start()) {
		LOG4CXX_ERROR(logger_, "thread creation failed");
		return TransportAdapter::FAIL;
	}

	LOG4CXX_INFO(logger_, "thread created");
	return TransportAdapter::OK;
}

void UsbmuxdSocketConnection::Finalize() {
	LOG4CXX_AUTO_TRACE(logger_);
	if (unexpected_disconnect_) {
		LOG4CXX_DEBUG(logger_, "unexpected_disconnect");
		controller_->ConnectionAborted(
	    	device_handle(), application_handle(), CommunicationError());
	} else {
		LOG4CXX_DEBUG(logger_, "not unexpected_disconnect");
		controller_->ConnectionFinished(device_handle(), application_handle());
	}
	close(socket_);
}

TransportAdapter::Error UsbmuxdSocketConnection::Notify() const {
	LOG4CXX_AUTO_TRACE(logger_);
	if (-1 == write_fd_) {
		LOG4CXX_ERROR_WITH_ERRNO(
	    logger_, "Failed to wake up connection thread for connection " << this);
		LOG4CXX_TRACE(logger_, "exit with TransportAdapter::BAD_STATE");
		return TransportAdapter::BAD_STATE;
	}
	uint8_t c = 0;
	if (1 != write(write_fd_, &c, 1)) {
		LOG4CXX_ERROR_WITH_ERRNO(
	    logger_, "Failed to wake up connection thread for connection " << this);
		return TransportAdapter::FAIL;
	}
	return TransportAdapter::OK;
}

TransportAdapter::Error UsbmuxdSocketConnection::SendData(
    ::protocol_handler::RawMessagePtr message) {
    
	LOG4CXX_AUTO_TRACE(logger_);
	sync_primitives::AutoLock auto_lock(frames_to_send_mutex_);
	frames_to_send_.push(message);
	return Notify();
}

TransportAdapter::Error UsbmuxdSocketConnection::Disconnect() {	
	LOG4CXX_AUTO_TRACE(logger_);
	terminate_flag_ = true;
	return Notify();
}


bool UsbmuxdSocketConnection::IsFramesToSendQueueEmpty() const {
	// Check Frames queue is empty or not
	sync_primitives::AutoLock auto_lock(frames_to_send_mutex_);
	return frames_to_send_.empty();
}

void UsbmuxdSocketConnection::threadMain() {
	LOG4CXX_AUTO_TRACE(logger_);
	controller_->ConnectionCreated(this, device_handle(), application_handle());
	ConnectError* connect_error = NULL;
	time_t nlasttime = 0,nnowtime = 0;
	nlasttime = nnowtime = time(NULL);

	LOG4CXX_DEBUG(logger_, "Connection established");
	while (!terminate_flag_) {
		nnowtime = time(NULL);
		DeviceSptr device = controller()->FindDevice(device_handle());
		if (!device.valid()) {
			Abort();
		}

		if (!Establish(&connect_error)) {
			if((nnowtime - nlasttime) > 1){
				nlasttime = nnowtime;
			}
			delete connect_error;	  
			usleep(100);	
			LOG4CXX_ERROR(logger_,"connect port " <<  myport << "failed");
		}
		else{
			LOG4CXX_ERROR(logger_,"connect port " <<  myport << "success");
		  	break;
		}
	}

	const DeviceUID device_uid_1 = device_handle();
	controller_->ConnectDone(device_handle(), application_handle());
	while (!terminate_flag_) {	
		DeviceSptr device = controller()->FindDevice(device_handle());
		if (!device.valid()) {
	  		Abort();
		}
		Transmit();
	}

	LOG4CXX_DEBUG(logger_, "Connection is to finalize");
	Finalize();
	sync_primitives::AutoLock auto_lock(frames_to_send_mutex_);
	while (!frames_to_send_.empty() && !terminate_flag_) {
		LOG4CXX_INFO(logger_, "removing message");
		::protocol_handler::RawMessagePtr message = frames_to_send_.front();
		frames_to_send_.pop();
		controller_->DataSendFailed(
		    device_handle(), application_handle(), message, DataSendError());
	}
}

bool UsbmuxdSocketConnection::Establish(ConnectError** error) {
	std::string str;
	DeviceSptr device = controller()->FindDevice(device_handle());
	if (!device.valid()) {
		LOG4CXX_ERROR(logger_, "Device " << device_handle() << " not found");
		*error = new ConnectError();
		Abort();
		return false;
	}

	UsbmuxdDevice* Usbmuxd_device = static_cast<UsbmuxdDevice*>(device.get());
	ApplicationHandle tmp = application_handle();
	uint32_t handle = Usbmuxd_device->applications_[tmp].apphandle;

	const int socket = usbmuxd_connect(handle,myport);
	if (socket < 0) {
		*error = new ConnectError();
		return false;
	}

	set_socket(socket);
	Usbmuxd_device->applications_[tmp].socket = socket;
	return true;
}

void UsbmuxdSocketConnection::Transmit() {
	LOG4CXX_AUTO_TRACE(logger_);
	const nfds_t kPollFdsSize = 2;
	pollfd poll_fds[kPollFdsSize];
	poll_fds[0].fd = socket_;

	const bool is_queue_empty_on_poll = IsFramesToSendQueueEmpty();

	poll_fds[0].events = POLLIN | POLLPRI | (is_queue_empty_on_poll ? 0 : POLLOUT);
	poll_fds[1].fd = read_fd_;
	poll_fds[1].events = POLLIN | POLLPRI;

	LOG4CXX_DEBUG(logger_, "poll " << this);
	if (-1 == poll(poll_fds, kPollFdsSize, -1)) {
		LOG4CXX_ERROR_WITH_ERRNO(logger_, "poll failed for connection " << this);
		Abort();
		return;
	}
	LOG4CXX_DEBUG(logger_,
	            "poll is ok " << this << " revents0: " << std::hex
	                          << poll_fds[0].revents << " revents1:" << std::hex
	                          << poll_fds[1].revents);
	// error check
	if (0 != (poll_fds[1].revents & (POLLERR | POLLHUP | POLLNVAL))) {
		LOG4CXX_ERROR(logger_,
		              "Notification pipe for connection " << this << " terminated");
		Abort();
		return;
	}

	if (poll_fds[0].revents & (POLLERR | POLLHUP | POLLNVAL)) {
		LOG4CXX_WARN(logger_, "Connection " << this << " terminated");
		Abort();
		return;
	}

	// clear notifications
	char buffer[256];
	ssize_t bytes_read = -1;

	do {
		bytes_read = read(read_fd_, buffer, sizeof(buffer));
		DeviceSptr device = controller()->FindDevice(device_handle());
		if (!device.valid()) {
		  	Abort();
			return;
		}
	} while (bytes_read > 0);
	if ((bytes_read < 0) && (EAGAIN != errno)) {
		LOG4CXX_ERROR_WITH_ERRNO(logger_, "Failed to clear notification pipe");
		LOG4CXX_ERROR_WITH_ERRNO(logger_, "poll failed for connection " << this);
		Abort();
		return;
	}

	const bool is_queue_empty = IsFramesToSendQueueEmpty();
	// Send data if possible
	if (!is_queue_empty && (poll_fds[0].revents | POLLOUT)) {
		LOG4CXX_DEBUG(logger_, "frames_to_send_ not empty() ");

		// send data
		const bool send_ok = Send();
		if (!send_ok) {
			LOG4CXX_ERROR(logger_, "Send() failed ");
			Abort();
			return;
		}
	}

	// receive data

	if (poll_fds[0].revents & (POLLIN | POLLPRI)) {
		const bool receive_ok = Receive();
		if (!receive_ok) {
			LOG4CXX_ERROR(logger_, "Receive() failed ");
			Abort();
			return;
		}
	}
  
}

/*recv head*/
int UsbmuxdSocketConnection::usbmuxd_recv_head(int connect_sfd){	
	if(connect_sfd < 1){		
		LOG4CXX_ERROR(logger_,"connect_sfd is error,not recv head data\n");		
		return -1;	
	}		

	char buffer[50] = "",buffertmp[50] = "";			  
	int nlen = 0,nindex = 0,tmp = 0;	
	int recv_bytes = 0,nlength = 0,nreallen = 0,nneedrecvheadlen = 0,headdatalen = 0;
	unsigned int unlength = 0;	
	time_t nlasttime = 0,nnowtime = 0;
	usbmuxd_header_real hdr;	

recv_head:
	memset(&hdr,0,sizeof(usbmuxd_header_real));
   	nneedrecvheadlen = sizeof(usbmuxd_header_real);

	if(recvbufferlen_nextframe > 0 && recvbufferlen_nextframe <= (int)sizeof(usbmuxd_header_real)){
		memcpy(&hdr,recv_buffer_nextframe,recvbufferlen_nextframe);
		nneedrecvheadlen = sizeof(usbmuxd_header_real) - recvbufferlen_nextframe;
		headdatalen = recvbufferlen_nextframe;
		recvbufferlen_nextframe = 0;
		memset(recv_buffer_nextframe,0,RECVBUFFERLEN);
	}
	else if(recvbufferlen_nextframe > (int)sizeof(usbmuxd_header_real)){
		memcpy(&hdr,recv_buffer_nextframe,sizeof(usbmuxd_header_real));
		nneedrecvheadlen = 0;
		headdatalen = sizeof(usbmuxd_header_real);
	}
  
	recv_bytes = recv(connect_sfd, &hdr+headdatalen, nneedrecvheadlen,MSG_DONTWAIT);	
	if (recv_bytes <= 0){			
		return recv_bytes;
	}
  	else if(recv_bytes < nneedrecvheadlen){
    	nlasttime = nnowtime = time(NULL);
		while((recv_bytes < nneedrecvheadlen)  && (terminate_flag_ == false)){
			nnowtime = time(NULL);
			tmp = nneedrecvheadlen - recv_bytes;
			nlen = recv(connect_sfd, buffer, tmp, MSG_DONTWAIT);	
			if(nlen > 0){				
				memcpy(&hdr+recv_bytes,buffer,nlen);	
				recv_bytes += nlen;
				nlasttime = nnowtime;		
		  	}	  
		  	else if(nlen < 0){
				if((nnowtime - nlasttime) > 3){
					LOG4CXX_ERROR(logger_,"recv head data timeout");
					break;
				}
		
        		usleep(10);
	    		continue;
      		}
      		else{
				LOG4CXX_ERROR(logger_,"recv back 0,connect is abort");
				return nlen;
      		}
    	}
	
    	if(recv_bytes < nneedrecvheadlen){	
      		LOG4CXX_ERROR(logger_,"recv head size is too small");	
      		return -1;	
		}
	}		

	//Check head
	if(hdr.headerframe != 0x26262626){
		LOG4CXX_ERROR(logger_,"recv head error");
		memset(recv_buffer_nextframe,0,RECVBUFFERLEN);
		recvbufferlen_nextframe = 0;
		headdatalen = 0;
		if(terminate_flag_ == true){
		  return -1;
		} 
		goto recv_head;
	}

	if(recvbufferlen_nextframe > (int)sizeof(usbmuxd_header_real)){
		memcpy(recv_buffer,recv_buffer_nextframe,recvbufferlen_nextframe - sizeof(usbmuxd_header_real));
		recvbufferlen = recvbufferlen_nextframe - sizeof(usbmuxd_header_real);
	}
	recvbufferlen_nextframe = 0;
	memset(recv_buffer_nextframe,0,RECVBUFFERLEN);

	nlength = hdr.length;	
	unlength = nlength;		
	nreallen = unlength/0x1000000 ;
	nreallen += ((unlength%0x1000000)/0x10000)*0x100 ;
	nreallen += ((unlength%0x10000)/0x100)*0x10000 ;
	nreallen += ((unlength%0x100))*0x1000000;
	sprintf(buffertmp,"%08X",nreallen);		
			
	nlength = 0;			
	for(nindex = 0;nindex < 8;nindex ++){				
		if(buffertmp[nindex] >= '0' && buffertmp[nindex] <= '9')					
	  		nlength = nlength*0x10 + (buffertmp[nindex]-'0');				
		else					
	  		nlength = nlength*0x10 + (buffertmp[nindex]-'A' + 0x0A);			
	}		

	return nlength;		
}	

/*recv data*/
int UsbmuxdSocketConnection::usbmuxd_recv_data(int connect_sfd,char *data,int datasize){		
	int recv_bytes = 0,recvsize = 0;	
	time_t lasttime,nowtime;

	lasttime = nowtime = time(NULL);

	do {		
		nowtime = time(NULL);
		recv_bytes = recv(connect_sfd,data+recvsize, datasize-recvsize, MSG_DONTWAIT);  
		if(recv_bytes <= 0){
			if((nowtime - lasttime > 3)){
				LOG4CXX_ERROR(logger_,"Don't recv data ok in 3s");
				return -1;
		  	}
			
		  	usleep(10);
		  	continue;
		}     
		lasttime = nowtime;
		recvsize += recv_bytes;	
	} while ((recvsize < datasize) && (terminate_flag_ == false));
	return recvsize;
}

/*send data*/
int  UsbmuxdSocketConnection::usbmuxd_send_data(int connect_sfd,uint8_t* sendsrcdata,int nsrcsendlen){	
	if(connect_sfd < 1 || nsrcsendlen < 1 || sendsrcdata == NULL){		
		return nsrcsendlen;	
	}	

	int tmp = 0,nindex = 0,nrealsendlen = 0;				
	usbmuxd_header_real head;	
	memset(&head,0,sizeof(usbmuxd_header_real));	

	head.version = 0x01000000;	
	head.tag = 0;	
	head.message = 0x65000000;	

	tmp = nsrcsendlen + 4;	
	nrealsendlen = 0;	
	memcpy(srealsenddata,&head,sizeof(usbmuxd_header_real)-4);	
	nrealsendlen += sizeof(usbmuxd_header_real)-4;	
	for(nindex = 3;nindex >= 0;nindex --){		
		srealsenddata[nrealsendlen+nindex] = tmp%0x100;		
		tmp /= 0x100;	
	}	
	nrealsendlen += 4;	

	tmp = nsrcsendlen;	
	for(nindex = 3;nindex >= 0;nindex --){		
		srealsenddata[nrealsendlen+nindex] = tmp%0x100;		
		tmp /= 0x100;	
	}	
	nrealsendlen += 4;	

	memcpy(srealsenddata+nrealsendlen,sendsrcdata,nsrcsendlen);
	nrealsendlen += nsrcsendlen;	
	::send(socket_,srealsenddata, nrealsendlen, 0);

	return nsrcsendlen;
}

bool UsbmuxdSocketConnection::explainrecvdata() {
	if(recv_buffer == NULL || recvbufferlen <= 8){
		LOG4CXX_ERROR(logger_,"recv data,but the data length is 0 or the data is NULL or strlen of recv_buffer < recvbufferlen");
		return false;
	}

	bool isfindtail = false;
	int nremainlen = 0;
	unsigned long long llcrc = 0,llcrcb = 0;	

	recvbufferlen_nextframe = 0;
	memset(recv_buffer_nextframe,0,RECVBUFFERLEN);
	//tail check
	for(int nindex = recvbufferlen - 1;nindex > 4;nindex --){
		if(recv_buffer[nindex] == '@' && recv_buffer[nindex - 1] == '@' && recv_buffer[nindex - 2] == '@' && recv_buffer[nindex - 3] == '@'){
			isfindtail = true;
			break;
		}
		nremainlen ++;
	}
	
	if(isfindtail == false){
		LOG4CXX_ERROR(logger_,"usbmuxd transfer data erro,cannot find data of tail");
		recvbufferlen_nextframe = 0;
		memset(recv_buffer_nextframe,0,RECVBUFFERLEN);	
		return false;
	}	  
	//data len error				  
	if(nremainlen > 0){				  
		LOG4CXX_ERROR(logger_,"usbmuxd transfer data erro");				  
		recvbufferlen_nextframe = nremainlen; 				  
		memcpy(recv_buffer_nextframe,recv_buffer+recvbufferlen-nremainlen,recvbufferlen_nextframe); 				  
		return false; 			  
	}

    	// crc check
    	if(recvbufferlen > 8){
		for(int i = 0;i<recvbufferlen-8;i++){			    		
			llcrc += recv_buffer[i] & 0xFF;			    		
			llcrc %= 0xFFFFFFFF;			  		
		}
		
		llcrcb += (recv_buffer[recvbufferlen-8]&0xFF)*0x1000000;llcrcb %= 0xFFFFFFFF;
		llcrcb += (recv_buffer[recvbufferlen-7]&0xFF)*0x10000;llcrcb %= 0xFFFFFFFF;			  		
		llcrcb += (recv_buffer[recvbufferlen-6]&0xFF)*0x100;llcrcb %= 0xFFFFFFFF;			  		
		llcrcb += (recv_buffer[recvbufferlen-5]&0xFF);llcrcb %= 0xFFFFFFFF;			  		
		if(llcrc != llcrcb){			    		
			LOG4CXX_ERROR(logger_,"usbmuxd transfer data llcrc erro");				  		
			return false;
		}
		else{
			return true;
		}
	}
	else{
		return false;
	}
}

bool UsbmuxdSocketConnection::Receive() {
	LOG4CXX_AUTO_TRACE(logger_);
	bool explainflag = false;
	char buffer[4096];
	int bytes_read = -1,bytes_data = -1,needrecvsize = 0;

	memset(recv_buffer,0,RECVBUFFERLEN);
	memset(recv_buffer_nextframe,0,RECVBUFFERLEN);;
	recvbufferlen = 0;
	recvbufferlen_nextframe = 0;
	do {
		memset(buffer,0,4096);	
		if(needrecvsize <= 0){
			memset(recv_buffer,0,RECVBUFFERLEN);
			recvbufferlen = 0;
			bytes_data = usbmuxd_recv_head(socket_);
			bytes_read = bytes_data;
			needrecvsize = bytes_read;	
			if(recvbufferlen >=  needrecvsize && needrecvsize > 0){
				needrecvsize = 0;
			}
		}

		if(terminate_flag_ == true){
		  break;
		}

		if(needrecvsize > 0){
			if(needrecvsize > 4096){
			 	bytes_data = 4096;
			}
			else{
				bytes_data = needrecvsize;
			}
		 
			bytes_read = usbmuxd_recv_data(socket_,buffer,bytes_data); 
			needrecvsize -= bytes_read;
			memcpy(recv_buffer+recvbufferlen,buffer,bytes_read);
			recvbufferlen += bytes_read;
		}
		
		if (recvbufferlen > 0 && needrecvsize <= 0 && bytes_read > 0) {			
			//check data
			explainflag = explainrecvdata();			   
			if(!explainflag || recvbufferlen <= 8){
				LOG4CXX_ERROR(logger_,"usbmuxd data erro,Discard data");
			}
			else{
				LOG4CXX_DEBUG(logger_,
					"Received " << recvbufferlen << " bytes for connection "
					<< this);
			  	::protocol_handler::RawMessagePtr frame(
			 	new protocol_handler::RawMessage(0, 0,(uint8_t*)recv_buffer, recvbufferlen-8));
			  	controller_->DataReceiveDone(
			    	device_handle(), application_handle(), frame);
			}		

		  	memset(recv_buffer,0,RECVBUFFERLEN);
		 	recvbufferlen = 0;
		} 
		else if (bytes_read < 0) {
			if (EAGAIN != errno && EWOULDBLOCK != errno) {
		    	LOG4CXX_ERROR_WITH_ERRNO(logger_,
		       		"recv() failed for connection " << this);
		    	return false;
		  	}
		} 
		else if(bytes_read == 0){
			LOG4CXX_WARN(logger_, "Connection " << this << " closed by remote peer");
			return false;
		}
	} while (bytes_read > 0 && (terminate_flag_ == false));

	return true;
}

bool UsbmuxdSocketConnection::Send() {
	LOG4CXX_AUTO_TRACE(logger_);
	FrameQueue frames_to_send_local;
	{
		sync_primitives::AutoLock auto_lock(frames_to_send_mutex_);
		std::swap(frames_to_send_local, frames_to_send_);
	}

	size_t offset = 0;
	while (!frames_to_send_local.empty() && (terminate_flag_ == false)) {
		int bytes_sent = 0;
		LOG4CXX_INFO(logger_, "frames_to_send is not empty");
		::protocol_handler::RawMessagePtr frame = frames_to_send_local.front();

		int nneedsenddatalen = frame->data_size() - offset;
		int ncurrentsendlen = 0,nreadsendlen = 0,nrealoffset = 0;
		while(nneedsenddatalen > 0){
			if(nneedsenddatalen > RECVBUFFERLEN){
				ncurrentsendlen = RECVBUFFERLEN;
			}
			else{
				ncurrentsendlen = nneedsenddatalen;
			}

			nrealoffset = offset + nreadsendlen;
			bytes_sent = usbmuxd_send_data(socket_,frame->data() + nrealoffset, ncurrentsendlen);

			if (bytes_sent >= 0) {
				LOG4CXX_DEBUG(logger_, "bytes_sent >= 0");
				offset += bytes_sent;	  
				if (offset == frame->data_size()) {
					frames_to_send_local.pop();
					offset = 0;
					controller_->DataSendDone(device_handle(), application_handle(), frame);
				}
			} 
			else {
				LOG4CXX_DEBUG(logger_, "bytes_sent < 0");
				LOG4CXX_ERROR_WITH_ERRNO(logger_, "Send failed for connection " << this);
				frames_to_send_local.pop();
				offset = 0;
				controller_->DataSendFailed(
				  device_handle(), application_handle(), frame, DataSendError());
			}

			nneedsenddatalen -= ncurrentsendlen;
			nreadsendlen += ncurrentsendlen;
		}

	}   

	return true;
}

UsbmuxdSocketConnection::UsbmuxdSocketConnectionDelegate::UsbmuxdSocketConnectionDelegate(
    UsbmuxdSocketConnection* connection)
    : connection_(connection) {}

void UsbmuxdSocketConnection::UsbmuxdSocketConnectionDelegate::threadMain() {
	LOG4CXX_AUTO_TRACE(logger_);
	DCHECK(connection_);
	connection_->threadMain();
}

void UsbmuxdSocketConnection::UsbmuxdSocketConnectionDelegate::exitThreadMain() {
	LOG4CXX_AUTO_TRACE(logger_);
}

UsbmuxdServerOiginatedSocketConnection::UsbmuxdServerOiginatedSocketConnection(
    const DeviceUID& device_uid,
    const ApplicationHandle& app_handle,
    TransportAdapterController* controller)
    : UsbmuxdSocketConnection(device_uid, app_handle, controller,(unsigned int)20001) {}

UsbmuxdServerOiginatedSocketConnection::~UsbmuxdServerOiginatedSocketConnection() {}

bool UsbmuxdServerOiginatedSocketConnection::Establish(ConnectError** error) {
	LOG4CXX_AUTO_TRACE(logger_);
	DCHECK(error);
	LOG4CXX_DEBUG(logger_, "error " << error);
	DeviceSptr device = controller()->FindDevice(device_handle());
	if (!device.valid()) {
		LOG4CXX_ERROR(logger_, "Device " << device_handle() << " not found");
		*error = new ConnectError();
		return false;
	}

	UsbmuxdDevice* Usbmuxd_device = static_cast<UsbmuxdDevice*>(device.get());
	ApplicationHandle tmp = application_handle();
	uint32_t handle = Usbmuxd_device->applications_[tmp].apphandle;
	int mobileappport = 20001;

	const int socket = usbmuxd_connect(handle,mobileappport);
	if (socket < 0) {
		LOG4CXX_ERROR(logger_, "Failed to connect");
		*error = new ConnectError();
		return false;
	}

	set_socket(socket);
	return true;
}

}  // namespace transport_adapter
}  // namespace transport_manager
