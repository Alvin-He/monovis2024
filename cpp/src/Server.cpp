

#include <iostream>
#include <time.h>
#include <boost/asio.hpp>
#include <deque> 

using boost::asio::ip::tcp; 

typedef std::shared_ptr<std::vector<uint8_t>> t_CameraFrame; 

class CameraDataFrames : private std::deque<t_CameraFrame>
	{
	private:
		int id;
		int maxBufCount; 
	public: 
		CameraDataFrames() = default;
		CameraDataFrames(int camID, int maxBufCount = 5): 
			id(camID), maxBufCount(maxBufCount)
		{}
		t_CameraFrame get(int frameNum=0) {
			return this->get(frameNum); 
		}
		void add(t_CameraFrame data) {
			if (this->size() >= 5) {
				this->pop_back(); 
			}
			this->push_front(data); 
		}
	};

class CameraTransportProtocol {
	private: 
		CameraDataFrames m_dataFrames {0}; 
		double m_lastMessageBytePosition = 0; 
		bool m_inFrameTransport = false; 
		boost::asio::mutable_buffer m_frameBuf; 
		boost::asio::mutable_buffer chunkBuffer = boost::asio::mutable_buffer(&std::vector<std::byte>(5000), 5000); 

		int m_camID = 0; 
		double m_frameSize[2] = {0,0};
		double m_expectedNumOfDataBytes = 0; 
		bool m_initizlized = false;  

	public: 
	CameraTransportProtocol(){

	};

	void HandleConnection(std::shared_ptr<tcp::socket> socket) {
		boost::asio::async_read(socket, chunkBuffer, [this, socket] (const boost::system::error_code& error, size_t bytesTransfered) {
			if (error.failed()) return; // stop reading if connection is closed or we have an error
			DataReceived();
			HandleConnection(socket);
		});
	}

	void DataReceived() {
		
		
	}; 

}; 


void CompletionHandler(const boost::system::error_code& error, size_t bytesTransfered) {
	printf("A write have been completed with error %s\r\n", error.message()); 
}

int main(int argc, char const *argv[])
{
	printf("Control Server Starting\n");
	
	std::shared_ptr<CameraTransportProtocol> cameraProtocol = std::make_shared<CameraTransportProtocol>(CameraTransportProtocol()); 

	boost::asio::io_context ioContext; 

	tcp::acceptor tcpConnectionAcceptor(ioContext, tcp::endpoint(boost::asio::ip::address::from_string("127.0.0.1"), 8090));

	//boost::asio::basic_stream_socket<boost::asio::ip::tcp, boost::asio::any_io_executor>
	tcpConnectionAcceptor.async_accept([cameraProtocol] (const boost::system::error_code& error, tcp::socket socket) {
		printf("%d\n", socket.is_open()); 
		printf("tttt");
		std::shared_ptr<tcp::socket> socketPtr = std::make_shared<tcp::socket>(socket); 
		cameraProtocol->HandleConnection(socketPtr);
	}); 

	ioContext.run();
	

	return 0;
}
