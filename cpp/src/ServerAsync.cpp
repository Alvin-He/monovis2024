
#include <boost/asio.hpp>
#include <boost/cobalt.hpp>
#include <iostream>

namespace cobalt = boost::cobalt;
namespace asio = boost::asio; 
using boost::asio::ip::tcp;

using tcp_acceptor = cobalt::use_op_t::as_default_on_t<tcp::acceptor>;
// using async_read = cobalt::use_op_t::as_default_on_t<boost::asio::async_read>;

typedef std::shared_ptr<std::vector<uint8_t>> CameraFrame_t; 

class CameraDataFrames : private std::deque<CameraFrame_t>
	{
	private:
		int id;
		int maxBufCount; 
	public: 
		CameraDataFrames() = default;
		CameraDataFrames(int camID, int maxBufCount = 5): 
			id(camID), maxBufCount(maxBufCount)
		{}
		CameraFrame_t get(int frameNum=0) {
			return this->get(frameNum); 
		}
		void add(CameraFrame_t data) {
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
		std::shared_ptr<std::vector<uint8_t>> m_frameBuf; // these pointers to these 2 vectors should always be valid 

		// boost::asio::mutable_buffer m_chunkBufferMut {&m_chunkBuffer, 5000}; 

		uint8_t m_camID = 0; 
		uint16_t m_frameSize[2] = {0,0};
		double m_expectedNumOfDataBytes = 0; 
		bool m_initizlized = false;  

	public: 
	CameraTransportProtocol(){

	};

	cobalt::promise<void> HandleConnection(tcp::socket incoming) {
	try 
	{
		std::byte chunkBuffer[5000]; 
		while (incoming.is_open()) 
		{
			// size_t n = co_await incoming.async_receive(boost::asio::buffer(m_chunkBuffer, 5000)); 
			// double n = co_await boost::asio::async_read(incoming, boost::asio::buffer(chunkBuffer), cobalt::use_op);
			double n = co_await incoming.async_read_some(boost::asio::buffer(chunkBuffer), cobalt::use_op); 
			printf("%f bytes read\n", n);
			co_await DataReceived(n, chunkBuffer); 
		}; 
		printf("client disconnected"); 
	} catch (std::exception& e) {
		printf("Exception: %s\n", e.what());
	}
	}

	cobalt::promise<void> DataReceived(double numBytes, std::byte chunkBuffer[]) {
		if (m_inFrameTransport) {
			co_return; 
		}

		// |-HEADER-									  -|-BODY-									-|    
		// 1byte			2bytes			2byte		   	xbit
		// camId		 	frame width	 	frame height	frame data(width*height amount of bits)
		// uint8			uint16			uint16		  	x of uint8

		if(!m_initizlized) {
			// deserialization of camera parameters 
			m_camID = std::bit_cast<uint8_t>(chunkBuffer[0]);
			memcpy(this->m_frameSize, chunkBuffer+1, sizeof m_frameSize);

			m_expectedNumOfDataBytes = (double)m_frameSize[0] * (double)m_frameSize[1]; 

			// re-call with only data and in data mode 
			this->m_inFrameTransport = true;
			this->m_initizlized = true; 

			std::byte newBuffer[5000]; 
			double newNumBytes = numBytes - 5; // header consists of 5 bytes total
			if (newNumBytes > 0) {
				memcpy(newBuffer, chunkBuffer+5, newNumBytes); 
				co_await DataReceived(newNumBytes, newBuffer); 
			}
		}
		
		
		// std::cout << _frameSize[0] << std::endl << _frameSize[1] <<std::endl;
		// m_frameSize[0] = reinterpret_cast<uint16_t>(m_chunkBuffer.data()+1);
		// m_frameSize[1] = reinterpret_cast<uint16_t>(m_chunkBuffer.data()+3); 
		// printf("%d, %d, %d", m_camID, m_frameSize[0], m_frameSize[1]); 
	}; 

}; 



cobalt::generator<tcp::socket> listenForNewConnection() {
  tcp_acceptor acceptor(co_await cobalt::this_coro::executor, tcp::endpoint(boost::asio::ip::address::from_string("127.0.0.1"), 8090));
//   tcp::acceptor acceptor_ (co_await cobalt::this_coro::executor, tcp::endpoint(boost::asio::ip::address::from_string("127.0.0.1"), 8090))
  while (true)
  {
    tcp::socket socket = co_await acceptor.async_accept();
    co_yield std::move(socket); 
  }
  co_return tcp::socket{acceptor.get_executor()}; 
} 

cobalt::main co_main(int argc, char * argv[]) {

  CameraTransportProtocol protocol; 

  cobalt::generator<tcp::socket> listener = listenForNewConnection();
  while (true)
  {
    tcp::socket incomingSocket = co_await listener;
    co_await protocol.HandleConnection(std::move(incomingSocket)); 
  }
  


  co_return 0; 
}
