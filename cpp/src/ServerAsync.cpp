#include <boost/asio.hpp>
#include <boost/cobalt.hpp>
#include <iostream>
#include <array>
#include <iterator>
#include <opencv4/opencv2/opencv.hpp>

constexpr size_t kBUF_DEFAULT_SIZE = 640*480*3;



namespace cobalt = boost::cobalt;
namespace asio = boost::asio; 
using boost::asio::ip::tcp;

using tcp_acceptor = cobalt::use_op_t::as_default_on_t<tcp::acceptor>;
// using async_read = cobalt::use_op_t::as_default_on_t<boost::asio::async_read>;

typedef std::vector<uint8_t> CameraFrame_t; 
typedef std::shared_ptr<CameraFrame_t> CameraFrame_pt; 

class CameraDataFrames : private std::deque<CameraFrame_pt>
	{
	private:
		int id;
		int maxBufCount; 
	public: 
		CameraDataFrames(int maxBufCount = 5): 
			maxBufCount(maxBufCount)
		{}
		// CameraFrame_pt getPtr(int frameNum=0) {
		// 	return std::make_shared<CameraFrame_t>(this->at(frameNum));
		// }
		CameraFrame_pt get(int frameNum=0) {
			return this->at(frameNum);
		}
		void add(CameraFrame_pt data) {
			if (this->size() >= 5) {
				this->pop_back(); 
			}
			this->push_front(data); 
		}
	};

class CameraTransportProtocol {
	private: 
		CameraDataFrames m_dataFrames; 
		double m_lastMessageBytePosition = 0; 
		bool m_inFrameTransport = false; 
		// do not initizlize buffer before hand, messes up .size(), always use .reserve()
		CameraFrame_pt m_frameBuf; 

		// boost::asio::mutable_buffer m_chunkBufferMut {&m_chunkBuffer, 5000}; 

		uint8_t m_camID = 0; 
		uint16_t m_frameSize[2] = {0,0};
		double m_expectedNumOfDataBytes = 0; 
		bool m_initizlized = false;  

	public: 
	CameraTransportProtocol(){

	};

	// initial connection socket handler
	cobalt::promise<void> HandleConnection(tcp::socket incoming) {
	try 
	{
		std::byte chunkBuffer[kBUF_DEFAULT_SIZE]; 
		while (incoming.is_open()) 
		{ // continues read data in chunks, change chunksize to be higher to reduce the number of calls to DataReceived
			// size_t n = co_await incoming.async_receive(boost::asio::buffer(m_chunkBuffer, kBUF_DEFAULT_SIZE)); 
			// double n = co_await boost::asio::async_read(incoming, boost::asio::buffer(chunkBuffer), cobalt::use_op);
			size_t n = co_await incoming.async_read_some(boost::asio::buffer(chunkBuffer), cobalt::use_op); 
			// printf("%f bytes read\n", (double)n);
			co_await DataReceived(n, chunkBuffer); 
		}; 
		printf("client disconnected"); 
	} catch (std::exception& e) {
		printf("Exception: %s\n", e.what());
	}
	}

	// data chunk handler
	cobalt::promise<void> DataReceived(size_t numBytes, std::byte chunkBuffer[]) {
		if (numBytes <= 0) co_return; 

		// |-HEADER-									  -|-BODY-									-|    
		// 1byte			2bytes			2byte		   	xbit
		// camId		 	frame width	 	frame height	frame data(width*height*3 amount of bits)
		// uint8			uint16			uint16		  	x of uint8
		if(!m_initizlized) { // initiliazation
			// deserialization of camera parameters 
			m_camID = std::bit_cast<uint8_t>(chunkBuffer[0]);
			memcpy(this->m_frameSize, chunkBuffer+1, sizeof m_frameSize);

			m_expectedNumOfDataBytes = (double)m_frameSize[0] * (double)m_frameSize[1] * 3; 
			this->m_frameBuf = std::make_shared<CameraFrame_t>(CameraFrame_t());

			this->m_frameBuf->reserve(m_expectedNumOfDataBytes); // pre allocate
			
			// re-call with only data and in data mode 
			this->m_inFrameTransport = true;
			this->m_initizlized = true; 
			printf("Initialized for camera %d, %.0fx%.0f\n", m_camID, (double)m_frameSize[0], (double)m_frameSize[1]);

			size_t numOverflowBytes = numBytes - 5; // header consists of 5 bytes total
			if (numOverflowBytes > 0) { // numOverflowBytes can be negative, so this if is needed
				std::byte overflow[kBUF_DEFAULT_SIZE]; 
				memcpy(overflow, chunkBuffer+5, numOverflowBytes); 
				co_await DataReceived(numOverflowBytes, overflow); 
			}
			co_return; // exit 
		}
		
		// handle after initialization data chunk inflow 

		// type casting to uint8 storage types ~~~~ scarrrrrrry ~~ should be all 1 byte data
		uint8_t* uint8ChunkBuf = reinterpret_cast<uint8_t*>(chunkBuffer); // compiler be like: no wrong type, me: no it's not, compiler: ok, you good 

		// if numBytesWanted is ever somehow zero, a seg fault due to recursion will happen
		size_t numBytesWanted = m_expectedNumOfDataBytes - m_frameBuf->size();  
		// if (numBytesWanted <= 0) { 
		// 	printf("zero bytes wanted \n"); 
		// 	co_return; 
		// } 
		if (numBytes <= numBytesWanted) {
			m_frameBuf->insert(std::end(*m_frameBuf), uint8ChunkBuf, uint8ChunkBuf + numBytes); 

			// printf("processed %.0f bytes\n", (double)numBytes);
		} else {
			m_frameBuf->insert(std::end(*m_frameBuf), uint8ChunkBuf, uint8ChunkBuf + numBytesWanted); 
			
			// store the current acquired data and allocate a new frame
			m_dataFrames.add(m_frameBuf); 
			this->m_frameBuf = std::make_shared<CameraFrame_t>(CameraFrame_t()); 
			this->m_frameBuf->reserve(m_expectedNumOfDataBytes); 

			try
			{
			cv::Mat image{(int)m_frameSize[1], (int)m_frameSize[0], CV_8UC3, m_dataFrames.get()->data()};
			cv::imshow("got", image);
			cv::pollKey();
			}
			catch( cv::Exception& e )
			{
				std::cerr << e.what() << '\n';
			}
			
			 

			// recursion for overflow handling 
			size_t numOverflowBytes = numBytes - numBytesWanted; 
			if (numOverflowBytes <= 0) co_return;
			std::byte overflow[kBUF_DEFAULT_SIZE]; 
			memcpy(overflow, chunkBuffer + numBytesWanted, numOverflowBytes); 
			// printf("overflew %.0f bytes\n", (double)numBytesWanted);
			co_await DataReceived(numOverflowBytes, overflow); // causing segfault
		}

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
  co_return tcp::socket{acceptor.get_executor()}; // to make c++ intellisense and compiler happy, absoulotely does nothing 
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
