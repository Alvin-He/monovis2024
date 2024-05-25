#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core/cuda.hpp>
#include <opencv4/opencv2/cudawarping.hpp>

#include <boost/asio.hpp>
#include <boost/cobalt.hpp>

constexpr uint8_t kCAM_ID = 0; 
constexpr uint16_t kTRANSPORT_WIDTH = 640; 
constexpr uint16_t kTRANSPORT_HEIGHT = 480; 

namespace cobalt = boost::cobalt;
namespace asio = boost::asio;  
using tcp = asio::ip::tcp; 



cobalt::generator<cv::Mat> getFrameGen(cv::VideoCapture cap,  cv::Mat buffer = cv::Mat()){
	while (true)
	{
		bool ret = cap.read(buffer); 
		if (!ret) continue;
		//resize
		cv::cuda::GpuMat cudaBuf;
		cudaBuf.upload(buffer); 
		cv::cuda::resize(cudaBuf, cudaBuf, cv::Size2d(kTRANSPORT_WIDTH, kTRANSPORT_HEIGHT)); 
		cudaBuf.download(buffer); 
		cv::imshow("test", buffer);
		cv::waitKey(30);  
		co_yield buffer; 
	}	
	co_return cv::Mat(); // cpp syntax fix
}


// int main(int argc, char const *argv[])
// {
// 	asio::io_context asioContext; 

	

// 	cv::VideoCapture camera(0); 

// 	while (camera.isOpened())
// 	{
// 		cv::Mat frameBuf; 
// 		// bool ret = camera.read(frameBuf);

// 		getImage(camera, frameBuf);
// 		cv::imshow("reader", frameBuf); 
// 		cv::waitKey(30); 
// 	}
	


// 	return 0;
// }

// |-HEADER-									  -|-BODY-									-|    
// 1byte			2bytes			2byte		   	xbit
// camId		 	frame width	 	frame height	frame data(width*height*3 amount of bits)
// uint8			uint16			uint16		  	x of uint8
std::array<std::byte, 5> getHeader() { // pointer to a 5 byte array
	std::array<std::byte, 5> internal; 
	std::memcpy(std::begin(internal), &kCAM_ID, 1); 
	std::memcpy(std::begin(internal) + 1, &kTRANSPORT_WIDTH, 2); 
	std::memcpy(std::begin(internal) + 3, &kTRANSPORT_HEIGHT, 2); 
	return internal;
}

cobalt::main co_main(int argc, char * argv[]) {
	tcp::endpoint endpoint {asio::ip::address::from_string("127.0.0.1"), 8090}; 
	printf("Establishing connection\n"); 
	tcp::socket connection {co_await cobalt::this_coro::executor};
	connection.connect(endpoint);  

	cv::VideoCapture cap(0); 
	cobalt::generator<cv::Mat> frameGen = getFrameGen(cap); 

	connection.send(asio::buffer(getHeader()));
	
	printf("Connection established, send started. \n");
	while (true)
	{
		// frame should now be continuous
		cv::Mat frame = (co_await frameGen).clone(); 
		std::byte writeBuf[(int)kTRANSPORT_HEIGHT * (int)kTRANSPORT_WIDTH * 3]; 
		memcpy(writeBuf, frame.data, std::size(writeBuf));
		co_await connection.async_send(asio::buffer(std::move(writeBuf)), cobalt::use_op);

	}
	co_return 0; 
} 