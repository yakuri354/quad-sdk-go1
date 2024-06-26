/*
MIT License (modified)

Copyright (c) 2022 Ghost Robotics

Permission is hereby granted, free of charge, to any person obtaining a copy
of this **file** (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

This sends messages to the Ghost Robotics robots via ethernet UDP MAVLink packets.

Updated: April 2020

*/
#pragma once

// Network
#ifdef _MSC_VER
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <winsock2.h>
#define socklen_t int
#else
#include <arpa/inet.h>
#include <net/if.h>
#include <unistd.h>
#define SOCKET_ERROR -1
#define SOCKET int
#define closesocket close
#endif

// Protocol
#include "mblink_protocol.hpp"
#include <thread>

namespace gr
{

class MBLink : public MBLinkProtocol
{
public:
	MBLink() : rxKeepRunning(true) {}

	~MBLink()
	{
		if (rxThread.joinable())
		{
			rxThread.join();
		}
	}

	/**
	 * @brief Initialize from command line arguments
	 * 
	 * @param argc 
	 * @param argv 
	 * @return int 
	 */
	int start(int argc, char **argv);
	/**
	 * @brief Start MAVLink connection socket and check IP address
	 * 
	 * @param address defaults to broadcast to robot subnet
	 * @param print Print messages
	 */
	int start(bool sim, bool verbose);

	void rxstart();

	void rxstop(bool waitForExit = true);

	/**
	 * @brief Read a parameter. It will block till the param value comes back.
	 * 
	 * @param name param name
	 * @return float param value
	 */
	float readParam(const std::string &name);

	/**
	 * @brief Set a named parameter on the mainboard with retries for robustness
	 * 
	 * @param name param name
	 * @param val new value
	 */
	void setRetry(const std::string &name, float val);

	/**
	 * @brief Get all the rxdata. It will make a copy if there is new data.
	 * 
	 * @return RxData_t 
	 */
	RxData_t get();

	// avg rate at which new data is being updated
	float avgRate = 0;

protected:
	// Addresses
	std::string LOCAL_ADDR = "127.0.0.1";
	std::string MB_ADDR = "192.168.168.5";
	std::string BCAST_ADDR = "192.168.168.255";
	uint16_t TX_PORT = 14999;
	uint16_t RX_PORT = 15000;

	bool verbose = true;

	// Message buffer
	constexpr static int BUFFER_LENGTH = 1024;
	unsigned char buffer[BUFFER_LENGTH];
	struct sockaddr_in mbAddr;
	SOCKET txSocket, rxSocket;
	struct timeval tv;

	bool ipCheck();

	SOCKET createSocket(std::string address, int port, bool bcast, bool input);

	/**
	 * @brief Basic TX implementation which creates a UDP packet from each message
	 */
	virtual int queueMessage(uint16_t messageLength);

	void rxThreadTask();

	std::atomic_bool rxKeepRunning;
	std::thread rxThread;
};

} // namespace gr
