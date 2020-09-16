#pragma once
#include <chrono>

class ElapsedTimer {
	std::chrono::high_resolution_clock::time_point m_start;
public:
	ElapsedTimer();
	
	void start();
	void restart();
	
	int elapsed_ms();
	
};
