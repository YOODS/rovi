#pragma once
#include <chrono>

class ElapsedTimer {
	std::chrono::high_resolution_clock::time_point m_start;
	std::chrono::high_resolution_clock::time_point m_lap;
public:
	ElapsedTimer();
	
	void start();
	void restart();
	void start_lap();
	int elapsed_ms()const;
	int elapsed_lap_ms()const;
	
	static int duration_ms(const std::chrono::system_clock::duration &duration);
};
