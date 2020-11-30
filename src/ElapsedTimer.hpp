#pragma once
#include <chrono>

class ElapsedTimer {
	std::chrono::high_resolution_clock::time_point m_start;
	std::chrono::high_resolution_clock::time_point m_lap;
public:
	ElapsedTimer();
	
	void start();
	void restart();
	void save_lap();
	int elapsed_ms();
	int lap_ms()const;
	
	static int duration_ms(const std::chrono::system_clock::duration &duration);
};
