#include "ElapsedTimer.hpp"

using namespace std::chrono;

ElapsedTimer::ElapsedTimer()
{
	start();
}

void ElapsedTimer::start(){
	m_start=high_resolution_clock::now();
	m_lap = m_start;
}

void ElapsedTimer::restart(){
	start();
}

int ElapsedTimer::elapsed_ms() const{
	return duration_cast<milliseconds>(high_resolution_clock::now() - m_start).count();
}

void ElapsedTimer::start_lap(){
	m_lap = high_resolution_clock::now();
}

int ElapsedTimer::elapsed_lap_ms()const{
	return duration_cast<milliseconds>(high_resolution_clock::now() - m_lap).count();
}


int ElapsedTimer::duration_ms(const std::chrono::system_clock::duration &duration){
	return (int)std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}