#include "ElapsedTimer.hpp"

using namespace std::chrono;

ElapsedTimer::ElapsedTimer()
{
	start();
}

void ElapsedTimer::start(){
	m_start=high_resolution_clock::now();
}

void ElapsedTimer::restart(){
	start();
}

int ElapsedTimer::elapsed_ms() {
	return duration_cast<milliseconds>(high_resolution_clock::now() - m_start).count();
}

