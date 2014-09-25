#pragma once

class Particle{
private:
	float x, y, theta;

public:
	Particle():x(0.0), y(0.0), theta(0.0){}

	void ApplyMotionModel();

};
