#include "Transport.h"
#include <memory>

class Waves : public Noise
{
	
};

class Ship : public Transport
{
public:
	Ship();

protected:
	void move() override;

private:
	Waves wave;

};