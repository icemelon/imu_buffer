#include "GPSWrapper.h"
#include "GPS.h"

extern "C" {

	CGPS * GPS_new(PinName tx, PinName rx) {
		GPS *gps = new GPS(tx, rx);

		return (CGPS *)gps;
	}

	void test_delete(CGPS *gps) {
		Test *t = (Test *)gps;

		delete t;
	}
}