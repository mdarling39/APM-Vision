#ifndef RELNAV_H
#define RELNAV_H

// This is a header-only class definition file to handle relative navigation

#include <AP_AHRS.h>
#include <FastSerial.h>
#include <math.h>
#include "vector3.h"
#include "matrix3.h"

#define MASK_LED_1		(1<<0)
#define MASK_LED_2		(1<<1)
#define MASK_LED_3		(1<<2)
#define MASK_LED_4		(1<<3)
#define MASK_LED_5		(1<<4)
#define MASK_LED_ALL	(MASK_LED_1 | MASK_LED_2 | MASK_LED_3 | MASK_LED_4 | MASK_LED_5)




class RelNAV {
protected:

	Vector3<float> dx_b;			// relative vector in follower's body frame (inches)
	Vector3<float> dx_ff;			// relative vector in formation frame (inches)
	float dphi, dtheta, dpsi;		// relative Euler angles (degrees)
	byte LED_bitmask;				// gives the LEDs that are within the frame (when using HIL_MODE_ATTITUDE)

	Matrix3<float> DCM;
	FastSerial* rNAVSerial;

	int32_t bearing_err;	// 100*degrees
	int32_t altitude_err;   // cm
	int32_t level_dist;		// level distance between aircraft (cm)

public:


	// constructor
	RelNAV(){

		// dx_b
		// dx_ff
		dphi = dtheta = dpsi = 0;

		// DCM 
		rNAVSerial = NULL;

		bearing_err = 0;
		altitude_err = 0;

	};


	// destructor
	~RelNAV(){};


	// set serial port to accept relative navigation data over
	void setSerial(FastSerial* serial_ptr){
		rNAVSerial = serial_ptr;
		rNAVSerial->println("H");	// put in a request for data
	}



	// get relative bearing error
	int32_t relative_bearing_error(){
		return bearing_err;
	}

	// get relative altitude error
	int32_t relative_altitude_error(){
		return altitude_err;
	}

	// get level distance between aircraft
	int32_t get_level_dist(){
		return level_dist;
	}

	// get pitch_cmd
	double pitch_cmd(){
		return 100*(180/M_PI)*atan2(-dx_b.z,dx_b.x);
	}

	// get other pitch cmd
	double bz(){
		return -dx_b.z * (2.5400);
	}

	// get relative x  (inches)
	double get_relx() {return dx_b.x;};

	// get relative y  (inches)
	double get_rely() {return dx_b.y;};

	// get relative z  (inches)
	double get_relz() {return dx_b.z;};

	// get relative bank  (degrees)
	double get_relBank() {return dphi;};

	// get relative pitch  (degrees)
	double get_relPitch() {return dtheta;};

	// get relative heading  (degrees)
	double get_relHdg() {return dpsi;};

	// get the LED_bitmask
	byte get_LED_bitmask() {return LED_bitmask;};

	// update the DCM for FF frame
	void updateDCM(int32_t roll_centi, int32_t pitch_centi) {
		// update DCM for body to Formation Frame
		// (Do not rotate back through heading)
		DCM.from_euler((M_PI/180)*roll_centi/100.0, (M_PI/180)*pitch_centi/100.0, 0);

		//compute relative vector in 
		dx_ff = DCM * dx_b;

		bearing_err = 100 * atan2(dx_ff.y,dx_ff.x) * (180/M_PI);  // convert to centidegrees
		altitude_err = -(dx_ff.z) * (2.5400);  // convert inches to cm
		
		level_dist = sqrt( pow(dx_ff.x,2) + pow(dx_ff.y,2) ) * (2.5400);  // convert inches to cm
	}



	// listen over serial port for relative navigation update
	boolean update() {
		
			byte incomingByte, _LED_bitmask;
			boolean receivedData = false;
			float payload[6];

			if (rNAVSerial->available() >= 5) {

				// check for message header
				rNAVSerial->find("DATA");
				uint8_t chk = 'D' ^ 'A' ^ 'T' ^ 'A';

				// get the payload length and make sure it has a length of 6
				uint8_t payload_len = rNAVSerial->read();
				chk = chk ^ payload_len;


#if HIL_MODE==HIL_MODE_ATTITUDE
				if ((payload_len==6) && (rNAVSerial->available() >= (4*payload_len + 2)) ) {
#else
				if ((payload_len==6) && (rNAVSerial->available() >= (4*payload_len + 1)) ) {
#endif
					for (int i = 0; i<payload_len; i++) {

						union {
							uint8_t b[4];
							float f;
						} pld;

						pld.b[0] = rNAVSerial->read();
						pld.b[1] = rNAVSerial->read();
						pld.b[2] = rNAVSerial->read();
						pld.b[3] = rNAVSerial->read();

						chk = chk ^ pld.b[0] ^ pld.b[1] ^ pld.b[2] ^ pld.b[3];

						payload[i] = pld.f;
					}

#if HIL_MODE == HIL_MODE_ATTITUDE
					// read the bitmask that gives the LEDs in the field of view
						_LED_bitmask = rNAVSerial->read();
						chk = chk ^ _LED_bitmask;
#endif

					// compare checksums
					if ( (rNAVSerial->read()) == chk) {
						receivedData = true;  // we at least received data
#if HIL_MODE==HIL_MODE_ATTITUDE
						LED_bitmask = _LED_bitmask;
#else
						LED_bitmask = 0xFF;

#endif
						if ((LED_bitmask & 0x3F) == MASK_LED_ALL) {
							dx_b.x		= payload[0];
							dx_b.y		= payload[1];
							dx_b.z		= payload[2];
							dphi		= payload[3];
							dtheta		= payload[4];
							dpsi		= payload[5];
						} else {
							// not all LEDs in the frame
							Serial1.println("LEDS_MISSING");
						}

					} else {
						// checksum did not match read value
					}

				} else {
					// there were not enough bytes in the buffer to read the paylaod
				}

			} else {
				// There wasn't enough data in the buffer to read a header message
			}

			// request data for next time
			rNAVSerial->println("H");


			return receivedData;

		} // end #MD

};



#endif /*RELNAV_H*/