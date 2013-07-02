#ifndef RELNAV_H
#define RELNAV_H

// This is a header-only class definition file to handle relative navigation

#include <AP_AHRS.h>
#include <FastSerial.h>
#include <math.h>
#include "vector3.h"
#include "matrix3.h"





class RelNAV {
protected:

	Vector3<float> dx_b;			// relative vector in follower's body frame (inches)
	Vector3<float> dx_ff;			// relative vector in formation frame (inches)
	float dphi, dtheta, dpsi;		// relative Euler angles (degrees)

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
		return (180/M_PI)*atan2(-dx_b.z,dx_b.x);
	}

	// get other pitch cmd
	double bz(){
		return -dx_b.z * (2.5400);
	}

	// get relative bank
	double get_relBank() {return dphi;};

	// get relative heading
	double get_relHdg() {return dpsi;};

	// update the DCM for FF frame
	void updateDCM(int32_t roll_centi, int32_t pitch_centi) {
		// update DCM for body to Formation Frame
		// (Do not rotate back through heading)
		DCM.from_euler((M_PI/180)*roll_centi/100.0, (M_PI/180)*pitch_centi/100.0, 0);

		//compute relative vector in 
		dx_ff = DCM * dx_b;

		bearing_err = 100 * atan2(dx_ff.y,dx_ff.x) * (180/M_PI);  // convert to centidegrees
		altitude_err = -dx_ff.z * (2.5400);  // convert inches to cm
		
		level_dist = sqrt( pow(dx_ff.x,2) + pow(dx_ff.y,2) ) * (2.5400);  // convert inches to cm
	}



	// listen over serial port for relative navigation update
	boolean update() {
		
			uint8_t incomingByte;
			boolean receivedData = false;
			float payload[6];

			if (rNAVSerial->available() >= 5) {

				// check for message header
				rNAVSerial->find("DATA");
				uint8_t chk = 'D' ^ 'A' ^ 'T' ^ 'A';

				// get the payload length and make sure it has a length of 6
				uint8_t payload_len = rNAVSerial->read();
				chk = chk ^ payload_len;

				if ((payload_len==6) && (rNAVSerial->available() >= (4*payload_len + 1)) ) {

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

					// compare checksums
					if ( (rNAVSerial->read()) == chk) {
						receivedData = true;
						dx_b.x		= payload[0];
						dx_b.y		= payload[1];
						dx_b.z		= payload[2];
						dphi		= payload[3];
						dtheta		= payload[4];
						dpsi		= payload[5];

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