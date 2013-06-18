#ifndef RELNAV_H
#define RELNAV_H

// This is a header-only class definition file

class RelNAV {

		FastSerial* rNAVSerial;
		float payload[6];
public:



	RelNAV(){
		rNAVSerial = NULL;
	};

	~RelNAV(){};

	void setSerial(FastSerial* serial_ptr){
		rNAVSerial = serial_ptr;
		rNAVSerial->println("H");	// put in a request for data // #MD
	}

	 

	boolean update(float relState[6]) {
		
			uint8_t incomingByte;
			boolean receivedData = false;
			float payload[6];

			Serial1.println(rNAVSerial->available());
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
						memcpy(relState,payload,sizeof(payload));
					} else {
						// checksum did not match read value
						//Serial1.println("BAD CHECKSUM");
					}

				} else {
					// there were not enough bytes in the buffer to read the paylaod
					//Serial1.println("NOT ENOUGH BYTES FOR PAYLOAD");
				}

			} else {
				// There wasn't enough data in the buffer to read a header message
				//Serial1.println("NOT ENOUGH DATA IN BUFFER");
			}

			// request data for next time
			rNAVSerial->println("H");


			return receivedData;

		} // end #MD




};



#endif /*RELNAV_H*/