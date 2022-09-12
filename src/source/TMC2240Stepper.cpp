#include "TMCStepper.h"
#include "TMC_MACROS.h"
#include "SERIAL_SWITCH.h"



TMC2240Stepper::TMC2240Stepper(Stream * SerialPort, float RS, uint8_t addr) :
	TMCStepper(RS),
	slave_address(addr)
	{
		HWSerial = SerialPort;
		defaults();
	}


TMC2240Stepper::TMC2240Stepper(Stream * SerialPort, float RS, uint8_t addr, uint16_t mul_pin1, uint16_t mul_pin2) :
	TMC2240Stepper(SerialPort, RS)
	{
		SSwitch *SMulObj = new SSwitch(mul_pin1, mul_pin2, addr);
		sswitch = SMulObj;
	}



#if SW_CAPABLE_PLATFORM
	TMC2240Stepper::TMC2240Stepper(uint16_t SW_RX_pin, uint16_t SW_TX_pin, float RS, uint8_t addr) :
		TMCStepper(RS),
		RXTX_pin(SW_RX_pin == SW_TX_pin ? SW_RX_pin : 0),
		slave_address(addr)
		{
			SoftwareSerial *SWSerialObj = new SoftwareSerial(SW_RX_pin, SW_TX_pin);
			SWSerial = SWSerialObj;
			defaults();
		}

	void TMC2240Stepper::beginSerial(uint32_t baudrate) {
		if (SWSerial != nullptr)
		{
			SWSerial->begin(baudrate);
			SWSerial->end();
		}
		#if defined(ARDUINO_ARCH_AVR)
			if (RXTX_pin > 0) {
				digitalWrite(RXTX_pin, HIGH);
				pinMode(RXTX_pin, OUTPUT);
			}
		#endif
	}
#endif



void TMC2240Stepper::begin() {
	#if SW_CAPABLE_PLATFORM
		beginSerial(115200);
	#endif
}


void TMC2240Stepper::defaults() {
	// GCONF_register..sr = 0x0004;
	// CHOPCONF_register.sr = 0x10000053;
	// PWMCONF_register.sr = 0xC10D0024;
}


void TMC2240Stepper::push() {
	GCONF(GCONF_register.sr);
    DRV_CONF(DRV_CONF_register.sr);
	IHOLD_IRUN(IHOLD_IRUN_register.sr);
	CHOPCONF(CHOPCONF_register.sr);
	PWMCONF(PWMCONF_register.sr);
}


bool TMC2240Stepper::isEnabled() { return !drv_enn() && toff(); }

uint8_t TMC2240Stepper::calcCRC(uint8_t datagram[], uint8_t len) {
	uint8_t crc = 0;
	for (uint8_t i = 0; i < len; i++) {
		uint8_t currentByte = datagram[i];
		for (uint8_t j = 0; j < 8; j++) {
			if ((crc >> 7) ^ (currentByte & 0x01)) {
				crc = (crc << 1) ^ 0x07;
			} else {
				crc = (crc << 1);
			}
			crc &= 0xff;
			currentByte = currentByte >> 1;
		}
	}
	return crc;
}


__attribute__((weak))
int TMC2240Stepper::available() {
	int out = 0;
	#if SW_CAPABLE_PLATFORM
		if (SWSerial != nullptr) {
			out = SWSerial->available();
		} else
	#endif
		if (HWSerial != nullptr) {
			out = HWSerial->available();
		}

	return out;
}



__attribute__((weak))
void TMC2240Stepper::preWriteCommunication() {
	if (HWSerial != nullptr) {
		if (sswitch != nullptr)
			sswitch->active();
	}
}



__attribute__((weak))
void TMC2240Stepper::preReadCommunication() {
	#if SW_CAPABLE_PLATFORM
		if (SWSerial != nullptr) {
			SWSerial->listen();
		} else
	#endif
		if (HWSerial != nullptr) {
			if (sswitch != nullptr)
				sswitch->active();
		}
}



__attribute__((weak))
int16_t TMC2240Stepper::serial_read() {
	int16_t out = 0;
	#if SW_CAPABLE_PLATFORM
		if (SWSerial != nullptr) {
			out = SWSerial->read();
		} else
	#endif
		if (HWSerial != nullptr) {
			out = HWSerial->read();
		}

	return out;
}



__attribute__((weak))
uint8_t TMC2240Stepper::serial_write(const uint8_t data) {
	int out = 0;;
	#if SW_CAPABLE_PLATFORM
		if (SWSerial != nullptr) {
			return SWSerial->write(data);
		} else
	#endif
		if (HWSerial != nullptr) {
			return HWSerial->write(data);
		}

	return out;
}



__attribute__((weak))
void TMC2240Stepper::postWriteCommunication() {}

__attribute__((weak))
void TMC2240Stepper::postReadCommunication() {
	#if SW_CAPABLE_PLATFORM
		if (SWSerial != nullptr) {
			SWSerial->end();
		}
	#endif
}



void TMC2240Stepper::write(uint8_t addr, uint32_t regVal) {
	uint8_t len = 7;
	addr |= TMC_WRITE;
	uint8_t datagram[] = {TMC2240_SYNC, slave_address, addr, (uint8_t)(regVal>>24), (uint8_t)(regVal>>16), (uint8_t)(regVal>>8), (uint8_t)(regVal>>0), 0x00};

	datagram[len] = calcCRC(datagram, len);

	preWriteCommunication();

	for(uint8_t i=0; i<=len; i++) {
		bytesWritten += serial_write(datagram[i]);
	}
	postWriteCommunication();

	delay(replyDelay);
}



uint64_t TMC2240Stepper::_sendDatagram(uint8_t datagram[], const uint8_t len, uint16_t timeout) {
	while (available() > 0) serial_read(); // Flush

	#if defined(ARDUINO_ARCH_AVR)
		if (RXTX_pin > 0) {
			digitalWrite(RXTX_pin, HIGH);
			pinMode(RXTX_pin, OUTPUT);
		}
	#endif

	for(int i=0; i<=len; i++) serial_write(datagram[i]);

	#if defined(ARDUINO_ARCH_AVR)
		if (RXTX_pin > 0) {
			pinMode(RXTX_pin, INPUT_PULLUP);
		}
	#endif

	delay(this->replyDelay);

	// scan for the rx frame and read it
	uint32_t ms = millis();
	uint32_t sync_target = (static_cast<uint32_t>(datagram[0])<<16) | 0xFF00 | datagram[2];
	uint32_t sync = 0;

	do {
		uint32_t ms2 = millis();
		if (ms2 != ms) {
			// 1ms tick
			ms = ms2;
			timeout--;
		}
		if (!timeout) return 0;

		int16_t res = serial_read();
		if (res < 0) continue;

		sync <<= 8;
		sync |= res & 0xFF;
		sync &= 0xFFFFFF;

	} while (sync != sync_target);

	uint64_t out = sync;
	ms = millis();
	timeout = this->abort_window;

	for(uint8_t i=0; i<5;) {
		uint32_t ms2 = millis();
		if (ms2 != ms) {
			// 1ms tick
			ms = ms2;
			timeout--;
		}
		if (!timeout) return 0;

		int16_t res = serial_read();
		if (res < 0) continue;

		out <<= 8;
		out |= res & 0xFF;

		i++;
	}

	#if defined(ARDUINO_ARCH_AVR)
		if (RXTX_pin > 0) {
			digitalWrite(RXTX_pin, HIGH);
			pinMode(RXTX_pin, OUTPUT);
		}
	#endif

	while (available() > 0) serial_read(); // Flush

	return out;
}




uint32_t TMC2240Stepper::read(uint8_t addr) {
	constexpr uint8_t len = 3;
	addr |= TMC_READ;
	uint8_t datagram[] = {TMC2240_SYNC, slave_address, addr, 0x00};
	datagram[len] = calcCRC(datagram, len);
	uint64_t out = 0x00000000UL;

	for (uint8_t i = 0; i < max_retries; i++) {
		preReadCommunication();
		out = _sendDatagram(datagram, len, abort_window);
		postReadCommunication();

		delay(replyDelay);

		CRCerror = false;
		uint8_t out_datagram[] = {
			static_cast<uint8_t>(out>>56),
			static_cast<uint8_t>(out>>48),
			static_cast<uint8_t>(out>>40),
			static_cast<uint8_t>(out>>32),
			static_cast<uint8_t>(out>>24),
			static_cast<uint8_t>(out>>16),
			static_cast<uint8_t>(out>> 8),
			static_cast<uint8_t>(out>> 0)
		};
		uint8_t crc = calcCRC(out_datagram, 7);
		if ((crc != static_cast<uint8_t>(out)) || crc == 0 ) {
			CRCerror = true;
			out = 0;
		} else {
			break;
		}
	}

	return out>>8;
}


uint8_t TMC2240Stepper::IFCNT() {
	return read(IFCNT_t::address);
}


void TMC2240Stepper::SLAVECONF(uint16_t input) {
	SLAVECONF_register.sr = input&0xF00;
	write(SLAVECONF_register.address, SLAVECONF_register.sr);
}

uint16_t TMC2240Stepper::SLAVECONF() {
	return SLAVECONF_register.sr;
}

void TMC2240Stepper::senddelay(uint8_t B) 	{ SLAVECONF_register.senddelay = B; write(SLAVECONF_register.address, SLAVECONF_register.sr); }
uint8_t TMC2240Stepper::senddelay() 		{ return SLAVECONF_register.senddelay; }



uint32_t TMC2240Stepper::IOIN() {
	return read(TMC2240_n::IOIN_t::address);
}


bool TMC2240Stepper::step()			    { TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.step;		}
bool TMC2240Stepper::dir()			    { TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.dir;		}
bool TMC2240Stepper::encb()			    { TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.encb;		}
bool TMC2240Stepper::enca()			    { TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.enca;		}
bool TMC2240Stepper::drv_enn()		    { TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.drv_enn;	}
bool TMC2240Stepper::encn()			    { TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.encn;		}
bool TMC2240Stepper::uart_en()		    { TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.uart_en;	}
bool TMC2240Stepper::comp_a()		    { TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.comp_a;	}
bool TMC2240Stepper::comp_b()		    { TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.comp_b;	}
bool TMC2240Stepper::comp_a1_a2()	    { TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.comp_a1_a2;	}
bool TMC2240Stepper::comp_b1_b2()	    { TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.comp_b1_b2;	}
bool TMC2240Stepper::output()		    { TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.output;	}
bool TMC2240Stepper::ext_res_det()	    { TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.ext_res_det;	}
bool TMC2240Stepper::ext_clk()	        { TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.ext_clk;	}
bool TMC2240Stepper::adc_err()		    { TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.adc_err;	}
uint8_t TMC2240Stepper::silicon_rv() 	{ TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.silicon_rv;	}
uint8_t TMC2240Stepper::version() 	    { TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.version;	}


uint32_t TMC2240Stepper::PWM_SCALE() {
	return read(TMC2240_n::PWM_SCALE_t::address);
}
uint8_t TMC2240Stepper::pwm_scale_sum() {
	TMC2240_n::PWM_SCALE_t r{0};
	r.sr = PWM_SCALE();
	return r.pwm_scale_sum;
}



int16_t TMC2240Stepper::pwm_scale_auto() {
	TMC2240_n::PWM_SCALE_t r{0};
	r.sr = PWM_SCALE();
	return r.pwm_scale_auto;
	// Not two's complement? 9nth bit determines sign
	/*
	uint32_t d = PWM_SCALE();
	int16_t response = (d>>PWM_SCALE_AUTO_bp)&0xFF;
	if (((d&PWM_SCALE_AUTO_bm) >> 24) & 0x1) return -response;
	else return response;
	*/
}



// R: PWM_AUTO
uint32_t TMC2240Stepper::PWM_AUTO() {
	return read(PWM_AUTO_t::address);
}
uint8_t TMC2240Stepper::pwm_ofs_auto()  { PWM_AUTO_t r{0}; r.sr = PWM_AUTO(); return r.pwm_ofs_auto; }
uint8_t TMC2240Stepper::pwm_grad_auto() { PWM_AUTO_t r{0}; r.sr = PWM_AUTO(); return r.pwm_grad_auto; }


