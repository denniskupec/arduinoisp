// Modified ArduinoISP 
//
// Pin 10 - RESET
// Pin 07 - (LED) Heartbeat, shows the programmer is running
// Pin 06 - (LED) Error, Lights up if something goes wrong
// Pin 05 - (LED) Programming, In communication with the slave
//

#include "Arduino.h"
#include "SPI.h"

#undef SERIAL
#define SERIAL Serial

#define PROG_FLICKER true

#define SPI_CLOCK 	(1000000/6)
#define BAUDRATE  	38400
#define HWVER 			2
#define SWMAJ 			1
#define SWMIN 			18

#define RESET     	10
#define LED_HB    	9
#define LED_ERR   	8
#define LED_PMODE 	7
#define PIN_MOSI  	MOSI
#define PIN_MISO  	MISO
#define PIN_SCK   	SCK

#define STK_OK      0x10
#define STK_FAILED  0x11
#define STK_UNKNOWN 0x12
#define STK_INSYNC  0x14
#define STK_NOSYNC  0x15
#define CRC_EOP     0x20
#define PTIME 		  30

void pulse(int pin, int times);

void setup()
{
	SERIAL.begin(BAUDRATE);

	pinMode(LED_PMODE, OUTPUT);
	pulse(LED_PMODE, 2);
	pinMode(LED_ERR, OUTPUT);
	pulse(LED_ERR, 2);
	pinMode(LED_HB, OUTPUT);
	pulse(LED_HB, 2);
}

int error = 0;
int pmode = 0;
unsigned int here;
uint8_t buff[256];

#define beget16(addr) (*addr * 256 + *(addr+1) )
typedef struct param
{
	uint8_t devicecode;
	uint8_t revision;
	uint8_t progtype;
	uint8_t parmode;
	uint8_t polling;
	uint8_t selftimed;
	uint8_t lockbytes;
	uint8_t fusebytes;
	uint8_t flashpoll;
	uint16_t eeprompoll;
	uint16_t pagesize;
	uint16_t eepromsize;
	uint32_t flashsize;
}
parameter;

parameter param;

uint8_t hbval = 128;
int8_t hbdelta = 8;
void heartbeat() {
	static unsigned long last_time = 0;
	unsigned long now = millis();
	if ((now - last_time) < 40)
		return;
	last_time = now;
	if (hbval > 192) hbdelta = -hbdelta;
	if (hbval < 32) hbdelta = -hbdelta;
	hbval += hbdelta;
	analogWrite(LED_HB, hbval);
}

static bool rst_active_high;

void reset_target(bool reset)
{
	digitalWrite(RESET, ((reset && rst_active_high) || (!reset && !rst_active_high)) ? HIGH : LOW);
}

void loop(void)
{
	if (pmode) {
		digitalWrite(LED_PMODE, HIGH);
	} else {
		digitalWrite(LED_PMODE, LOW);
	}

	if (error) {
		digitalWrite(LED_ERR, HIGH);
	} else {
		digitalWrite(LED_ERR, LOW);
	}

	heartbeat();
	if (SERIAL.available()) {
		avrisp();
	}
}

uint8_t getch() {
	while (!SERIAL.available());
	return SERIAL.read();
}

void fill(int n) {
	for (int x = 0; x < n; x++) {
		buff[x] = getch();
	}
}

void pulse(int pin, int times)
{
	do {
		digitalWrite(pin, HIGH);
		delay(PTIME);
		digitalWrite(pin, LOW);
		delay(PTIME);
	} while (times--);
}

void prog_lamp(int state)
{
	if (PROG_FLICKER) {
		digitalWrite(LED_PMODE, state);
	}
}

uint8_t spi_transaction(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
	SPI.transfer(a);
	SPI.transfer(b);
	SPI.transfer(c);
	return SPI.transfer(d);
}

void empty_reply()
{
	if (CRC_EOP == getch()) {
		SERIAL.print((char)STK_INSYNC);
		SERIAL.print((char)STK_OK);
	} else {
		error++;
		SERIAL.print((char)STK_NOSYNC);
	}
}

void breply(uint8_t b) {
	if (CRC_EOP == getch()) {
		SERIAL.print((char)STK_INSYNC);
		SERIAL.print((char)b);
		SERIAL.print((char)STK_OK);
	} else {
		error++;
		SERIAL.print((char)STK_NOSYNC);
	}
}

void get_version(uint8_t c)
{
	switch (c) {
		case 0x80:
			breply(HWVER);
			break;
		case 0x81:
			breply(SWMAJ);
			break;
		case 0x82:
			breply(SWMIN);
			break;
		case 0x93:
			breply('S');
			break;
		default:
			breply(0);
	}
}

void set_parameters()
{
	// call this after reading paramter packet into buff[]
	param.devicecode = buff[0];
	param.revision   = buff[1];
	param.progtype   = buff[2];
	param.parmode    = buff[3];
	param.polling    = buff[4];
	param.selftimed  = buff[5];
	param.lockbytes  = buff[6];
	param.fusebytes  = buff[7];
	param.flashpoll  = buff[8];
	// ignore buff[9] (= buff[8])
	// following are 16 bits (big endian)
	param.eeprompoll = beget16(&buff[10]);
	param.pagesize   = beget16(&buff[12]);
	param.eepromsize = beget16(&buff[14]);

	// 32 bits flashsize (big endian)
	param.flashsize = buff[16] * 0x01000000
										+ buff[17] * 0x00010000
										+ buff[18] * 0x00000100
										+ buff[19];

	// avr devices have active low reset, at89sx are active high
	rst_active_high = (param.devicecode >= 0xe0);
}

void start_pmode()
{

	// Reset target before driving PIN_SCK or PIN_MOSI

	// SPI.begin() will configure SS as output,
	// so SPI master mode is selected.
	// We have defined RESET as pin 10,
	// which for many arduino's is not the SS pin.
	// So we have to configure RESET as output here,
	// (reset_target() first sets the correct level)
	reset_target(true);
	pinMode(RESET, OUTPUT);
	SPI.begin();
	SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));

	// See avr datasheets, chapter "SERIAL_PRG Programming Algorithm":

	// Pulse RESET after PIN_SCK is low:
	digitalWrite(PIN_SCK, LOW);
	delay(20); // discharge PIN_SCK, value arbitrally chosen
	reset_target(false);
	// Pulse must be minimum 2 target CPU clock cycles
	// so 100 usec is ok for CPU speeds above 20KHz
	delayMicroseconds(100);
	reset_target(true);

	// Send the enable programming command:
	delay(50); // datasheet: must be > 20 msec
	spi_transaction(0xAC, 0x53, 0x00, 0x00);
	pmode = 1;
}

void end_pmode()
{
	SPI.end();
	pinMode(PIN_MOSI, INPUT);
	pinMode(PIN_SCK, INPUT);
	reset_target(false);
	pinMode(RESET, INPUT);
	pmode = 0;
}

void universal()
{
	uint8_t ch;

	fill(4);
	ch = spi_transaction(buff[0], buff[1], buff[2], buff[3]);
	breply(ch);
}

void flash(uint8_t hilo, unsigned int addr, uint8_t data)
{
	spi_transaction(0x40 + 8 * hilo,
									addr >> 8 & 0xFF,
									addr & 0xFF,
									data);
}

void commit(unsigned int addr)
{
	if (PROG_FLICKER) {
		prog_lamp(LOW);
	}
	spi_transaction(0x4C, (addr >> 8) & 0xFF, addr & 0xFF, 0);
	if (PROG_FLICKER) {
		delay(PTIME);
		prog_lamp(HIGH);
	}
}

unsigned int current_page()
{
	if (param.pagesize == 32) {
		return here & 0xFFFFFFF0;
	}
	if (param.pagesize == 64) {
		return here & 0xFFFFFFE0;
	}
	if (param.pagesize == 128) {
		return here & 0xFFFFFFC0;
	}
	if (param.pagesize == 256) {
		return here & 0xFFFFFF80;
	}
	return here;
}


void write_flash(int length)
{
	fill(length);
	if (CRC_EOP == getch()) {
		SERIAL.print((char) STK_INSYNC);
		SERIAL.print((char) write_flash_pages(length));
	} else {
		error++;
		SERIAL.print((char) STK_NOSYNC);
	}
}

uint8_t write_flash_pages(int length)
{
	int x = 0;
	unsigned int page = current_page();
	while (x < length) {
		if (page != current_page()) {
			commit(page);
			page = current_page();
		}
		flash(LOW, here, buff[x++]);
		flash(HIGH, here, buff[x++]);
		here++;
	}

	commit(page);

	return STK_OK;
}

#define EECHUNK (32)

uint8_t write_eeprom(unsigned int length)
{
	// here is a word address, get the byte address
	unsigned int start = here * 2;
	unsigned int remaining = length;
	if (length > param.eepromsize) {
		error++;
		return STK_FAILED;
	}
	while (remaining > EECHUNK) {
		write_eeprom_chunk(start, EECHUNK);
		start += EECHUNK;
		remaining -= EECHUNK;
	}
	write_eeprom_chunk(start, remaining);
	return STK_OK;
}

uint8_t write_eeprom_chunk(unsigned int start, unsigned int length)
{
	// this writes byte-by-byte,
	// page writing may be faster (4 bytes at a time)
	fill(length);
	prog_lamp(LOW);
	for (unsigned int x = 0; x < length; x++) {
		unsigned int addr = start + x;
		spi_transaction(0xC0, (addr >> 8) & 0xFF, addr & 0xFF, buff[x]);
		delay(45);
	}
	prog_lamp(HIGH);
	return STK_OK;
}

void program_page()
{
	char result = (char) STK_FAILED;
	unsigned int length = 256 * getch();
	length += getch();
	char memtype = getch();
	// flash memory @here, (length) bytes
	if (memtype == 'F') {
		write_flash(length);
		return;
	}
	if (memtype == 'E') {
		result = (char)write_eeprom(length);
		if (CRC_EOP == getch()) {
			SERIAL.print((char) STK_INSYNC);
			SERIAL.print(result);
		} else {
			error++;
			SERIAL.print((char) STK_NOSYNC);
		}
		return;
	}
	SERIAL.print((char)STK_FAILED);
	return;
}

uint8_t flash_read(uint8_t hilo, unsigned int addr)
{
	return spi_transaction(0x20 + hilo * 8,
												 (addr >> 8) & 0xFF,
												 addr & 0xFF,
												 0);
}

char flash_read_page(int length)
{
	for (int x = 0; x < length; x += 2) {
		uint8_t low = flash_read(LOW, here);
		SERIAL.print((char) low);
		uint8_t high = flash_read(HIGH, here);
		SERIAL.print((char) high);
		here++;
	}
	return STK_OK;
}

char eeprom_read_page(int length)
{
	// here again we have a word address
	int start = here * 2;
	for (int x = 0; x < length; x++) {
		int addr = start + x;
		uint8_t ee = spi_transaction(0xA0, (addr >> 8) & 0xFF, addr & 0xFF, 0xFF);
		SERIAL.print((char) ee);
	}
	return STK_OK;
}

void read_page()
{
	char result = (char)STK_FAILED;
	int length = 256 * getch();
	length += getch();
	char memtype = getch();
	if (CRC_EOP != getch()) {
		error++;
		SERIAL.print((char) STK_NOSYNC);
		return;
	}
	SERIAL.print((char) STK_INSYNC);
	if (memtype == 'F') result = flash_read_page(length);
	if (memtype == 'E') result = eeprom_read_page(length);
	SERIAL.print(result);
}

void read_signature()
{
	if (CRC_EOP != getch()) {
		error++;
		SERIAL.print((char) STK_NOSYNC);
		return;
	}
	SERIAL.print((char) STK_INSYNC);
	uint8_t high = spi_transaction(0x30, 0x00, 0x00, 0x00);
	SERIAL.print((char) high);
	uint8_t middle = spi_transaction(0x30, 0x00, 0x01, 0x00);
	SERIAL.print((char) middle);
	uint8_t low = spi_transaction(0x30, 0x00, 0x02, 0x00);
	SERIAL.print((char) low);
	SERIAL.print((char) STK_OK);
}

void avrisp() {
	uint8_t ch = getch();
	switch (ch) {
		case '0': // signon
			error = 0;
			empty_reply();
			break;
		case '1':
			if (getch() == CRC_EOP) {
				SERIAL.print((char) STK_INSYNC);
				SERIAL.print("AVR ISP");
				SERIAL.print((char) STK_OK);
			}
			else {
				error++;
				SERIAL.print((char) STK_NOSYNC);
			}
			break;
		case 'A':
			get_version(getch());
			break;
		case 'B':
			fill(20);
			set_parameters();
			empty_reply();
			break;
		case 'E': // extended parameters - ignore for now
			fill(5);
			empty_reply();
			break;
		case 'P':
			if (!pmode)
				start_pmode();
			empty_reply();
			break;
		case 'U': // set address (word)
			here = getch();
			here += 256 * getch();
			empty_reply();
			break;

		case 0x60: //STK_PROG_FLASH
			getch(); // low addr
			getch(); // high addr
			empty_reply();
			break;
		case 0x61: //STK_PROG_DATA
			getch(); // data
			empty_reply();
			break;

		case 0x64: //STK_PROG_PAGE
			program_page();
			break;

		case 0x74: //STK_READ_PAGE 't'
			read_page();
			break;

		case 'V': //0x56
			universal();
			break;
		case 'Q': //0x51
			error = 0;
			end_pmode();
			empty_reply();
			break;

		case 0x75: //STK_READ_SIGN 'u'
			read_signature();
			break;

		// expecting a command, not CRC_EOP
		// this is how we can get back in sync
		case CRC_EOP:
			error++;
			SERIAL.print((char) STK_NOSYNC);
			break;

		// anything else we will return STK_UNKNOWN
		default:
			error++;
			if (CRC_EOP == getch())
				SERIAL.print((char)STK_UNKNOWN);
			else
				SERIAL.print((char)STK_NOSYNC);
	}
}
