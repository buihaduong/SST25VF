#include <msp430g2553.h>

/*
 * main.c
 */

//	MSP430	-	SST25VF
// Pin 1.1 UCA0SOMI -> Pin 2 SO
// Pin 1.2 UCA0SIMO -> Pin 5 SI
// Pin 1.4 UCA0CLK 	-> Pin 6 SCK
// Pin 1.5 GPIO		-> Pin 1 #CE
// Pin GND			-> Pin 4 VSS
// Pin VCC			-> Pin 8 VDD

#define SPI_MODE_0 (UCCKPH)			    /* CPOL=0 CPHA=0 */
#define SPI_MODE_1 (0)                 	/* CPOL=0 CPHA=1 */
#define SPI_MODE_2 (UCCKPL | UCCKPH)    /* CPOL=1 CPHA=0 */
#define SPI_MODE_3 (UCCKPL)			    /* CPOL=1 CPHA=1 */

#define SPI_MODE_MASK (UCCKPL | UCCKPH)

volatile char received_ch = 0;

void spiSetDataMode(int mode) {
	UCA0CTL1 |= UCSWRST;        // go into reset state
	switch (mode) {
	case 0: /* SPI_MODE0 */
		UCA0CTL0 = (UCA0CTL0 & ~SPI_MODE_MASK) | SPI_MODE_0;
		break;
	case 1: /* SPI_MODE1 */
		UCA0CTL0 = (UCA0CTL0 & ~SPI_MODE_MASK) | SPI_MODE_1;
		break;
	case 2: /* SPI_MODE2 */
		UCA0CTL0 = (UCA0CTL0 & ~SPI_MODE_MASK) | SPI_MODE_2;
		break;
	case 4: /* SPI_MODE3 */
		UCA0CTL0 = (UCA0CTL0 & ~SPI_MODE_MASK) | SPI_MODE_3;
		break;
	default:
		break;
	}
	UCA0CTL1 &= ~UCSWRST;       // release for operation
}

int spiSendByte(int data) {
//	while (!(IFG2 & UCA0TXIFG))
//		;   // USCI_A0 TX buffer ready?
	UCA0TXBUF = data;              // Send 0xAA over SPI to Slave
	while ((UCBUSY & UCA0STAT))
		;   // USCI_A0 RX Received?
	return UCA0RXBUF;       // Store received data
}

void init(void) {
	P1OUT &= (~BIT5);
	spiSendByte(0x50);
	P1OUT |= (BIT5);
	_delay_cycles(50);

	P1OUT &= (~BIT5);
	spiSendByte(0x01);
	spiSendByte(0x00);
	P1OUT |= (BIT5);
	_delay_cycles(100);
}

int readID() {
	int id = 0xFF, mtype = 0xFF, dev = 0xFF;
	P1OUT &= (~BIT5);
	spiSendByte(0x9F);
	id = spiSendByte(0x0);
	mtype = spiSendByte(0x0);
	dev = spiSendByte(0x0);
	P1OUT |= (BIT5);

	if (id == 0xBF && mtype == 0x25 && dev == 0x41)
		return 1;
	else
		return 0;
}

int bitRead(char b, int bitPos) {
	int x = b & (1 << bitPos);
	return x == 0 ? 0 : 1;
}

void waitUntilDone() {
	unsigned char data = 0;
	while (1) {
		P1OUT &= (~BIT5);
		spiSendByte(0x05);
		data = spiSendByte(0);
		P1OUT |= (BIT5);
		if (!bitRead(data, 0))
			break;
		_delay_cycles(10);
	}
}

void totalErase() {
	P1OUT &= (~BIT5);
	spiSendByte(0x06);
	P1OUT |= (BIT5);
	_delay_cycles(10);

	P1OUT &= (~BIT5);
	spiSendByte(0x60);
	P1OUT |= (BIT5);
	waitUntilDone();
}

void setAddress(unsigned long address) {
	spiSendByte(address >> 16);
	spiSendByte(address >> 8);
	spiSendByte(address);
}

void readInit(unsigned long address) {
	P1OUT &= (~BIT5);
	spiSendByte(0x03);
	setAddress(address);
}

unsigned char readNext() {
	return spiSendByte(0x00);
}

void readFinish() {
	P1OUT |= (BIT5);
}

void writeByte(unsigned long address, unsigned char data) {
	P1OUT &= (~BIT5);
	spiSendByte(0x06);
	P1OUT |= (BIT5);
	_delay_cycles(10);

	P1OUT &= (~BIT5);
	spiSendByte(0x02);
	setAddress(address);
	spiSendByte(data);
	P1OUT |= (BIT5);
	waitUntilDone();
}

void sectorErase(unsigned char sectorAddress) {
	P1OUT &= (~BIT5);
	spiSendByte(0x06);
	P1OUT |= (BIT5);
	_delay_cycles(10);

	P1OUT &= (~BIT5);
	spiSendByte(0x20);
	setAddress((unsigned long) 4096 * (long) sectorAddress);
	P1OUT |= (BIT5);
	waitUntilDone();
}

int main(void) {
	WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

	P1OUT |= BIT5;
	P1DIR |= BIT5;
	P1SEL = BIT1 | BIT2 | BIT4;
	P1SEL2 = BIT1 | BIT2 | BIT4;

	UCA0CTL1 = UCSWRST;
	UCA0CTL0 |= UCCKPH + UCMSB + UCMST + UCSYNC;  // 3-pin, 8-bit SPI master
	UCA0CTL1 |= UCSSEL_2;                     // SMCLK
	UCA0BR0 |= 0x02;                          // /2
	UCA0BR1 = 0;                              //
	UCA0MCTL = 0;                             // No modulation
	UCA0CTL1 &= ~UCSWRST;                   // **Initialize USCI state machine**

	spiSetDataMode(0);

	init();
	if (!readID())
		return 0;

	unsigned char i, q;

	for (i = 0; i < 22; i++) {
		writeByte((unsigned long) i, i*9);
	}

	long x = 0;
	readInit((unsigned long) 4096 * x);
	for (q = 0; q < 22; q++) {
		received_ch = readNext();
		if (received_ch == 0xFF)
			break;
	}
	readFinish();

	sectorErase(0);

	readInit((unsigned long) 4096 * x);
	for (q = 0; q < 22; q++) {
		received_ch = readNext();
		if (received_ch == 0xFF)
			break;
	}
	readFinish();
}
