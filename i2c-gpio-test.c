#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/time.h>
#include <stddef.h>

#define BIT0 0x01
#define BIT1 0x02
#define BIT2 0x04
#define BIT3 0x08
#define BIT6 0x40
#define BIT15 0x8000 

#define I2C_ADDRESS 0x27

// I2C Linux device handle
int g_i2cFile;

// open the Linux device
void i2cOpen()
{
	g_i2cFile = open("/dev/i2c-1", O_RDWR);
	if (g_i2cFile < 0) {
		perror("i2cOpen");
		exit(1);
	}
}

// close the Linux device
void i2cClose()
{
	close(g_i2cFile);
}

// set the I2C slave address for all subsequent I2C device transfers
void i2cSetAddress(int address)
{
	if (ioctl(g_i2cFile, I2C_SLAVE, address) < 0) {
		perror("i2cSetAddress");
		exit(1);
	}
}

// write a 16 bit value to a register pair
// write low byte of value to register reg,
// and high byte of value to register reg+1
void pca9555WriteRegisterPair(uint8_t reg, uint16_t value)
{
	uint8_t data[3];
	data[0] = reg;
	data[1] = value & 0xff;
	data[2] = (value >> 8) & 0xff;
	if (write(g_i2cFile, data, 3) != 3) {
		perror("pca9555SetRegisterPair");
	}
}

// read a 16 bit value from a register pair
uint16_t pca9555ReadRegisterPair(uint8_t reg)
{
	uint8_t data[3];
	data[0] = reg;
	if (write(g_i2cFile, data, 1) != 1) {
		perror("pca9555ReadRegisterPair set register");
	}
	if (read(g_i2cFile, data, 2) != 2) {
		perror("pca9555ReadRegisterPair read value");
	}
	return data[0] | (data[1] << 8);
}

// set IO ports to input, if the corresponding direction bit is 1,
// otherwise set it to output
void pca9555SetInputDirection(uint16_t direction)
{
	pca9555WriteRegisterPair(6, direction);
}

// set the IO port outputs
void pca9555SetOutput(uint16_t value)
{
	pca9555WriteRegisterPair(2, value);
}

// read the IO port inputs
uint16_t pca9555GetInput()
{
	return pca9555ReadRegisterPair(0);
}


/* http://www.gnu.org/software/libtool/manual/libc/Elapsed-Time.html
Subtract the `struct timeval' values X and Y,
storing the result in RESULT.
Return 1 if the difference is negative, otherwise 0.  */
int timeval_subtract (struct timeval *result, struct timeval *x, struct timeval *y)
{
    /* Perform the carry for the later subtraction by updating y. */
    if (x->tv_usec < y->tv_usec) {
     int nsec = (y->tv_usec - x->tv_usec) / 1000000 + 1;
     y->tv_usec -= 1000000 * nsec;
     y->tv_sec += nsec;
    }
    if (x->tv_usec - y->tv_usec > 1000000) {
     int nsec = (x->tv_usec - y->tv_usec) / 1000000;
     y->tv_usec += 1000000 * nsec;
     y->tv_sec -= nsec;
    }

    /* Compute the time remaining to wait.
      tv_usec is certainly positive. */
    result->tv_sec = x->tv_sec - y->tv_sec;
    result->tv_usec = x->tv_usec - y->tv_usec;

    /* Return 1 if result is negative. */
    return x->tv_sec < y->tv_sec;
}


//CTRL+C handler
int execute;
void trap(int signal) {execute = 0;}


int main(int argc, char** argv)
{
    int cyclesPass = 0, cyclesFailed = 0, tempFailed; 

    int totlBytes = 0; 

    int buttonPressed;

    struct timeval timeStart, timeEnd, timeDelta;
    float deltasec;

    i2cOpen();
    i2cSetAddress(I2C_ADDRESS);

    //CTRL+C handler
    signal(SIGINT,&trap);
    execute = 1; 

    pca9555SetInputDirection(~(BIT0 + BIT1)); // GPIO 0.0 and 0.1 as output

    gettimeofday( &timeStart, NULL );

    fprintf(stdout, "\n+++START+++\n");
    fprintf(stdout, "CTRL+C to exit\n"); 

    uint16_t regOut = 0xFFFF;
    uint16_t regIn = 0xFFFF;

    regOut ^= BIT0; // change BIT0 from default to level will be changed
    pca9555SetOutput(regOut);

    do
    {
        tempFailed = 0;

        regOut ^= (BIT0+BIT1);
        pca9555SetOutput(regOut);
        regIn = pca9555GetInput();

        if ( (regOut & BIT0 ) != ( (regIn & BIT2) >> 2 ) )
        {
            fprintf(stdout, "!!!ERROR: value written to GPIO 0.0 does not match with read from GPIO 0.2\n" );
            tempFailed++;
        }
        totlBytes++;

        if ( ((regOut & BIT1 ) >> 1 ) != ( (regIn & BIT3) >> 3 ) )
        {
            fprintf(stdout, "!!!ERROR: value written to GPIO 0.1 does not match with read from GPIO 0.3\n" );
            tempFailed++;
        }
        totlBytes++;

        if ( !( regIn & BIT15) && !(buttonPressed) )
        {
            fprintf(stdout, "User button is pressed.\n");
            buttonPressed = 1;
        }

        if ( ( regIn & BIT15) && (buttonPressed) )
        {
            fprintf(stdout, "User button is released.\n");
            buttonPressed = 0;
        }

        if ( tempFailed )
            cyclesFailed++;
        else
            cyclesPass++;

    }while(execute);

    //CTRL+C handler
    signal(SIGINT,SIG_DFL);

    gettimeofday( &timeEnd, NULL );

    i2cClose();

    timeval_subtract(&timeDelta, &timeEnd, &timeStart);
    deltasec = timeDelta.tv_sec+timeDelta.tv_usec*1e-6;

    fprintf(stdout, "\n+++DONE+++\n");
    fprintf(stdout, "CYCLES PASS: %d FAILED: %d\n", cyclesPass, cyclesFailed);
    fprintf(stdout, "Total: %d bytes. Test time: %ld s %ld us. \n", totlBytes, timeDelta.tv_sec, timeDelta.tv_usec);
    fprintf(stdout, "Measured: total %.02f Bps.\n", totlBytes/deltasec);

    return 0;
}
