#ifndef _myserial_h_
#define _myserial_h_

/*
inline void MyPutchar(char var);
void MyPrint(char *msg);
void MyPrintnum(int num);
int FreeRam() ;
void MyPrintln(char *msg, int num);
void serialprintPGM(PGM_P str);
*/

#include <avr/pgmspace.h>


extern USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface;
inline void MyPutchar(char var)
{
	CDC_Device_SendByte(&VirtualSerial_CDC_Interface, var);
}

static void MyPrint(char *msg)
{
	for(int i=0; msg[i] != '\0'; i++)
	{
		MyPutchar(msg[i]);
	}
}

static void MyPrintnum(unsigned int num) /* Print Number */
{
	char str[6];
	MyPrint( itoa(num, str, 10) );  //Base 10
}

/** Amount of free RAM - return The number of free bytes.  */
static int FreeRam() 
{
	extern char *__brkval;
	extern char __bss_end;
	char top;
	return __brkval ? &top - __brkval : &top - &__bss_end;
}



//#define MyEOL() do { MyPutchar('\r'); MyPutchar('\n'); } while(0)



//#define MyEOL()			MyPutchar('\r'); MyPutchar('\n')
#define MyPrintpgm(x)		serialprintPGM(PSTR(x))
#define MyPrintpgmln(x)		do { serialprintPGM(PSTR(x)); MyEOL();  } while(0)
#define MyPrintpair(str, num)	do { serialprintPGM(PSTR(str)); MyPrintnum(num); ;  } while(0)
#define MyPrintpairln(str, num)	do { serialprintPGM(PSTR(str)); MyPrintnum(num); MyEOL();  } while(0)

static void MyEOL()
{
	MyPutchar('\r'); 
	MyPutchar('\n');
}

static void MyPrintln(char *msg, int num)
{
	MyPrint(msg);
	MyPrintnum(num);
	MyEOL();
}


static void serialprintPGM(PGM_P str)
{
  char ch=pgm_read_byte(str);
  while(ch)
  {
    MyPutchar(ch);
    ch=pgm_read_byte(++str);
  }
}


static void printFreeMem()
{
	//dev_dbg("ram:%i\n",FreeRam());
	//char a[6];

	//MyPrint("fram:");
	//MyPrintnum ( FreeRam() );
	//MyPutchar('\n');
	//MyPrintln("fram:", FreeRam() );

	//MyPrintpgm( "pgm:" ); 
	//MyPrintnum ( FreeRam() );
	//MyPutchar('\n');
	MyPrintpairln("mem:", FreeRam() );
}

#define  pdev_dbg MyPrintlnpgm
#define  pdev_err MyPrintlnpgm
#define  pdev_warn MyPrintlnpgm



#endif
