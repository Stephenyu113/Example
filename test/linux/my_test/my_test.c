/** \file
* \ Test code for SDO and PDO configuration
*
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <inttypes.h>
#include <math.h>

#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatdc.h"
#include "ethercatcoe.h"
#include "ethercatfoe.h"
#include "ethercatconfig.h"
#include "ethercatprint.h"

/*---------------------DEFINES--------------------*/
#define NSEC_PER_SEC 1000000000
#define stack64k (64 * 1024)
#define EC_TIMEOUTMON 500
#define Encoder_Resolution  32767.0 // 10v -- 16bITS
#define Counts_per_radian 52151.8917
#define Volts2Radian  2.0 // 5V -- 1 rad

/*--------------------VARIBLES--------------------*/
char IOmap[4096];
pthread_t  thread1, thread2;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
uint encoder_slave = 0; // Index of Encoder
uint pressure_slave = 0;// Index of Pressure Sensor
uint motor1_slave = 0;  // Index of Motor Drive 1
int64 toff, gl_delta;
int dorun = 0;
int64 Actual_Position = 0;
double encoder_raw, encoder;


/*---------------- DISTRIBUTED CLOCK----------------- */
/*
static int slave_dc_config (uint16 slave)
{
    ec_dcsync0(slave, 0, 0, 0);
    ec_dcsync0(slave, 1, 10000, 0);
    return 0;
}
*/


/*----------------------SDO-------------------------*/
static int motor_write8 (uint slave, uint16 index, uint8 subindex, uint8 value)
{
    int wkc;
    wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
    return wkc;
}

static int motor_write16 (uint slave, uint16 index, uint8 subindex, uint16 value)
{
    int wkc;
    wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
    return wkc;
}

static int motor_write32 (uint slave, uint16 index, uint8 subindex, uint value)
{
    int wkc;
    wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
    return wkc;
}

static int motor_setup (uint16 slave)
{
    int wkc = 0;

    printf ("Motor drive setup\n");

    wkc += motor_write8  (slave, 0x1C12, 0, 0);
    wkc += motor_write8  (slave, 0x1C13, 0, 0);

    // CSP_Inputs
    wkc += motor_write8  (slave, 0x1A00, 0, 0);
    wkc += motor_write32 (slave, 0x1A00, 1, 0x60410010); // Status Word
    wkc += motor_write32 (slave, 0x1A00, 2, 0x60640020); // Position Actual Value
    wkc += motor_write32 (slave, 0x1A00, 3, 0x606C0020); // Velocity Actual Value
    wkc += motor_write32 (slave, 0x1A00, 4, 0x60770010); // Torque Actual value
    wkc += motor_write32 (slave, 0x1A00, 5, 0x60F40020); // Position Error
    wkc += motor_write8  (slave, 0x1A00, 0, 5);

    // CSP_Outputs
    wkc += motor_write8  (slave, 0x1600, 0, 0);
    wkc += motor_write32 (slave, 0x1600, 1, 0x60400010); // Control Word
    wkc += motor_write32 (slave, 0x1600, 2, 0x607A0020); // Target Position
    wkc += motor_write32 (slave, 0x1600, 3, 0x60860010); // Velocity Offset
//    wkc += motor_write32 (slave, 0x1600, 4, 0x60B10020); // Torque Offset
    wkc += motor_write8  (slave, 0x1600, 0, 3);

    wkc += motor_write16 (slave, 0x1C12, 1, 0x1600);
    wkc += motor_write8  (slave, 0x1C12, 0, 1);

    wkc += motor_write16 (slave, 0x1C13, 1, 0x1A00);
    wkc += motor_write8  (slave, 0x1C13, 0, 1);
    /* Explicitly set flags that are (probably) invalid in EEPROM */
    ec_slave[slave].SM[2].SMflags = 0x10024;
	    
    /* Explicitly disable the sync managers that are activated by EEPROM */
    ec_slave[slave].SM[4].StartAddr = 0;
    ec_slave[slave].SM[5].StartAddr = 0;

    /* Set a slave name */
//    strncpy (ec_slave[slave].name, "MOTOR", EC_MAXNAME);

    if (wkc !=18)
    {
	printf ("Motor setup failed\n");
	return (-1);
    }

    return 0;
}


/*--------------------STRUCT----------------------*/
// Target can handle non aligned pointers to the IOmap
typedef struct PACKED
{
    uint16 value_6041; // Status Word
    int32  value_6064; // Position Actual Value
    int32  value_606C; // Velocity Actual Value
    int16  value_6077; // Torque Actual Value
    int32  value_60F4; // Position Error
} CSP_In;

typedef struct PACKED
{
    uint16 value_6040; // Control Word
    int32  value_607A; // Target Position
    int32  value_60B1; // Velocity Offset
} CSP_Out;

//struct sched_param schedp;
//struct timeval tv, t1, t2;


double encoder_pre[] = {0,0,0,0}; // Store x(k-2), x(k-1), y(k-2), y(k-1)
double encoder_filter(double encoder_raw, double *encoder_pre)
{
    double b0 = 0.01338, b1 = 0.02676, b2 = 0.01338, a1 = -1.65, a2 = 0.7035;
    double x = encoder_raw, y;
    y = b2*encoder_pre[0] + b1*encoder_pre[1] +b0*x - a2*encoder_pre[2] - a1*encoder_pre[3];
    encoder_pre[0] = encoder_pre[1]; encoder_pre[1] = x;
    encoder_pre[2] = encoder_pre[3]; encoder_pre[3] = y;
    return y;
}


/*--------------------TEST------------------------*/
void mytest(char *ifname)
{
    int cnt, i, chk=0;
    int j, iloop, oloop;
    needlf = FALSE;
    inOP = FALSE;

    printf("Starting my test\n");

    /* Initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
	printf("ec_init on %s succeeded.\n", ifname);

	/* Find and auto-config slaves */
	if (ec_config_init(FALSE) > 0)
	{
	    printf("%d slaves found and configured.\n", ec_slavecount);

	    if (ec_slavecount > 0)
	    {

		/* Configure distributed clock */
//	        ec_configdc();

		/* Locate the slave (motor drive) */
		for(cnt = 1; cnt <= ec_slavecount; cnt ++)
		{

//		    ec_dcsync0(cnt, 1, 10000, 0);
//		    ec_slave[cnt].PO2SOconfig = slave_dc_config;
		
//		    printf("Slave:%d Name:%s Output size:%3dbits Input size:%3dbits State:%2d delay:%d.%d\n", cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits, ec_slave[cnt].state, (int)ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
		    
//		    printf("	Out:%8.8x, %4d In:%8.8x,%4d\n", (int)ec_slave[cnt].outputs, ec_slave[cnt].Obits, (int)ec_slave[cnt].inputs, ec_slave[cnt].Ibits);		    

		    // Motor drive -- bel (slave1)
		    if((ec_slave[cnt].eep_man == 0x000000ab) && (ec_slave[cnt].eep_id == 0x00001110))
		    {
			motor1_slave = cnt;
			printf("Found %s at position %d\n", ec_slave[cnt].name, motor1_slave);
			ec_slave[motor1_slave].PO2SOconfig = motor_setup;
		    }
//		    // Encoder -- EL3064 (slave5)
//		    if((ec_slave[cnt].eep_man == 0x00000002) && (ec_slave[cnt].eep_id == 0x0bf83052))
//		    {
//			printf("Found %s at position %d\n", ec_slave[cnt].name, cnt);
//		    }
		    // Encoder -- EL3162 (slave2)
		    if((ec_slave[cnt].eep_man == 0x00000002) && (ec_slave[cnt].eep_id == 0x0c5a3052))
		    {
			encoder_slave = cnt;
			printf("Found %s at position %d\n", ec_slave[cnt].name, encoder_slave);
		    }
		}			
	    }
		
	    /* Configure I/O map */
	    ec_config_map(&IOmap);

	    /* Configure distributed clock */
	    ec_configdc();

	    printf("Slaves mapped, state to SAFE_OP.\n");
	    /* Wait for all slaves to reach SAFE_OP state */
	    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

	    /* Read individule slave state and store in ec_slave[] */
//	    ec_readstate();

	    printf("segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0], ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);

	   
	    /* Calculate workcounter */
	    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
	    printf("Calculated workcounter %d\n", expectedWKC);
	    
	    printf("Request operational state for all slaves\n");	
	    ec_slave[0].state = EC_STATE_OPERATIONAL;
	    
	    /* Send one valid process data to make outputs in slaves happy */
	    ec_send_processdata();
	    ec_receive_processdata(EC_TIMEOUTRET);

	    /* Request OP state for all slaves */
	    ec_writestate(0);

	    /* Activate cyclic process data */
	    dorun = 1;

	    /* Wait for all slaves to reach OP state */	
//	    ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
 
	    chk = 40;
	
	    /* Wait for all salves to reach OP state */
	    do
	    {
		ec_send_processdata();
		ec_receive_processdata(EC_TIMEOUTRET);
		ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
	    }
	    while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

	    /* Specify transfer bits for inputs and outputs */
	    oloop = ec_slave[0].Obytes;
	    if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
	    if (oloop > 8) oloop = 8;
	    iloop = ec_slave[0].Ibytes;
	    if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
	    if (iloop > 8) iloop = 8;

	    /* Transfer data */

	    if (ec_slave[0].state == EC_STATE_OPERATIONAL)
	    {
		printf("Operational state reached for all slaves.\n");
		inOP = TRUE;

		/* Output encoder values to a file */
		FILE *fp;
		char filename[] = "filter.dat";
		remove(filename);
		fp = fopen(filename, "a");

//		CSP_In *M1_In = (CSP_In *)ec_slave[motor1_slave].inputs;
		CSP_Out *M1_Out = (CSP_Out *)ec_slave[motor1_slave].outputs;
		M1_Out->value_6040 = (uint)15;
		//struct CSP_Out *target;

		/* Init Encoder */
 		encoder_raw = *(ec_slave[encoder_slave].inputs + 2)*256 + *(ec_slave[encoder_slave].inputs + 1);
		ec_send_processdata();
		ec_receive_processdata(EC_TIMEOUTRET);
		encoder_pre[0] = encoder_raw; encoder_pre[1] = encoder_raw; encoder_pre[2] = encoder_raw; encoder_pre[3] = encoder_raw;

		/* Cyclic loop */	
		for(i = 1; i <= 10000; i++)
		{
		    ec_send_processdata();
		    wkc = ec_receive_processdata(EC_TIMEOUTRET);

		    // Read Position (Encoder)
//		    encoder = *(ec_slave[encoder_slave].inputs + 2);
		    encoder_raw = *(ec_slave[encoder_slave].inputs + 2)*256 + *(ec_slave[encoder_slave].inputs + 1);
//		    encoder = encoder/3276.7;
//		    printf("Encoder: %d ", encoder);
		    encoder = encoder_filter(encoder_raw, encoder_pre);
//		    printf("Comment: %lf ", Counts_per_radian * encoder/Encoder_Resolution);
		    printf("filtered: %lf, raw: %lf ", encoder, encoder_raw);

		    fprintf(fp, "%d %lf %lf \n", i, encoder_raw, encoder); //File Operation		   

  		    /* Transfer PDO */
		    M1_Out->value_607A = Volts2Radian * Counts_per_radian * encoder/Encoder_Resolution;

		    if (wkc >= expectedWKC)
		    {
//			printf("Processdata cycle %5d , WKC %d, DCtime %12lld, dt %12lld, 0:", dorun, wkc, ec_DCtime, gl_delta);

			printf("Processdata cycle %4d, WKC %d , O:", i, wkc);	

			for(j = 0; j < oloop; j++)
			{
//			    printf(" %2.2x", *(ec_slave[0].outputs + j));
			}

			printf("I: ");
			for(j = 0; j < iloop; j++)
			{
//			    printf(" %x", *(ec_slave[0].outputs + j));
			}
//			printf("\r");
			fflush(stdout);
			printf(" T:%ld\r", ec_DCtime);
//			printf(" T:%lld, Actual Position: %d\r", ec_DCtime, Actual_Position);
			needlf = TRUE;
		    }
		    
		    usleep(5000);
		
		}
		dorun = 0;
		inOP = FALSE;
		fclose(fp);
	
	    }
	    else
	    {
		printf("Not all slaves reached operational state.\n");
		ec_readstate();
		for(i = 1; i <= ec_slavecount; i++)
		{
		    if(ec_slave[i].state != EC_STATE_OPERATIONAL)
		    {
			printf("Slave %d State=0x%2.2x StateCode=0x%4.4x : %s\n", i, ec_slave[i].state,
		     	ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
		    }
		}
	    }	
	    printf("\nRequest init state for all slaves\n");
	    ec_slave[0].state = EC_STATE_INIT;
	    /* Request INIT state for all slaves */
	    ec_writestate(0);
	}
	else
	{
	    printf("No slaves found!\n");
	}	
	printf("End my test, close socket\n");
	/* Stop SOEM, close socket */
	ec_close();
    }
    else
    {
	printf("NO socket connection on %s\nExcecute as root\n", ifname);
    }
}


/*----------------ADD ns To TIMESPEC---------------*/
void add_timespec(struct timespec *ts, int64 addtime)
{
    int64 sec, nsec;

    nsec = addtime % NSEC_PER_SEC;
    sec = (addtime - nsec) / NSEC_PER_SEC;
    ts->tv_sec += sec;
    ts->tv_nsec += nsec;
    if (ts->tv_nsec > NSEC_PER_SEC)
    {
	nsec = ts->tv_nsec % NSEC_PER_SEC;
	ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
	ts->tv_nsec = nsec;
    }
}


/* PI calculation to get linux synced to DC time */
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime)
{
    static int64 integral = 0;
    int64 delta;
    // set linux sync point 50us later than DC sync, just as example
//    delta = (reftime - 50000) % cycletime;
    delta = reftime % cycletime;
    if(delta> (cycletime / 2)) { delta = delta - cycletime; }
    if(delta>0) { integral++; }
    if(delta<0) { integral--; }
    *offsettime = -(delta / 100) - (integral / 20);
    gl_delta = delta;
}


/*------------------RT-THREAD---------------------*/
OSAL_THREAD_FUNC_RT ecatthread(void *ptr)
{
    struct timespec ts, tleft;
    int ht;
    int64 cycletime;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    ht = (ts.tv_nsec / 1000000) + 1; // round to nearest ms
    ts.tv_nsec = ht * 1000000;
    cycletime = *(int*)ptr * 1000; // cycletime in ns
    toff = 0;
    dorun = 0;
    ec_send_processdata();
    while(1)
    {
	/* calculate next cycle start */
	add_timespec(&ts, cycletime + toff);
	/* wait to cycle start */
	clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);
        if (dorun > 0)
        {
	    wkc = ec_receive_processdata(EC_TIMEOUTRET);

	    dorun++;
	 
	    if (ec_slave[0].hasdc)
	    {
		/* calculate toff to get linux time and DC synced */
		ec_sync(ec_DCtime, cycletime, &toff);
	    }
	    ec_send_processdata();
	}		
    }
}


/*--------------------ECATCHECK-------------------*/
OSAL_THREAD_FUNC ecatcheck(void *ptr)
{
    int slave;
    (void)ptr;  // Not used

    while(1)
    {
	if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate) )
	{
	    if (needlf)
	    {
		needlf = FALSE;
		printf("\n");
	    }
	
	    /* One or more slaves are not responding */
	    ec_group[currentgroup].docheckstate = FALSE;
	    ec_readstate();
	
	    for (slave = 1; slave <= ec_slavecount; slave++)
	    {
	
		if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
		{
		    ec_group[currentgroup].docheckstate = TRUE;
	
		    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
		    {
			printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
			ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
			ec_writestate(slave);
		    }
	
		    else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
		    {
			printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
			ec_slave[slave].state = EC_STATE_OPERATIONAL;
			ec_writestate(slave);
		    }
	
		    else if(ec_slave[slave].state > 0)
		    {
			if(ec_reconfig_slave(slave, EC_TIMEOUTMON))
			{
			    ec_slave[slave].islost = FALSE;
			    printf("MESSAGE : slave %d reonfigured\n", slave);
			}
		    }

		    else if(!ec_slave[slave].islost)
		    {
			/* Re-check state*/
			ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
			if(!ec_slave[slave].state)
			{
			    ec_slave[slave].islost = TRUE;
			    printf("ERROR : slave %d lost\n", slave);
			}
		    }
		}
		
		if(ec_slave[slave].islost)
		{
		   
		    if(!ec_slave[slave].state)
		    {
		
			if(ec_recover_slave(slave, EC_TIMEOUTMON))
			{
			    ec_slave[slave].islost = FALSE;
			    printf("MESSAGE : slave %d recovered\n", slave);
			}	
		    }
		
		    else
		    {
			ec_slave[slave].islost = FALSE;
			printf("MESSAGE : slave %d found\n", slave);
		    }
		}

	    }
		
		if(!ec_group[currentgroup].docheckstate)
		    printf("OK : all slaves resumed OPERATIONAL.\n");
	}
	usleep(10000);

    }
}


/*--------------------MAIN------------------------*/
int main(int argc, char *argv[])
{
    printf("SOEM (Simple Open EtherCAT Master)\nMy test\n");
    
    if (argc > 1)
    {
	dorun = 0;
//	int ctime = atoi(argv[2]);
	/* Create thread to handle slave error handling in OP*/
	osal_thread_create( &thread1, 128000, &ecatcheck, (void*) &ctime);
	
	/* Create RT thread */
//	osal_thread_create_rt(&thread1, stack64k * 2, &ecatthread, (void*) &ctime);

	/* Create thread to handle slave error handling in OP */
//	osal_thread_create(&thread2, stack64k * 4, &ecatcheck, NULL);
	
	/* Start cyclic part */
	mytest(argv[1]);	    
    }
    else
    {
	printf("Usage: my_test ifname\nifname = eth1 for example\n");
    }

    printf("End test\n");
	
    return (0);
}

















