/*=========================================================================
 | (c)   BioMechatronics Lab - UCLA
 |--------------------------------------------------------------------------
 | Project  : Bair Claw Main/Interface
 | File     : BairClawMain.cpp
 | Author   : Randy Hellman (RBHellman@gmail.com, RHellman@asu.edu)
 |          : Cheng-Ju Wu (ws1008taiwan@gmail.com)
 | Instrutor: Veronica Santos (vjsantos@ucla.edu)
 |--------------------------------------------------------------------------
 | Function: Main entry point for BairClaw controls CAN communication between
 | 			BairClaw and EPOS motor controllers. Adapted from ex05/sg_log
 |           and etc...
 |--------------------------------------------------------------------------
 import current receival
 ========================================================================*/

#include <iostream> // standard C++ I/O stream
#include <cstdio>   // standard C   I/O stream
#include <fstream>  // standard C++ file I/O stream
#include <sstream>  // standard C++ string stream
#include <vector>
#include <string>
#include <cmath>

#include <unistd.h>		// UNIX standard library
#include <sys/mman.h>	// UNIX memory allocation lock library
#include <termios.h>    // UNIX terminal control definitions
#include <getopt.h>     // UNIX command-line arguments parsing functions

#include <boost/thread.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/tuple/tuple.hpp>
//Boost martix allows easy matrix maniulation
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <barrett/os.h>  // For btsleep()
#include <barrett/bus/can_socket.h>
// Base class: barrett::systems::System
#include <barrett/systems/abstract/system.h>				// USED : Input, Output, AbstractInput, AbstractOutput, and structure/class definitions 
#include <barrett/systems.h>								// USED : includes most realtime and thread managing Barrett libraries (ExecutionManager, etc.)
#include <barrett/math.h>									// USED : (--> matrix.h) Vector<size>, which is math::Matrix<R,1,Units=void>
															// inherits Eigen/Core library, a premade library for linear algebra
//#include <barrett/units.h>
#include <barrett/log.h>									// USED : RealTimeWriter
#include <barrett/products/product_manager.h>				// USED : BCProductManager
//Xenomia RTThread REQ
#include <native/task.h>   									// USED : include <native/types.h> such as RTIME, define rt_tasks
#include <native/timer.h>									// USED : timer used within rt_tasks. rt_timer_ns2tcks, rt_timer_read(): for rt-suitable timer (in ns)
//BairClaw Defined Functions and Classes
#include "BairClawInterface.h"								// USED : includes EPOS library, uses namespaces robotics and
#include "BairClawDataPlayBack.h"   						// USED : slowly remove or change
//BioTac REQUIRED
#include "BCbiotac.h"										// USED : includes cheetah.h, provided by SynTouch
#include "filters.h" //%%%%%%
// libraries for Unix UDP via Internet (UDP/IP)
#include <sys/types.h>										// USED : data types used in system calls, used in the next two include files.
#include <arpa/inet.h>										// USED : definitions for internet operations, includes <netinet/in.h>, which defines
															//        constants and structures needed for internet domain (IP addresses).
#include <sys/socket.h>										// USED : UNIX IP : data structures and defines sockets

#define DEBUG true
#define POSITIONCONTROL true
#define biotacOff false

using namespace barrett;
using namespace bairclaw; // for new BCInterface
using systems::connect;



/* ======================================================================== */
/*  Global Variables */
/* ======================================================================== */

/*  --- CAN-bus ----------------------------------------------------------- */
const int CAN_PORT = 1; //0 or 1
const bus::CANSocket CANbus(CAN_PORT);


/*  --- BairClaw ---------------------------------------------------------- */
const int NUM_OF_DIGITS = 1;
//const int NUM_OF_DIGITS = 2;
//bt_single_batch data[2];		// %%%%%%%%%%% NEW 
BCHand bairClaw(NUM_OF_DIGITS, &CANbus);

const int JOINT_PER_DIGIT = 4; 
const int MOTOR_PER_DIGIT = 2;		// %%%%%%%%%%% OLD  
const int TENDON_PER_DIGIT = 4; 
//const int MOTOR_PER_DIGIT = 3;	// %%%%%%%%%%% NEW
//const int TENDON_PER_DIGIT = 6;

// Data of 1 digit: 1 CAN_counter, 1 CAN_counter, 4 joint, 6 tendon, 3 motor
const int BC_NUM_VAR_RECORD = 1 + JOINT_PER_DIGIT + \
                                    MOTOR_PER_DIGIT + TENDON_PER_DIGIT;
                                    
// Data of 1 BioTac: 1 btCounter, 44 BioTac standardized output (1 Pdc, 1 Tac,
// 1 Tdc, 19 electrodes, 22 Pac)
const int BT_NUM_VAR_RECORD = 45;
	
/* Total of logged data = 1 time + 1 bcCounter + digits*(BC_NUM_VAR_RECORD + 
                          BT_NUM_VAR_RECORD) + 1 controller_mode + 1 status
   For new layout, should be 123. */


/*  --- RealTime Operation ------------------------------------------------ */
// Thread-shared Variables
static SRTIME delay = 500000; // in nanoseconds, = 500us

/* PlayBackData: came from the idea of WAM PlayBackData 
   Pre-records a trajectory and replays */
dataRecording *playbackData = new dataRecording(); // structure tbd, need to be global to be accessed by BCProductManager

// Matlab RT visualizer
std::string host; // reads in first argument IP address (receiver IP)





// Current Control Global ----//
int setCurrent = 0 ;
double static const Kp = 0.97;
double static const Kd = 0.01;
double static const Ki = 7.0;

double tensionDesired = 1500;
int node3currSetCount = 0, node3error = 0, A2setCounter = 0;
double error, integral, derivative, previous_error;


// Real-time controller state machine variables
enum controlState {positionControl, pdcControl, idle};
controlState currentControl = positionControl;
int settingStatus = 0;

// Global Shared Mutexs
boost::condition_variable cv;
boost::mutex mut;
bool ready = false;
bool going = true;
bool displayOn = false;
bool biotacInit = false;
bool shouldStart = false;



typedef math::Vector< BT_NUM_VAR_RECORD >::type btSizedVector; // Vector sized for BioTac [barrett/math.h]
typedef math::Vector< BC_NUM_VAR_RECORD >::type bcSizedVector; // Vector sized for BairClaw
btSizedVector btInput;
bcSizedVector bcInput;
systems::ExposedOutput<btSizedVector> bt_EoSys;      // used for BIOTAC   data harvesting from EPOS to C++ interface to data structure
systems::ExposedOutput<bcSizedVector> bc_EoSys;      // used for BAIRCLAW data harvesting from EPOS to C++ interface to data structure
/* Exposed Outputs are required in global for passing data vectors between threads. 
   In this case, from can_receive/bt_record to BCProductManager/tupleGrouper
   Will be required to be expandable for multiple digits, and assigning would matter %%%%%%%%%%%%%%%%%%%%%%%*/
   
//btSizedVector btInput[NUM_OF_DIGITS];
//bcSizedVector bcInput[NUM_OF_DIGITS];
//systems::ExposedOutput<btSizedVector> bt_EoSys[NUM_OF_DIGITS];      // used for BIOTAC   data harvesting from EPOS to C++ interface to data structure
//systems::ExposedOutput<bcSizedVector> bc_EoSys[NUM_OF_DIGITS];      // used for BAIRCLAW data harvesting from EPOS to C++ interface to data structure






#pragma mark - BCProductManager
class BCProductManager : public systems::System {
	/* Single BC Digit Black Box 
	   Input: bcInput (14: 1 CAN_counter, 4 joint, 6 tendon, 3 motorCurrent)
	     from can_receive_thread
	  Output: (17: 1 bc_counter, 1 CAN_counter, 4 joint, 6 tendon, 3 motorCurrent, 1 controlState, 1 settingStatus)
	*/
	// 1 bc_counter, 1 CAN_counter, 4 joint, 6 tendon, 3 motorCurrent, 1 controlState, 1 settingStatus
                                    //  = 17, maybe 1 more for bcState?
                                    
    // IO
    // Marked as "public" because Inputs and Output are (except in special cases)
    // part of a System's public interface.
public:
    Input<math::Vector< BC_NUM_VAR_RECORD >::type> input;
    Output<int> output;
    Output<math::Vector< BC_NUM_VAR_RECORD >::type> jointsOutput;
    int countCheck;

	// Every System has a human readable name. It's good practice to provide an
	// appropriate default. Notice that outputValue is associated with output
	// via output's constructor.
	explicit BCProductManager(const std::string& sysName = "BairClawProductManager") : systems::System(sysName), input(this), output(this, &outputValue),jointsOutput(this, &jointOutput),  countCheck(0)
    {
        error = 0;
        integral = 0;
        derivative = 0;
        previous_error = 0;
    }
    
	// Every System is required to call System::mandatoryCleanUp() in its
	// destructor, preferably as early as possible. It's common for libbarrett
	// to be used in a multi-threaded environment. This function cleans up all
	// of libbarrett's references to this System so that the library won't try
	// to interact with it from Thread A while it's in the process of being
	// destroyed in Thread B. If you forget this, you may occasionally see your
	// program crash with the message: "Pure virtual function called".
	virtual ~BCProductManager() { mandatoryCleanUp(); }
    
    int getCount()
    {
        return countCheck;
    }
    
    // Marked as "protected" because this object lets us change the value of an
    // Output, which should only be allowed from within this System.
protected:
    Output<int>  ::Value* outputValue;
    Output<math::Vector< BC_NUM_VAR_RECORD >::type>::Value* jointOutput;
    math::Vector< BC_NUM_VAR_RECORD >::type joints;  
	// Implement System::operate(). The operate() function must be declared with
	// the "protected" access specifier.
	virtual void operate() {
		joints = input.getValue();  // Pull data from the input
        
        
        static int count = 0;
        count++;
        countCheck++;     

        
        if (shouldStart == true && playbackData->dataCount > 0)
        {
            tensionDesired = playbackData->getDataValueAtIndex(count % (playbackData->dataCount-2));
        }
        else
        {
            tensionDesired = 1500;
        }

        
        
        //Initial setup for PID to control current on Flexion of MCP
        error = tensionDesired - (double)bairClaw.digit[0].FEmotor.A2;
        integral = integral + error * 0.005;
        derivative = (error - previous_error) / 0.005;
        if(!DEBUG){
            setCurrent = (int) ( (Kp * error) + (Ki * integral) + (Kd * derivative) );
        }
        else{
            setCurrent = 0;
        }
        previous_error = error;
        
        rt_task_sleep(rt_timer_ns2ticks(delay));//
        //Should be moved into SetCurrent to account for static friciton///////////////////////////
        if(setCurrent < 0)/////////////////////////////////////////////////////////////////////////
        {
            setCurrent =   abs(setCurrent) + abs(bairClaw.digit[0].FEmotor.staticFrictionE);
        }else if(setCurrent > 0)
        {
            setCurrent =   -(setCurrent + abs(bairClaw.digit[0].FEmotor.staticFrictionF));
        }else
        {
            setCurrent = 0;
        }//////////////////////////////////////////////////////////////////////////////////////////
        bairClaw.digit[0].FEmotor.SetCurrent( setCurrent);
        rt_task_sleep(rt_timer_ns2ticks(delay));//
        bairClaw.digit[0].PIPmotor.SetCurrent(0);
        rt_task_sleep(rt_timer_ns2ticks(delay));//
        bairClaw.digit[0].ADABmotor.SetCurrent(setCurrent);//--------------------------
        rt_task_sleep(rt_timer_ns2ticks(delay));//

        //REQUESTS EPOS analog values at same freq as BCProductManager
        bairClaw.digit[0].FEmotor.readAnalog1();
        rt_task_sleep(rt_timer_ns2ticks(delay));//
        bairClaw.digit[0].FEmotor.readAnalog2();
        rt_task_sleep(rt_timer_ns2ticks(delay));//
        bairClaw.digit[0].PIPmotor.readAnalog1();
        rt_task_sleep(rt_timer_ns2ticks(delay));//
        bairClaw.digit[0].PIPmotor.readAnalog2();
        
        outputValue->setData(&count);
        jointOutput->setData(&joints);
        

        
	}
    
    
};
#pragma mark - displayThread
void displayThread()
{
    while(going)
    {
        while(displayOn && going)
        {
            if(system("clear")) {
            	printf("Error from displayThread : system(\"clear\") failed");
			}
            printf("Display- \n");
            printf("MCP_F - %d (mV), MCP_E - %d (mV), setCount - %d\n", bairClaw.digit[0].FEmotor.A2,bairClaw.digit[0].FEmotor.A1, A2setCounter/2);
            printf("PIP_F - %d (mV), PIP_E - %d (mV), setCount - %d\n", bairClaw.digit[0].PIPmotor.A2,bairClaw.digit[0].PIPmotor.A1);
            printf("SetCurrent - %d (mA), demandCurr - %d\n", setCurrent, bairClaw.digit[0].ADABmotor.currDemand);
            printf("error - %4.2f, int - %4.2f, deriv - %4.2f, pre-error - %4.2f \n", error, integral, derivative, previous_error);
            printf("FEset   - %d, error - %d\n", bairClaw.digit[0].FEmotor.currSetCount, bairClaw.digit[0].FEmotor.currSetError);
            printf("PIPset  - %d, error - %d\n", bairClaw.digit[0].PIPmotor.currSetCount, bairClaw.digit[0].PIPmotor.currSetError);
            printf("AdAbset - %d, error - %d\n", bairClaw.digit[0].ADABmotor.currSetCount, bairClaw.digit[0].ADABmotor.currSetError);
            printf("Digit - [%d, %d, %d, %d]\n",bairClaw.digit[0].jointVal[0], bairClaw.digit[0].jointVal[1], bairClaw.digit[0].jointVal[2], bairClaw.digit[0].jointVal[3]);
           
            
            printf("Digit JointPercent-\n[%4.2f, %4.2f, %4.2f, %4.2f]\n",bairClaw.digit[0].jointPercent[0], bairClaw.digit[0].jointPercent[1], bairClaw.digit[0].jointPercent[2], bairClaw.digit[0].jointPercent[3]);
            printf("Digit JointAngleRad  -\n[%4.2f, %4.2f, %4.2f, %4.2f]\n",bairClaw.digit[0].jointValRad[0], bairClaw.digit[0].jointValRad[1], bairClaw.digit[0].jointValRad[2], bairClaw.digit[0].jointValRad[3]);
            printf("Digit TendonForce -\n[%4.2f, %4.2f, %4.2f, %4.2f]\n",bairClaw.digit[0].mcpFest, bairClaw.digit[0].mcpEest, bairClaw.digit[0].pipFest, bairClaw.digit[0].pipEest);
            /*
            std::cout << "Jacobian" << std::endl << std::endl << std::endl;
            std::cout << std::setprecision(5) << std::fixed << bairClaw.digit[0].DHp.jacobian << std::endl;
            std::cout << "jacobianTransposePseudoInverse()" << std::endl << std::endl;
            std::cout << bairClaw.digit[0].DHp.jacobianTransposePseudoInverse << std::endl;
            std::cout << "jacobianActuation" << std::endl;
            std::cout << bairClaw.digit[0].DHp.jacobianActuation << std::endl;
            */
            std::cout << "loadCell" << std::endl;
            std::cout << std::endl << "endEffectorForce" << std::endl;
            
            
            printf("\nP_dc - %4.2f\n", btInput[1]);
            printf(" currentControl  - %d\n", currentControl);
            printf(" settingStatus - %d\n", settingStatus);
            printf("\nPress [Enter] to stop recording\n");
            
            usleep(100000);
        }
        usleep(500000);
    }
    
}





// secondsToRTIME - takes defined task period in seconds and casts into RTIME in nanoseconds 
#pragma mark - RTaskFunctions
RTIME secondsToRTIME(double s) {
	return static_cast<RTIME>(s * 1e9);
}



// Soft realtime
double period_visualizer = 0.02;  // 20000us = 20ms = 0.02s = 50Hz
#pragma mark - UDP/IP communication for BairClaw MATLAB visualizer
void bairClawMatlabVisualizerSender()
{
    printf(" > In bairClaw MATLAB vis");
    const int BUFLEN = 4096;
    const int PORT = 5678; // needs to an available udp port. MATLAB must match
    
    
    struct sockaddr_in serv_addr;
    int sockfd, i, j, slen = sizeof(serv_addr);
    char buf[BUFLEN];
    
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
    	fprintf(stderr, "Error from VisualizerSender: socket assigning error\n");
        exit(1);
	}
    
    bzero(&serv_addr, sizeof(serv_addr)); // zeros serv_addr
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);
    if (inet_aton(host.c_str(), &serv_addr.sin_addr)==0) {
        fprintf(stderr, "Error from VisualizerSender: inet_aton() failed\n");
        exit(1);
    }
    
    while(going)
    {
        memset(buf, 0, BUFLEN);
        // (Constructing buf) For each digit...
        for (j = 0; j < NUM_OF_DIGITS; ++j) {
        	// Send joint angles
        	for (i = 0; i < 4; ++i) {
            	sprintf(buf + strlen(buf), "%d,", bairClaw.digit[j].jointVal[i]);
//            	sprintf(buf + strlen(buf), "%d,", bairClaw.digit[j].jointVal[i]); // NEW
        	}
        	// Send tendon tension
        	sprintf(buf + strlen(buf), "%d,%d,%d,%d,", bairClaw.digit[j].PIPmotor.A1, bairClaw.digit[j].PIPmotor.A2, bairClaw.digit[j].FEmotor.A1, bairClaw.digit[j].FEmotor.A2);
//        	sprintf(buf + strlen(buf), "%d,%d,%d,%d,%d,%d,", bairClaw.digit[j].PIPmotor.A1, bairClaw.digit[j].PIPmotor.A2,\
//															 bairClaw.digit[j].FEmotor.A1, bairClaw.digit[j].FEmotor.A2,\
//															 bairClaw.digit[j].ABADmotor.A1, bairClaw.digit[j].ABADmotor.A2 );
        	// Send BioTac data
        	for (i = 0; i < BT_NUM_VAR_RECORD; ++i) {
            	sprintf(buf + strlen(buf), "%d,", int(btInput[i]));
//            	sprintf(buf + strlen(buf), "%d,", int(btInput[j][i])); // NEW
        	}
		}
//        sprintf(buf + strlen(buf), "\n");

//        if (strcmp(buf,"exit") == 0)
//            exit(0);	// leaves thread
        
        if (sendto(sockfd, buf, BUFLEN, 0, (struct sockaddr*)&serv_addr, slen)==-1) {
    		perror("Error from VisualizerSender: sendto(), "); return;
		}
        usleep(period_visualizer*1000000);
    }
    // inform MATLAB to exit loop
    memset(buf, 0, BUFLEN);
    sprintf(buf,"EXIT");
    for (i = 0; i < 10; ++i) {
        if (sendto(sockfd, buf, BUFLEN, 0, (struct sockaddr*)&serv_addr, slen)==-1) {
    		perror("Error from VisualizerSender: sendto(), "); return;
		}
        usleep(50000);
    }
    close(sockfd);
}






#pragma mark - BioTac Record Thread
double period_biotac = 0.01; // in seconds
void biotacRecordThread(void *arg)
{
    /****************************/
	/* --- Define variables --- */
	/****************************/
    bt_info biotac;
    bt_property biotac_property[MAX_BIOTACS_PER_CHEETAH];
	bt_single_batch data;
	BioTac bt_err_code;
	Cheetah ch_handle;
	int i, count = 0;
	double length_of_data_in_second;
	int number_of_samples;
	int number_of_loops;
    
    if(!biotacOff)
    {
        /**************************************************************************/
        /* --- Initialize BioTac settings (only default values are supported) --- */
        /**************************************************************************/
        biotac.spi_clock_speed = BT_SPI_BITRATE_KHZ_DEFAULT;
        biotac.number_of_biotacs = 0;
        biotac.sample_rate_Hz = BT_SAMPLE_RATE_HZ_DEFAULT;
        biotac.frame.frame_type = 0;
        biotac.batch.batch_frame_count = 1; //BT_FRAMES_IN_BATCH_DEFAULT;
        biotac.batch.batch_ms = 10; 		//BT_BATCH_MS_DEFAULT;
        /* BT_SAMPLE_RATE_HZ_DEFAULT = 44 * (1000/batch_ms) * batch_frame_count
		   Since sampling rate is locked at 4400Hz for each sample, to have 100Hz
		   sampling on all data (44 samples), can only send 1 frame per patch.*/

        
        // Set the duration of the run time
        length_of_data_in_second = 0.1; // 100Hz
        number_of_samples = (int)(BT_SAMPLE_RATE_HZ_DEFAULT * length_of_data_in_second);
//        printf(" number_of_samples - %d", number_of_samples);
        
        // Check if any initial settings are wrong
        if (MAX_BIOTACS_PER_CHEETAH != 3 && MAX_BIOTACS_PER_CHEETAH != 5)
        {
            bt_err_code = BT_WRONG_MAX_BIOTAC_NUMBER;
            bt_display_errors(bt_err_code);
            exit(1);
        }
        /******************************************/
        /* --- Initialize the Cheetah devices --- */
        /******************************************/
        ch_handle = bt_cheetah_initialize(&biotac);
        
        /*********************************************************/
        /* --- Get and print out properties of the BioTac(s) --- */
        /*********************************************************/
        
        for (i = 0; i < MAX_BIOTACS_PER_CHEETAH; ++i)
        {
            bt_err_code = bt_cheetah_get_properties(ch_handle, i+1, &(biotac_property[i]));
            
            if (biotac_property[i].bt_connected == YES)
            {
                (biotac.number_of_biotacs)++;
            }
            
            if (bt_err_code)
            {
                bt_display_errors(bt_err_code);
                exit(1);
            }
        }
        
        // Check if any BioTacs are detected
        if (biotac.number_of_biotacs == 0)
        {
            bt_err_code = BT_NO_BIOTAC_DETECTED;
            bt_display_errors(bt_err_code);
        }
        else
        {
            printf("\n%d BioTac(s) detected.\n\n", biotac.number_of_biotacs);
        }

        
        /*******************************/
        /* --- Configure the batch --- */
        /*******************************/
        bt_err_code = bt_cheetah_configure_batch(ch_handle, &biotac, number_of_samples);
        if (bt_err_code < 0)
        {
            bt_display_errors(bt_err_code);
            exit(1);
        }
        else
        {
            printf("\nConfigured the batch\n");
        }
        
        printf("about to collect \n");
        number_of_loops = (int)(number_of_samples / ((double)(biotac.frame.frame_size * biotac.batch.batch_frame_count)));
        printf("Start collecting BioTac data for %4.0f seconds(s)\n\tnumber_of_loops - %d \n", length_of_data_in_second,number_of_loops);
        
        printf("Press [Enter] to continue ...");
        fflush(stdout);
        getchar();
        
        biotacInit = true; //Flag to tell main that biotac is init.
        

        rt_task_set_periodic(NULL, TM_NOW, secondsToRTIME(period_biotac));
        while(!shouldStart)
        {
            rt_task_wait_period(NULL);
        }
//        RTIME now;
		RTIME previous;
        
        
        
        
        previous = rt_timer_read();
        while (going)
        {
            rt_task_wait_period(NULL);
        
            bt_cheetah_collect_single_batch(ch_handle, &biotac, &data, NO);
            
            btInput[0] = count++; //Counter to ensure fresh batch of data
            btInput[1] = data.bt[0].pdc;
            btInput[2] = data.bt[0].tac;
            btInput[3] = data.bt[0].tdc;
            
            for(i=0; i<19; i++) //Set electrode values into logget
            {
                btInput[i+4]= data.bt[0].elec[i];
            }
            
            for(i=0; i<22; i++) //Set pac into a singe row to be pulled out
            {
                btInput[i+23] = data.bt[0].pac[i];
            }
            
            bt_EoSys.setValue(btInput);
        }

    }
}





// canReceiveThread - 
#pragma mark - canThread
double period_canBus = 0.0001; // 10000 Hz, in seconds
void canReceiveThread(void *arg) {
	
	/*  Received data structure from CAN bus:
			4 Joint angles : through Arduino, 10-bit
			4 Tendon forces (6) : through EPOS, 12-bit
			2 Motor currents (3) : 16-bit
		Packaging: Add 1 CAN_counter */
	int CAN_counter = 0;
	
	
	/* Local Variables */ 
    int ret;			// check if function calls are valid
	int id, SDO, i;		// CAN-ID, SDO instruction
	size_t len;			// Data array length
    EPOS2* currentEPOSController = NULL;
	unsigned char data[bus::CANSocket::MAX_MESSAGE_LEN];

	
    /* Make a real-time task periodic. (Subsequent calls to rt_task_wait_period()
	   will delay the task until the next periodic release point is reached.)
		int rt_task_set_periodic ( RT_TASK*	task, RTIME	idate, RTIME period )	
			task	Address of affected task. If task is NULL, current task is selected.
			idate	Initial absolute time point of first release point (unit: ns). 
					If = TM_NOW, use current system date, no initial delay takes place.
			period	period of task in nanoseconds.
	*/
    rt_task_set_periodic(NULL, TM_NOW, secondsToRTIME(period_canBus));
    

    // NOT OK HERE NEED TO CLEANUP and correct the inital zero Analog readings %%%%%%%%%%%%%%%%%
    bairClaw.digit[0].FEmotor.readAnalog1();
    rt_task_sleep(rt_timer_ns2ticks(delay)); //
    bairClaw.digit[0].FEmotor.readAnalog2();
    rt_task_sleep(rt_timer_ns2ticks(delay)); //
    bairClaw.digit[0].PIPmotor.readAnalog1();
    rt_task_sleep(rt_timer_ns2ticks(delay)); //
    bairClaw.digit[0].PIPmotor.readAnalog2();
    
    
    // What needs to be read from EPOS? Joint Angles, Tendon Forces
    while (going) {
    	
		rt_task_wait_period(NULL);
		// from Barrett lib. blocking = false
        ret  = CANbus.receiveRaw(id, data, len, false);
        
		if (ret == 0) {  // success read off CANbus not determine what to do with that message.
                // todo: Update Tendon Forces				
				// todo: Update Motor Currents
            if(id == 0x201)		//CAN-id of BC digit set on arduino/CANbusShield. (only joint angle data)
            {
            	// Update Joint Angles
                bairClaw.digit[0].readJoints(data);
                bairClaw.digit[1].readJoints(data);

				
                bcInput[0] = CAN_counter++;
                for(i = 1; i < 5; ++i) {
                    bcInput[i] = bairClaw.digit[0].jointVal[i-1];
                }
                
                bcInput[5] = bairClaw.digit[0].FEmotor.A1;
                bcInput[6] = bairClaw.digit[0].FEmotor.A2;
                bcInput[7] = bairClaw.digit[0].PIPmotor.A1;
                bcInput[8] = bairClaw.digit[0].PIPmotor.A2;
                
                bc_EoSys.setValue(bcInput);
            }else {
                SDO = data[1] + (data[2] << 8); // SERVICE DATA OBJECT (SDO)
                    if (id == bairClaw.digit[0].FEmotor.nodeRec )
                    {
                        currentEPOSController = &bairClaw.digit[0].FEmotor;
                    }
                    else if (id == bairClaw.digit[0].PIPmotor.nodeRec )
                    {
                        currentEPOSController = &bairClaw.digit[0].PIPmotor;
                    }
                    else if (id == bairClaw.digit[0].ADABmotor.nodeRec )
                    {
                        currentEPOSController = &bairClaw.digit[0].ADABmotor;
                    }
                
                    if ( SDO == 0x207c ){
                        if      (data[3] == 0x01)
                        {
                            currentEPOSController->A1 = data[4] + (data[5] << 8);
                        }
                        else if (data[3] == 0x02)
                        {
                            currentEPOSController->A2 = data[4] + (data[5] << 8);
                            A2setCounter++;
                        }
                        bcInput[5] = bairClaw.digit[0].FEmotor.A1;
                        bcInput[6] = bairClaw.digit[0].FEmotor.A2;
                        bcInput[7] = bairClaw.digit[0].PIPmotor.A1;
                        bcInput[8] = bairClaw.digit[0].PIPmotor.A2;
                        
                        bc_EoSys.setValue(bcInput);
                    }
                    else if ( SDO == 0x2030)
                    {
                        if(data[0] == 0x60)
                        {
                            currentEPOSController->currSetCount++;
                        }
                        else if ( data[0] == 0x80)
                        {
                            currentEPOSController->currSetError++;
                        }
                    }
            }

		} else if (ret != 1) {  // error other than no data: time out, socket closed, incomplete, error flag
			printf("ERROR: bus::CANSocket::receive() returned %d.\n", ret);
		}
        
	} // end - while
} // end of canReceiveThread





//RT_CONTROL_LOOP - this loop is used for maintain a desired finger tapping freq.
#pragma mark - rt_control_loop


double T_s_rtControlThreadDelay = 0.01;
void rtControlThread(void *arg){
    
    rt_task_set_periodic(NULL, TM_NOW, secondsToRTIME(T_s_rtControlThreadDelay));
//    std::cout << "cv.wait(lock)" << std::endl;
    boost::unique_lock<boost::mutex> lock(mut);
    double desiredPos = 5, desiredPosPIPDIP = 10, desiredPdc = 1950, pdcError;
    double desiredPdcPIPDIPmaxJointPercentage = 40;
    
    
    double changeInMotorPosFE = 0, changeInMotorPosPD = 0;
    int sw=0, count=0;
    //enum currentControl{positionControl, pdcControl, idle}; DEFINED GLOBALLY
    controlState previousState = idle; //ensures that previousState and currentControl start differently
    //Define biotac filter
    double numBW[] = {   0.002898,   0.008695,   0.008695,   0.002898};
    double denBW[] = {   1.000000,  -2.374095,   1.929356,  -0.532075};
    butterworthFilter btpdcFilter(3, numBW, denBW);
    double btPdcInit = 0;
    if(POSITIONCONTROL)
    {
        bairClaw.digit[0].FEmotor.ActivateProfilePositionMode();
        bairClaw.digit[0].FEmotor.SetMaxFollowingError(MAX_FOLLOWING_ERROR);
        bairClaw.digit[0].FEmotor.enable();
        bairClaw.digit[0].ADABmotor.ActivateProfilePositionMode();
        bairClaw.digit[0].ADABmotor.enable();
        bairClaw.digit[0].PIPmotor.ActivateProfilePositionMode();
        bairClaw.digit[0].PIPmotor.SetMaxFollowingError(MAX_FOLLOWING_ERROR);
        bairClaw.digit[0].PIPmotor.enable();
    }
    
    while( !ready)
    { //waits until main is ready to start running thread. this allows everything to initialize properly.
        cv.wait(lock);
        rt_task_wait_period(NULL);
    }
    btPdcInit = btInput[1];
    while (true)
    {
        rt_task_wait_period(NULL);
         bairClaw.digit[0].calcJointAngles();
         bairClaw.digit[0].calcTendonForce();
        /* //UNCOMMENT TO ADD BACK IN END EFFECTOR FORCE CALCS
         bairClaw.digit[0].calcJacobianActuation();
         bairClaw.digit[0].calcDHparams(); //Will also calc jacobian, pinv(jacobian')
         bairClaw.digit[0].calcEndEffectorForce(); //end effoctor force
         
         btInput[loggerOffset + 7]  = bairClaw.digit[0].DHp.endEffectorForce(0,0);
         btInput[loggerOffset + 8]  = bairClaw.digit[0].DHp.endEffectorForce(1,0);
         btInput[loggerOffset + 9]  = bairClaw.digit[0].DHp.endEffectorForce(2,0);
         btInput[loggerOffset + 10] = bairClaw.digit[0].DHp.endEffectorForce(3,0);
         btInput[loggerOffset + 11] = bairClaw.digit[0].DHp.endEffectorForce(4,0);
         btInput[loggerOffset + 12] = bairClaw.digit[0].DHp.endEffectorForce(5,0);
         */
        btpdcFilter.update(btInput[1]);

        switch(currentControl) //THIS branch will never leave position control
        {
            /**
             * \desc Position control loop to move the hand to a desired location unless contact is made then it will switch modes.
             */
            case positionControl:
                
                if(previousState != currentControl)
                {
                    bairClaw.digit[0].FEmotor.SetCurrentLimit(350);
                    bairClaw.digit[0].FEmotor.SetPositionProfile(250,2500,2500);
                    bairClaw.digit[0].FEmotor.ActivateProfilePositionMode();
                    
                    bairClaw.digit[0].PIPmotor.SetCurrentLimit(350);
                    bairClaw.digit[0].PIPmotor.SetPositionProfile(250,2500,2500);
                    bairClaw.digit[0].PIPmotor.ActivateProfilePositionMode();
                    settingStatus = 0;
                }
                previousState = currentControl;
                
                changeInMotorPosFE = (bairClaw.digit[0].jointPercent[1] - desiredPos) * 50;
                changeInMotorPosPD = ((bairClaw.digit[0].jointPercent[2] + bairClaw.digit[0].jointPercent[3]) - desiredPosPIPDIP) * 50;

                if(count % 2 == 0)
                {
                    bairClaw.digit[0].FEmotor.MoveToPosition(changeInMotorPosFE, 1);
                }
                else
                {
                    bairClaw.digit[0].PIPmotor.MoveToPosition(changeInMotorPosPD, 1);
                }

                if(count % 100 == 0)
                {
                    sw++;
                    if(sw % 2 == 0 ){
                        /* tap
                        desiredPos = 45;
                        desiredPosPIPDIP = 16; //WAS 70
                         */
                        /* stroke */
                        desiredPos = 35;
                        desiredPosPIPDIP = 60; //WAS 70
                    }else{
                        desiredPos = 20;
                        desiredPosPIPDIP = 10; //WAS 10
                    }
                }
                
                /* UNCOMMENT to put state control back in
                if(btInput[1] > 1910)
                {
                    currentControl = pdcControl;
                }
                */
                /*
                if(count % 100 == 0){
                    sw++;
                    if(sw % 2 == 0 ){
                        desiredPos = 17;
                        desiredPosPIPDIP = 12; //WAS 70
                    }else{
                        desiredPos = 0;
                        desiredPosPIPDIP = 10; //WAS 10
                    }
                }*/

                
                break;
            /**
             * \desc pdcControl controlls motor torques to maintain a desired pdc.
             */
            case pdcControl:
               
                
                if(previousState != currentControl)
                {
                    count = 0;
                    bairClaw.digit[0].FEmotor.SetCurrentLimit(900);
                    bairClaw.digit[0].FEmotor.SetPositionProfile(800,2500,2500);
                    bairClaw.digit[0].FEmotor.ActivateProfilePositionMode();
                    
                    bairClaw.digit[0].PIPmotor.SetCurrentLimit(400);
                    bairClaw.digit[0].PIPmotor.SetPositionProfile(800,2500,2500);
                    bairClaw.digit[0].PIPmotor.ActivateProfilePositionMode();
                    settingStatus = 1;
                    
                }
                previousState = currentControl;
                
                if(count < 300)
                {
                    pdcError = desiredPdc - btInput[1];
                    changeInMotorPosFE = -pdcError;
                    
                    if((bairClaw.digit[0].jointPercent[2] + bairClaw.digit[0].jointPercent[3]) < desiredPdcPIPDIPmaxJointPercentage)
                    {
                        changeInMotorPosPD = -pdcError/10;
                    }
                    else
                    {
                        changeInMotorPosPD = 0;
                    }
                    bairClaw.digit[0].FEmotor.MoveToPosition (changeInMotorPosFE, 1);
                    bairClaw.digit[0].PIPmotor.MoveToPosition(changeInMotorPosPD, 1);
                }
                else if (count < 400){
                    if(settingStatus < 2)
                    {
                        desiredPos = 1;
                        desiredPosPIPDIP = 1;
                        bairClaw.digit[0].FEmotor.SetPositionProfile(800,2500,2500);
                        bairClaw.digit[0].PIPmotor.SetPositionProfile(800,2500,2500);
                    }
                    settingStatus = 3;
                    changeInMotorPosFE = (bairClaw.digit[0].jointPercent[1] - desiredPos) * 50;
                    changeInMotorPosPD = ((bairClaw.digit[0].jointPercent[2] + bairClaw.digit[0].jointPercent[3]) - desiredPosPIPDIP) * 50;
                    
                    bairClaw.digit[0].FEmotor.MoveToPosition(changeInMotorPosFE, 1);
                    bairClaw.digit[0].PIPmotor.MoveToPosition(changeInMotorPosPD, 1);
                    
                }
                else
                {
                    currentControl = positionControl;
                    settingStatus = 1;
                }
                
                

                break;
            //-------------------------------------------------------------------------------//
            case idle:
                previousState = currentControl;
                break;
        }
        count++;
//        btInput[loggerOffset + 7]  = currentControl;
//        btInput[loggerOffset + 8]  = settingStatus;
    }
    

    
}




/*
 |--------------------------------------------------------------------------
 | MAIN Outline:
 read arguments
 create tmpfile
 setup visualizer host
 load playbackdata
 initialize BT (create bt_record_thread)
    check if digits running fit BT loaded
 CAN initialize (create can_receive_thread)
    check if digits running fit CAN passed
>>if MISS LOADED, call dynamic destructor and TERMINATE
 create RealTimeEM
 create ramp time
 establish BCProductManagers
 tuplegrouper/periodicDataLogger
 connect all systems io
 create display thread
 create visualizer thread
 tendon friction adjustment
 >>start EM
 start display
 connect tuplegrouper to periodiclogger
 start controller
 terminate controller (into free mode)
 stop display
 stop visualizer
 stop EM
 remove tasks: rt/bt/can
 close logger
 create reader
 export tmp to reader
 delete tmp
*/
#pragma mark - mainEntryPoint
int main(int argc, char** argv) {

// Pre-run check up
	//// Inform entering pre-run
	printf("================================================================================\n");
	printf("  Entering BairClaw Pre-run\n");
	printf("================================================================================\n");
	
    mlockall(MCL_CURRENT|MCL_FUTURE);/* Avoids memory swapping for this program */
    
    /*//------------------ FHN MERGER SUCCESS?
    DHparams DHp;
    // Jacobian and Matrix test---------------------------------------------------------
    DHp.calcT();
    int runNum = 1000000;
    VectorXd thetaUpdate(4);
    clock_t begin = clock();
    for(int i=0; i < runNum; i++)
    {
        thetaUpdate << 0, 1, 0, 1;
        DHp.calcT(thetaUpdate);
        if( i%10000 == 0)
            std::cout << (double(i)/double(runNum))*100 << "%" << std::endl;
        DHp.calcJacobian();
        DHp.pinvJacobian();
        
    }
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    
    std::cout << "elapsed_secs = " << elapsed_secs << std::endl;
    std::cout << "Hz = " << double(runNum)/elapsed_secs << std::endl;
    
    std::cout << "Jacobian" << std::endl << std::endl;
    std::cout << DHp.jacobian << std::endl;
    std::cout << std::endl << "pinv(Jacobian) " << std::endl;
    std::cout << DHp.jacobianPseudoInverse << std::endl;
    //-----------------------------------------------------------------------------------*/
    
    // make temp logging file info
    char tmpFile[] = "/tmp/btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR %d : Couldn't create temporary file!\n", errno); return -1; }
	
	
	
	
	
    // Load in binary playbackData created by other devices (ex: MATLAB)
    playbackData->readFile("data2Track.bin");
    
    
    
    
    
    // MATLAB Visualizer
    if (argc >= 2) {
        host = argv[1];
        printf("> Visualizer data being sent to ip: %s \n", argv[1]);
    } else {
        host = "127.0.0.1"; // Local host (THIS computer)
    }
    




	/* Create execution manager. Realtime operation update period = 0.005 sec
		RealTimeExecutionManager::RealTimeExecutionManager(double period_s, int rt_priority) :
			ExecutionManager(period_s), thread(), priority(rt_priority), running(false), error(false), errorStr(), errorCallback()
			{	init();   } 
		[barrett/systems.h --> barrett/systems/real_time_execution_manager.h]
	*/
	systems::RealTimeExecutionManager mem(0.005, 50);
    
    
    
    
    
    // Creates and initializes BairClaw Product Manager (inheritting Systems)                        %%%%%%%%%%%%%%%%%%%%%
    BCProductManager bcSys;
    
    
    
    
    
	/* Create object Time, which updates every execution cycle. Time stays current even if isn't used for a time
		Ramp (ExecutionManager* em, double slope, const std::string& sysName = "Ramp") : gain(slope), System(sysName), ...
		[barrett/systems.h --> barrett/systems/ramp.h]
	*/
    systems::Ramp time(&mem, 1.0);
    
    
    
    
    
    /* Create object tg, which tuples a set of data (10 maximum appended types)
    	[barrett/systems.h --> barrett/systems/tuple_grouper.h]
    */
    systems::TupleGrouper <double, int, math::Vector< BC_NUM_VAR_RECORD >::type, math::Vector<BT_NUM_VAR_RECORD>::type> tg;
    
	/* Create a realtime logger while logs to tmpfile the same rate as the execution manager operation (0.003s < period < 1s)
		PeriodicDataLogger (ExecutionManager* em, LogWriterType* logWriter, size_t periodMultiplier = 10, const std::string& sysName = "PeriodicDataLogger");
		[barrett/systems.h --> barrett/systems/periodic_data_logger.h]
	*/
	typedef boost::tuple<double, int, math::Vector< BC_NUM_VAR_RECORD >::type, math::Vector<BT_NUM_VAR_RECORD>::type> tuple_type;
	const size_t PERIOD_MULTIPLIER = 1;	// log every period
	systems::PeriodicDataLogger<tuple_type> logger(&mem, new log::RealTimeWriter<tuple_type>(tmpFile, PERIOD_MULTIPLIER * mem.getPeriod()), PERIOD_MULTIPLIER);
	
	
	
	
    
	// Make connections between Systems
	//// Feed joint info into BairClaw system object, (passed to jointsOutput?)
    connect(bc_EoSys.output, bcSys.input);
	//// All system outputs transmitted to tuple grouper
	connect(time.output,       		tg.getInput<0>());  //  1 double
    connect(bcSys.output,      		tg.getInput<1>());  //  1 int
    connect(bcSys.jointsOutput, 	tg.getInput<2>());  // 11 double
    connect(bt_EoSys.output,		tg.getInput<3>());  // 45 double
    //// Print Bairclaw's output
//	connect(bcSys.output, printSys.input);
    //// Realtime ExecutionManager Check
	printf("> Status of RealTimeExecutionManager : ");
	if (mem.isRunning())	printf("On\n"); else 	printf("Off\n");
	
	
	
	
	RT_TASK bt_record_rt_task;
	RT_TASK can_receive_rt_task;
	RT_TASK rt_control_rt_task; //This is going to directly send CAN communcations to set MoveToPosition
	
	/* Creates hard realtime Xenomai threads (called RT_TASKs)
    	int rt_task_create (RT_TASK* task, const char* name, int stksize, int prio, int	mode)	
			name	symbolic name of task. 
			stksize	The size of the stack (in bytes) for the new task. If zero is passed, a reasonable pre-defined size will be substituted.
			prio	The base priority of the new task. This value must range from [0 .. 99] (inclusive) where 0 is the lowest effective priority.
			mode	The task creation mode. The following flags can be OR'ed into this bitmask.
	  	  return 0 is upon success
		[Xenomai - task.h]
	*/
    if (rt_task_create(&bt_record_rt_task, "btRecord", 0, 50, 0)) {
		printf("ERROR %d : Couldn't create bt record thread!\n", errno); return 0; }
	if (rt_task_create(&can_receive_rt_task, "canReceive", 0, 51, 0)) {
		printf("ERROR %d : Couldn't create CAN receive thread!\n", errno); return 0; }
	if (rt_task_create(&rt_control_rt_task, "rtController",0, 52,0)) {
		printf("ERROR %d : Couldn't create RT control thread!\n", errno); return 0; }
    




    /* Initiate and starts Xenomai real-time tasks												%%%%%%%%%%%%%%%%%%%%%%%%%
		int rt_task_start (RT_TASK* task, void(*)(void *cookie) entry, void* cookie)	
			task	The descriptor address of the affected task which must have been previously created by the rt_task_create() service.
			entry	The address of the task's body routine. In other words, it is the task entry point.
			cookie	A user-defined opaque cookie the real-time kernel will pass to the emerging task as the sole argument of its entry point.
					(In other words, the tasks' default input)
	  	  return 0 is upon success
		[Xenomai - task.h]
	*/
    if (rt_task_start(&bt_record_rt_task, &biotacRecordThread, NULL)) {
    	printf("ERROR %d : Couldn't start bt record thread!\n", errno); return 0; }
    if (rt_task_start(&can_receive_rt_task, &canReceiveThread, NULL)) {
    	printf("ERROR %d : Couldn't start CAN receive thread!\n", errno); return 0; }
    if (rt_task_start(&rt_control_rt_task, &rtControlThread, NULL)) {
    	printf("ERROR %d : Couldn't start RT control thread!\n", errno); return 0; }
	
	
	
	
	
	
	
    //// When BioTac is On, loop until BT initiation
    if(!biotacOff) {
        while(!biotacInit) {
            usleep(100000);
        }
    }
    
    
    
    
    
    // Push data into bcSys' input and run an execution cycle, %%%
	// causing bcSys::operate() to be called %%%
    printf("Press [Enter] to continue ... sets shouldStart = TRUE");
	fflush(stdout);
	getchar();
    
    
    if (!DEBUG){
        bairClaw.digit[0].FEmotor.SetCurrentLimit(MAXCURRENTALLOWED);
        bairClaw.digit[0].FEmotor.enable();
        bairClaw.digit[0].FEmotor.ActivateCurrentMode(5000, MAXCURRENTALLOWED, 25000);
        bairClaw.digit[0].ADABmotor.SetCurrentLimit(MAXCURRENTALLOWED);
        bairClaw.digit[0].ADABmotor.ActivateCurrentMode(5000, MAXCURRENTALLOWED, 25000);
        bairClaw.digit[0].ADABmotor.enable();
    }else{

    }
 
    // Start loadCellRecordThread
//    boost::thread t(loadcellRecordThread);
	boost::thread BCMatlabVis(bairClawMatlabVisualizerSender), disp(displayThread);
 
//    bc_EoSys.setValue(bcInput);
    shouldStart = true;
    
    
    
    //bairClaw.digit[0].setStaticFriction();
    printf(" Static setFriction complete MCP-F - %d MCP-E - %d \n Press [Enter] to start logging", bairClaw.digit[0].FEmotor.staticFrictionF, bairClaw.digit[0].FEmotor.staticFrictionE);
	fflush(stdout);
	getchar();
    
   


    
    mem.startManaging(bcSys);
    mem.start();
    
    time.start();
    
    // setTendonOffsets -------------------------------------------------
    usleep(500000);
    bairClaw.digit[0].setTendonForceOffset();
    
    
    printf("Press [Enter] to START display");
    fflush(stdout);
    getchar();
    
    displayOn = true; // has display thread clear and print to screen
    connect(tg.output, logger.input);

	printf("Logging started.\n");
    
    
    ready = true;
    cv.notify_all();

    
    printf("Press [Enter] to STOP logging");
	fflush(stdout);
	getchar();

   
    bairClaw.digit[0].FEmotor.resetAll();
    
    
    printf("Press [Enter] to ShutDown");
    fflush(stdout);
    getchar();
    
    going = false;

    
    
    disp.join(); //Stop display thread
    BCMatlabVis.join();
//    t.interrupt(); //Use interrupt instead of join() becasue recvfrom is blocking and if we lose network won't exit.
    std::cout << " About to remove tasks" << std::endl;

    delete playbackData;
    rt_task_delete(&bt_record_rt_task);
    std::cout << " bt_record_returned" << std::endl;
    rt_task_delete(&can_receive_rt_task);
    std::cout << " can_receive_returned" << std::endl;
    rt_task_delete(&rt_control_rt_task);
    std::cout << " rt_control_returned" << std::endl;
 

    
    
//    std::cout << " t joined" << std::endl;
    mem.stop();
    std::cout << "is running - ";
    std::cout << mem.isRunning() << std::endl;

    
    
    
    logger.closeLog();
    
    
	printf("Logging stopped.\n");
    
	log::Reader<tuple_type> lr(tmpFile);
    //Add file time stamp
    std::string fileName = "output_";
    time_t nowR;
    char timeNow[50];
    timeNow[0] ='\0';
    nowR = std::time(NULL);
    if(nowR != -1)
    {
        strftime(timeNow, 50, "%m-%d-%Y_%H-%M", localtime(&nowR));
        fileName.append(std::string(timeNow));
        fileName.append(".csv");
    }
    
	lr.exportCSV(fileName.c_str());
	printf("Output written to %s.\n", fileName.c_str());
	std::remove(tmpFile);
    
    std::cout << "About to print Period - ";
    
    std::cout << mem.getPeriod() << " count - " <<  count << std::endl;

	
    
	return 0;
}





















/* CODE CEMENTARY : Thread for loadcell, when LABVIEW was still used (pre-2014) */
//struct lCell
//{
//    double x,   y,  z;
//    double tx, ty, tz;
//    int numOfMsgReceived;
//}loadCell;
//void loadcellRecordThread()
//{
//    const int PORT = 12345;
//    const int BUFLEN = 1024;
//    
//    struct sockaddr_in my_addr, cli_addr;
//    int sockfd, i;
//    socklen_t slen=sizeof(cli_addr);
//    char buf[BUFLEN];
//    
//    if ((sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
//        err("socket");
//    else
//        printf("Server : Socket() successful\n");
//    
//    bzero(&my_addr, sizeof(my_addr));
//    my_addr.sin_family = AF_INET;
//    my_addr.sin_port = htons(PORT);
//    my_addr.sin_addr.s_addr = htonl(INADDR_ANY);
//    
//    if (bind(sockfd, (struct sockaddr* ) &my_addr, sizeof(my_addr))==-1)
//        err("bind");
//    else
//        printf("Server : bind() successful\n");
//    
//    loadCell.numOfMsgRevieved  = 0;
//
//    
//    while (going)
//    {
//        if (recvfrom(sockfd, buf, BUFLEN, 0, (struct sockaddr*)&cli_addr, &slen)==-1)
//            err("recvfrom()");
//        
//        sscanf(buf,"%lf %lf %lf %lf %lf %lf", &(loadCell.x), &(loadCell.y), &(loadCell.z), &(loadCell.tx), &(loadCell.ty), &(loadCell.tz) );
////        printf("Received packet from %s:%d\nData: %s",
////               inet_ntoa(cli_addr.sin_addr), ntohs(cli_addr.sin_port), buf);
////        printf("counter = %d\n",loadCell.numOfMsgRevieved++);
////
////        printf("loadCell Values [ %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f]\n\n", loadCell.x, loadCell.y, loadCell.z, loadCell.tx, loadCell.ty, loadCell.tz);
//        
//        
//        btInput[loggerOffset]     = tensionDesired;
//        btInput[loggerOffset + 1] = loadCell.x;
//        btInput[loggerOffset + 2] = loadCell.y;
//        btInput[loggerOffset + 3] = loadCell.z;
//        btInput[loggerOffset + 4] = loadCell.tx;
//        btInput[loggerOffset + 5] = loadCell.ty;
//        btInput[loggerOffset + 6] = loadCell.tz;
//        
//    }
//    
//    close(sockfd);
//    
//}
