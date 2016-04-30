#include "QB_lib/commands.h"
#include "QB_lib/qbmove_communications.h"
#include "iostream"
#include <math.h>
#include <fstream>
#include <signal.h>

comm_settings comm_settings_t ;
short int values[2] = {0,0};

short int imu_values[6] = {0,0,0,0,0,0};
double imu_norm = 0.0;

#define HAND_CLOSED 15000

using namespace std;

void close_fcn(int arg){
	values[0] = values[1] = 0;
	commSetInputs(&comm_settings_t, 0, values);
	Sleep( (DWORD) 500 );
	
	commActivate(&comm_settings_t, 0, 0);
	Sleep( (DWORD) 500 );
	
	cout << "Closing program" << endl;
	
	exit(0);
}

int main(){

	short int last_accel[3] = {0,0,0};
	double last_norm = 0;
	int cont = 0;

	signal(SIGINT, close_fcn);
	
    openRS485 ( &comm_settings_t , "COM2" ) ;       // QB Board Opening

//  RS485GetInfo (&comm_settings_t, buffer);
//  cout << buffer << endl;

    Sleep( (DWORD) 250 );

	commActivate(&comm_settings_t, 0, 1);
	Sleep( (DWORD) 500 );
			
	char package_in[50];		// output data buffer
    int package_in_size;
	int hand_movement = 0;
	
	commBumpDetectionActivate(&comm_settings_t, 0, 0);
	Sleep(500);
	
	
	while(1){
		
/******************************** Movement: 1st stage ********************************/
		
		// Search for starting movement
		cout << "Waiting for Movement 1" << endl << endl;
		
		
		// Perform a first reading
		commGetIMUMeasurements(&comm_settings_t, 0, imu_values);
		last_accel[0] = imu_values[0];
		last_accel[1] = imu_values[1];
		last_accel[2] = imu_values[2];
		Sleep(300);
		
		cont = 0;		
		while(!cont){
			
			commGetIMUMeasurements(&comm_settings_t, 0, imu_values);
	
			if (abs(imu_values[0]-last_accel[0]) < 300 && abs(imu_values[1]-last_accel[1]) < 300 && abs(imu_values[2]-last_accel[2]) > 300) {
				cout << "diff 0: " << abs(imu_values[0] - last_accel[0]) << endl;
					cout << "diff 1: " << abs(imu_values[1] - last_accel[1]) << endl;
					cout << "diff 2: " << abs(imu_values[2] - last_accel[2]) << endl;
				
				cont = 1;
			}
			
			last_accel[0] = imu_values[0];
			last_accel[1] = imu_values[1];
			last_accel[2] = imu_values[2];
			
			Sleep(100);
		}
	
	
/******************************** Movement: 2nd stage ********************************/
		cout << "Waiting for Movement 2" << endl << endl;
		
		cont = 0;	
		while(!cont){
			
			commGetIMUMeasurements(&comm_settings_t, 0, imu_values);
	
			if (abs(imu_values[0]-last_accel[0]) < 300 && abs(imu_values[1]-last_accel[1]) < 300 && abs(imu_values[2]-last_accel[2]) > 300) {
				cout << "diff 0: " << abs(imu_values[0] - last_accel[0]) << endl;
					cout << "diff 1: " << abs(imu_values[1] - last_accel[1]) << endl;
					cout << "diff 2: " << abs(imu_values[2] - last_accel[2]) << endl;
				
				cont = 1;
			}
			
			last_accel[0] = imu_values[0];
			last_accel[1] = imu_values[1];
			last_accel[2] = imu_values[2];
			
			Sleep(100);
		}
	
/********************** Movement: 3rd stage: closure + rotation **********************/
		cout << "Waiting for Movement 3" << endl << endl;
		
		values[0] = HAND_CLOSED;
		values[1] = 0;   // motor 2		
		commSetInputs(&comm_settings_t, 0, values);
		Sleep( (DWORD) 500 );
	
		cont = 0;
		while(!cont) {
			
			commGetIMUMeasurements(&comm_settings_t, 0, imu_values);
		
			// Compute bump reading IMU and move SoftHand accordingly
			imu_norm = sqrt(imu_values[0]*imu_values[0] + imu_values[1]*imu_values[1] + imu_values[2]*imu_values[2]);
			//cout << "Norm: " << imu_norm << endl;
			
			//!cont is useful to avoid difference from last bump could be seen as a new bump
			if (!cont && fabs(imu_norm - last_norm) > 250 && fabs(imu_norm - last_norm) < 500
					&& abs(imu_values[0]-last_accel[0]) > 100 && abs(imu_values[1]-last_accel[1]) < 300 && abs(imu_values[2]-last_accel[2]) < 300){
					cout << "Bump detected" << endl;
					
					cout << "diff 0: " << abs(imu_values[0] - last_accel[0]) << endl;
					cout << "diff 1: " << abs(imu_values[1] - last_accel[1]) << endl;
					cout << "diff 2: " << abs(imu_values[2] - last_accel[2]) << endl;
					cout << "diff norm: " << fabs(imu_norm - last_norm) << endl << endl;
					cout << "gyro: " << imu_values[3] << '\t' << imu_values[4] << '\t' << imu_values[5] << endl;
					cont = 1;
			}
			else {
				//cout << "No bump" << endl;
				cont = 0;
			}
				
			/*	cout << "imu: " << imu_values[0] << " diff: " << abs(imu_values[0] - last_accel[0]) << endl;
				cout << "imu: " << imu_values[1] << " diff: " << abs(imu_values[1] - last_accel[1]) << endl;
				cout << "imu: " << imu_values[2] << " diff: " << abs(imu_values[2] - last_accel[2]) << endl;
				cout << "diff norm: " << fabs(imu_norm - last_norm) << endl << endl;
			*/
			
			last_accel[0] = imu_values[0];
			last_accel[1] = imu_values[1];
			last_accel[2] = imu_values[2];
			last_norm = imu_norm;
			
			cout << endl;
			Sleep(100);
		}
		
/************************* Movement: 4th stage: hand opening *************************/		
		if (cont){
			
			// Movement implemented into code
			hand_movement = 1;
			values[0] = HAND_CLOSED;
			
			//Movement: 4th stage: open hand
			// Open SoftHand, while moving fingers up and down
			while (hand_movement) {
				
				if (values[0] < 1000){   // The hand has completely opened
					
					// Re-close hand and finishes primitive
					//values[0] = HAND_CLOSED;
					
					//Sleep(1000);  // Wait for hand closure
					
					hand_movement = 0;
				}
				else {
					
					// Set hand reference position
					values[0] -= 1000;                // motor 1
					
				}
			
				values[1] = 0;   // motor 2
				
				commSetInputs(&comm_settings_t, 0, values);
				//cout << values[0] << " " << values[1] << endl;
				Sleep(200);
			}
				
			//comm_Start_BAH_A_LA_LA_LA_CODE(&comm_settings_t, 0);		// Movement implemented into code

			cout << "FINISHED" << endl;
			
			cout << "Wait..." << endl;
			Sleep( (DWORD) 2000 );
			cout << "OK!" << endl << endl << endl;	
		}
		
		cont = 0;
		
	}
   
    return 0;
}
