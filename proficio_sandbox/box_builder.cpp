/**
 * @file box_builder.cpp
 * @author j.hagstrand
 * @date 7/15/14
 */

/**
 * Game Description:
 *
 * Game is intended to map range of motion of patient and store work space accessibility.
 * 1) Maximum Work range is determined by Proficio Arm
 * 2) Work Space accessibility will be determined from record point (button)
 * 3) Advanced feature (amount of force required to get to different planes can be adjusted on screen.
 * 4) visualization appears on screen while walls are being pushed out. 
 */

/* Standard Modules */
#include <cstdio>
#include <cassert>
#include <curses.h>
/* Barrett Library Modules */
#include <barrett/log.h>
#include <barrett/math.h>
#include <barrett/exception.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
//#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h>

using namespace barrett;

//BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;


/** Haptic Calculation Method
 *
 *
 */

class HapticForces: public systems::SingleIO<typename units::CartesianPosition::type, typename units::CartesianForce::type>{
  BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;
protected:

	static const double wallInc = 0.05; // distance at which walls increase
	static const double Stiffness = 0.0; //1200.0;
	static const double Dampness = 0.0; //10.0;
	
public:
	explicit HapticForces(const std::string& sysName = "HapticForces") : 
	systems::SingleIO<cp_type,cf_type>(sysName), wallLevel(), origin(),
	wallPosition(),maxPosReached(),cfSum(){}
	virtual ~HapticForces(){ this->mandatoryCleanUp(); }
	
	void reset();
	void print();
	
protected:
	cp_type curPos;
	cf_type result;	
	int wallLevel[6];
	double origin[6], wallPosition[6], maxPosReached[6];
	double cfSum[3];
	void updateWalls( void );
	
	virtual void operate(){
		curPos = this->input.getValue();
		
		for(int i = 0; i < 6; i++){
	  
		  if(i == 0 || i == 2 || i == 4){  // Calculations for Negative Cartesian Space Coordinates -XYZ
	 
		    // have we reached farther than before?
		    (curPos[i] < maxPosReached[i]) ? maxPosReached[i] = curPos[i] : maxPosReached[i] = maxPosReached[i];
		    // are we close to the wall position
		    if(abs(wallPosition[i] - curPos[i]) < 0.01){
		      // apply force in appropriate axis
		      if(i == 0){cfSum[0] += Dampness * (0.01 - abs(wallPosition[i] - curPos[i]));}
		      else if(i == 2){cfSum[1] += Dampness * (0.01 - abs(wallPosition[i] - curPos[i]));}
		      else if(i == 4){cfSum[2] += Dampness * (0.01 - abs(wallPosition[i] - curPos[i]));}
		    }
		    // Do Wall Positions need to be updated?
		    if(curPos[i] < wallPosition[i]){
			    //cout << "For I= " << i << " Test CurPos = " << curPos[i] << ", Wall Position: " << wallPosition[i] << endl;
			    wallLevel[i]++;
			    //cout << "New wall position Update: " << i << endl;
		    }
		  // End NEG Calcs
		  }else{// Calculations for Positive Cartesian Space Coordinates +XYZ
	  
		    // Determine Reach
		    (curPos[i] > maxPosReached[i]) ? maxPosReached[i] = curPos[i] : maxPosReached[i] = maxPosReached[i];
		    // are we close to the wall position
		    if(abs(wallPosition[i] - curPos[i]) < 0.01){
		      // apply force in appropriate axis
		      if(i == 1){cfSum[0] -= Dampness * (0.01 - abs(wallPosition[i] - curPos[i]));}
		      else if(i == 3){cfSum[1] -= Dampness * (0.01 - abs(wallPosition[i] - curPos[i]));}
		      else if(i == 5){cfSum[2] -= Dampness * (0.01 - abs(wallPosition[i] - curPos[i]));}
		    }
		    // Do Wall Positions need to be updated?
		    if(curPos[i] > wallPosition[i]){
			    //cout << "For I= " << i << " Test CurPos = " << curPos[i] << ", Wall Position: " << wallPosition[i] << endl;
			    wallLevel[i]++;
			    //cout << "New wall position Update: " << i <<  endl;
		    }
		  }// End Pos Cart Calcs
		}
		
		result << cfSum[0], cfSum[1], cfSum[2];
		this->outputValue->setData(&result);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(HapticForces);
};

/** Reset Method should accomplish following:
 * 1) Reset Wall level to 1 -> which should change wall positions also
 * 2) Set Origin to current Proficio CP
 * 3) Reset Max Cartesian Positions Reached
 */
void HapticForces::reset(void){
	for(int i=0;i<6;i++){
		wallLevel[i] = 1;
		if(i < 2){
		  origin[i] = curPos[0];
		  maxPosReached[i] = curPos[0];
		}else if(i < 4){
		  origin[i] = curPos[1];
		  maxPosReached[i] = curPos[1];
		}else if(i < 6){
		  origin[i] = curPos[2];
		  maxPosReached[i] = curPos[2];
		}
	}
	updateWalls();
}
/**
 * Update wall positions to most current wall Levels away from Origin.
 */
void HapticForces::updateWalls(void){
	for(int i = 0; i<6; i++){
		if(i == 0 || i == 2 || i == 4){
		  // decrease values for minimums
		  wallPosition[i] = wallLevel[i] * -1 * wallInc + origin[i];
		}else{
		  // increase values for maximums
		  wallPosition[i] = wallLevel[i] * wallInc + origin[i];
		}
	}
}
/** Method to Show Current Statistics
 */
void HapticForces::print(void){
	
	std::cout << "Wall Levels: ";
	for(int i=0;i<6;i++){
		std::cout << wallLevel[i] << ", ";
	}
	std::cout << std::endl;
	std::cout << "Wall Positions: ";
	for(int i=0;i<6;i++){
		std::cout << wallPosition[i] << ", ";
	}
	std::cout << std::endl;
	std::cout << "Origin Positions: ";
	for(int i=0;i<6;i++){
		std::cout << origin[i] << ", ";
	}
	std::cout << std::endl;
	std::cout << "Maximum Reach: ";
	for(int i=0;i<6;i++){
		std::cout << maxPosReached[i] << ", ";
	}
	std::cout << std::endl;
}

/** 
 *  Current Print Menu Options
 */
void printMenu( void ){
    std::cout << "\tD Turn on/off Forces\n" << std::endl;
    std::cout << "\tR Reset WorkSpace Values\n" << std::endl;
    std::cout << "\tQ or X to Quit\n" << std::endl;
}


/** Main Body
 *
 *  Main body will record full session data. 
 */
template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {

	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  //Right ARM
  jp_type startPoint;
  startPoint << 0.0, 0.0, 1.57;

	// Define/Instantiate systems 
	//HapticForces<cp_type,cf_type> hf;
	HapticForces hf;
	systems::ToolForceToJointTorques<DOF> tf2jt;
	
	// Make Major System Connections
	connect(wam.toolPosition.output, hf.input);
	connect(hf.output, tf2jt.input);
	connect(wam.kinematicsBase.kinOutput, tf2jt.kinInput);
	connect(tf2jt.output, wam.input);
	
	
	// Network haptic stuff
	//connect(wam.toolPosition.output, tg.<0>);
	//connect(wam.toolVelocity.output, tg.<1>);
	//connect(patientReach.output, tg.input<2>);
	
	wam.gravityCompensate();
	// Move to starting Point and Idle for Free Movement   
	wam.moveTo(startPoint); 
	wam.idle();
	
	// Initialize Everything
	hf.reset();
  
	// Get Loop running using flags
	bool running=true; 
	std::string line; 
	std::string str;
	printMenu();	
	while (running) {
		printf(">>> ");
		std::getline(std::cin, line);

		switch(line[0]){
			case 'd':
				std::cout << "Showing Statistics: " << std::endl;
				hf.print();
				break;
			case 'r':
				std::cout << "Reseting all Data" << std::endl;	
				hf.reset();
				break;
				
			case 'q':
			case 'Q':
			case 'x':
			case 'X':       
				std::cout << "Quiting...Moving WAM Home." << std::endl;
				running = false;
				break;
			default:
			  printMenu();
			  break;
		}

	}
	

	/* Move Home and then Wait for Shift-Idle of Wam */  
	wam.moveHome();
	wam.idle();
	printf("\n\n");
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
