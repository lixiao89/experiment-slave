/*-*- Mode++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-   */
/*ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab:*/

/*
  $Id: mtsUserStudySlave.h 3181 2011-11-15 15:41:28Z sleonar7 $

  Author(s):  Simon Leonard
  Created on: 2013-07-22

  (C) Copyright 2012 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <fstream>
#include <cisstCommon/cmnPath.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstOSAbstraction/osaGetTime.h>

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

#include <cisstVector.h>
#include <cisstParameterTypes.h>

#include <sawJR3ForceSensor/osaJR3ForceSensor.h>
#include <sawControllers/osaGravityCompensation.h>
#include "osaHybridForcePosition.h"

#include <cisstRobot/robLinearSE3.h>

#include <cisstNumerical/nmrLSMinNorm.h>
#include <cisstNumerical/nmrSavitzkyGolay.h>

#include "RLSestimator.h"

class mtsUserStudySlave : public mtsTaskPeriodic {

private:

  // robot stuff
  robManipulator robot;         // robot kinematics
  robManipulator tool;          // tool kinematics
  robLinearSE3* se3traj;        // interpolation 
  std::list<double> dt;
  double timer;

  RLSestimator* rls;
  std::ofstream ofsForceData;

  bool failstate;
  double startTime;

  vctVec    qsold;
  vctFrm4x4 Rtwtsold; // previous command to the slave
  vctRot3   Rts;      // rotation of sensor wrt tool

  //vctMatrixRotation3<double> Rts;
  
  // JR3 sensor
  osaJR3ForceSensor jr3;

  // the controllers
  osaHybridForcePosition hfp;
  
  // interfaces
  mtsInterfaceProvided* mastertel;  // telemetry (slave->master)
  mtsInterfaceProvided* mastercmd;  // commands  (master->slave)
  mtsInterfaceRequired* slave;      // interface to the slave PIDs
  mtsInterfaceRequired* control;    // control interface

  enum State{ IDLE, ENABLE };
  State state;
  bool IsEnabled() const { return state == ENABLE; }


  // Slave side
  //! Read the joint positions
  mtsFunctionRead  mtsGetPosition;
  //! Write the joint positions
  mtsFunctionWrite mtsSetPosition;

  // Master side 
  prmPositionCartesianGet prmCommandSE3;    // Cartesian command (master side)
  prmForceCartesianGet    prmCommandWrench; // Wrench command (master side)
  prmPositionCartesianGet prmTelemetrySE3;  // Cartesian telemetry
  prmPositionJointGet     prmTelemetryRn;   // Joint telemetry


  //! Get the position command increment from the master
  bool GetCommand( vctFrm4x4& Rtwts );

  //! Get the wrench command from the master
  bool GetCommand( osaJR3ForceSensor::Wrench& ws );

  //! Get the measured joint angles
  bool GetPosition( vctVec& q );

  //! Get the measured cartesian position
  bool GetPosition( vctFrm4x4& Rtwt );

  //! Get the measured wrench
  bool GetWrench( osaJR3ForceSensor::Wrench& w );

  //! Get the position command increment from the master
  void GetCommand();

  //! Set the telemetry 
  void SetTelemetry();

  void IdleMotion();
  void HybridMotion();
  
  vctDynamicVector<double> sg;
  std::list< osaJR3ForceSensor::Wrench > stdft;    

 public:
  int logcnt;
  double logtime;

  void PrintTime(){
      std::cout<< "current time is: "<< osaGetTime() - startTime <<std::endl;
  }


  void Toggle(){ 
    if( state == ENABLE ) {
      ofs.close();
      state = IDLE; 
      ofs2.open( "log.txt" );
    }
    else{
      char fname[32];
      sprintf( fname, "log%d.txt", logcnt++ );
      logtime=osaGetTime();
      ofs.open( fname );
      ofs2.close();
      state = ENABLE; 
    }
  }

 public:
  std::ofstream ofs;
  std::ofstream ofs2;
  //!
  /**
     @param name The component name
     @param period The component period (0.01s)
     @param robotfilename The kinematics file of the robot
     @param Rtw0 Orientation/position of the base wrt world
     @param Rtnt Orientation/position of tool wrt robot last link
     @param qinit Initial joints position of the robot robot
     @param qready Ready joints position of the robot
     @param jr3 JR3 F/T sensor
     @param gc Gravity compensation controller
     @param hfp Hybrid FT/Position controller
   */
  mtsUserStudySlave(const std::string& name,
		    double period,
                    
		    const std::string& robotfilename, 
		    const vctFrm4x4& Rtw0 = 
		    vctFrm4x4( vctRot3(  0.0,  0.0, -1.0,
					 0.0,  1.0,  0.0,
					 1.0,  0.0,  0.0,
					 VCT_NORMALIZE ),
			       vct3( 0.0 ) ),
		    
		    const vctFrm4x4& Rtnt =
		    vctFrm4x4( vctRot3(  0.9511,  0.0000,  -0.3090,
					 0.0000,  1.0000,   0.0000,
					 0.3090,  0.0000,   0.9511,
					 VCT_NORMALIZE ),
			       vct3( 0.0, 0.0, 0.15 ) ),
		    
		    const vctVec& qinit = 
		    vctVec(7.0, 0.0, -cmnPI_2, 0.0, cmnPI, 0.0, -cmnPI_2, 0.0));
    
  void Configure( const std::string& );
  void Startup();
  void Run();
  
};

