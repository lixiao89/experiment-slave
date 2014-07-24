/*-*- Mode++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-   */
/*ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab:*/

/*
  $Id: mtsHybridForcePosition.h 3181 2011-11-15 15:41:28Z sleonar7 $

  Author(s):  Simon Leonard
  Created on: 2013-07-22

  (C) Copyright 2012 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstCommon/cmnPath.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstOSAbstraction/osaGetTime.h>

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmEventButton.h>

#include <sawJR3ForceSensor/osaJR3ForceSensor.h>
#include <sawControllers/osaGravityCompensation.h>
#include "osaHybridForcePosition.h"

#include <cisstRobot/robLinearSE3.h>

#include <cisstNumerical/nmrLSMinNorm.h>
#include <cisstNumerical/nmrSavitzkyGolay.h>

class mtsHybridForcePosition : public mtsTaskPeriodic {

private:

  // robot stuff
  robManipulator robot;  // kin
  robManipulator tool;
  robLinearSE3* traj;
  double timer;
  vctFrame4x4<double> Rtwtsold;    // previous command to the slave
  vctFrame4x4<double> Rtwtsoldtrj; // previous interpolated from the master
  vctFrame4x4<double> Rtwtsoldcmd; // previous command from the master

  vctMatrixRotation3<double> Rts;
  
  // JR3 sensor
  osaJR3ForceSensor* jr3;

  //
  osaGravityCompensation* gc;
  osaHybridForcePosition* hfp;
  
  // interfaces
  mtsInterfaceProvided* mastertel;
  mtsInterfaceProvided* mastercmd;
  mtsInterfaceRequired* slave;
  mtsInterfaceRequired* control;
  mtsInterfaceRequired* increase;
  mtsInterfaceRequired* decrease;

  

  // Control
  vctDynamicVector<double> qready;
  vctDynamicVector<double> qsold;
  vctDynamicVector<double> tauold;

  enum State{ IDLE, RESET, ENABLE, TEST, HYBRID };
  State state;
  bool enable;

  double fz;

  // used to record PID input/output
  std::list<double> dt;
  std::ofstream ofsq, ofsqs;

  // Slave side
  //! Read the joint positions
  mtsFunctionRead  GetPosition;
  //! Write the joint torques
  mtsFunctionWrite SetPosition;

  // Master side 
  prmPositionCartesianGet prmCommandSE3;    // Cartesian command (master side)
  prmPositionCartesianGet prmTelemetrySE3;  // Cartesian telemetry
  prmPositionJointGet     prmTelemetryRn;   // Joint telemetry

  void HybridControl();
  void MoveToReady();
  
  vctDynamicVector<double> sg;
  std::list< osaJR3ForceSensor::Wrench > stdft;    

 public:

  void Increase( const prmEventButton& event );
  void Decrease( const prmEventButton& event );

  void Test(){ state = TEST; }
  void Hybrid(){ state = HYBRID; }
  void Reset(){ state = RESET; }
  void Force(){ fz -= 5.0; }
  void Enable(){ enable = !enable; }

  bool IsEnabled(){ return enable; }

 public:
  
  mtsHybridForcePosition(  const std::string& name,
			   double period,
                           
			   const std::string& robotfilename, 
			   const vctFrame4x4<double>& Rtw0,
			   const vctFrame4x4<double>& Rtnt,
			   const vctDynamicVector<double>& qinit,
			   const vctDynamicVector<double>& qready,

			   osaJR3ForceSensor* jr3,
			   osaGravityCompensation* gc,
			   osaHybridForcePosition* hfp );

  
  void Configure( const std::string& );
  void Startup();
  void Run();
  
};

