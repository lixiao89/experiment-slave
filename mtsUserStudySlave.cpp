/*-*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-   */
/*ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab:*/

/*
  $Id: mtsUserStudySlave.cpp 3181 2011-11-15 15:41:28Z sleonard $

  Author(s):  Simon Leonard
  Created on: 2013-07-22

  (C) Copyright 2012 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/
/*
    osaJR3ForceSensor::Wrench zero( 0.0 );
    osaJR3ForceSensor 
    jr3.Open();
*/

#include "mtsUserStudySlave.h"
#include <cisstParameterTypes/prmForceTorqueJointSet.h>
#include <cisstOSAbstraction/osaCPUAffinity.h>

osaJR3ForceSensor::Wrench 
convolve
( const std::list< osaJR3ForceSensor::Wrench >& stdft,
  const vctDynamicVector<double>& sg ){
    
    osaJR3ForceSensor::Wrench ft( 0.0 );
    std::list< osaJR3ForceSensor::Wrench >::const_iterator fti;
    size_t i=0;
    for( fti=stdft.begin(); fti!=stdft.end(); fti++, i++ )
        { ft = ft + sg[i] *(*fti); }
    
    return ft;
}

mtsUserStudySlave::mtsUserStudySlave
( const std::string& name,
  double period,
  
  const std::string& robotfilename, 
  const vctFrame4x4<double>& Rtw0,
  const vctFrame4x4<double>& Rtnt,
  const vctDynamicVector<double>& qinit ):
    
    mtsTaskPeriodic( name, period, true ),     // task

    robot( robotfilename, Rtw0 ),              // robot kinematics
    tool( Rtnt ),                              // tool kinematics
    se3traj( NULL ),                           // trajectory
    timer( 0.0 ),
    Rts( Rtnt.Rotation().Transpose() ),
    
    jr3( "/dev/comedi0",                               // comedi device
         osaJR3ForceSensor::METRIC,                    // metric units
         osaJR3ForceSensor::Wrench( 0.0 ),             // zero
         vctFrm4x4( vctRot3( 0.9511,  0.0000,  -0.3090,// ori tool wrt sensor
                             0.0000,  1.0000,   0.0000,
                             0.3090,  0.0000,   0.9511,
                             VCT_NORMALIZE ),
                    vct3( 0.0, 0.0, 0.09 ) ),          // pos tool wrt sensor
         0.145,                                        // tool mass
         vct3(  0.0, 0.0, 0.03 ) ),                    // tool COM

    hfp( osaHybridForcePosition::Mask( osaHybridForcePosition::POSITION ),
         vct6( 0.3, 0.3, 0.3, 0.1, 0.1, 0.1 ) ),
    
    mastertel( NULL ),
    mastercmd( NULL ),
    slave( NULL ),
    control( NULL ),

    state( IDLE ),

    sg( nmrSavitzkyGolay( 1, 0, 100, 0 ) ),
    logcnt( 0 ),
    logtime( 0.0 ),
    failstate(false){

    //--------------------------------------------------------
    //iniital value of estimated mu (Coeff. of friction) and Fc
    vctFixedSizeVector<double,2> xinit(0.5,1);
    rls = new RLSestimator(xinit);
    startTime = osaGetTime();
    ofsForceData.open("/home/lixiao/Desktop/experiment-slave-data.txt");
    //--------------------------------------------------------
    robot.Attach( &tool );

    jr3.Open();

    // zero the JR3 accounting for the mass of the tool
    jr3.Zero( robot.ForwardKinematics( qinit, 7 ) );

    control = AddInterfaceRequired( "Control" );
    if( control ){
        control->AddEventHandlerVoid( &mtsUserStudySlave::Toggle, 
                                      this, 
                                      "StartStop" );
        control->AddEventHandlerVoid( &mtsUserStudySlave::PrintTime, this, "PrintTime");
    }

    // This connects to the master
    mastertel = AddInterfaceProvided( "Output" );
    if( mastertel ){ 
        prmTelemetryRn.SetSize( 7 );
        prmTelemetryRn.Position().SetAll( 0 );
        StateTable.AddData( prmTelemetryRn,  "TelemetryRn" );
        mastertel->AddCommandReadState( StateTable, 
                                        prmTelemetryRn, 
                                        "GetPositionJoint" );

        StateTable.AddData( prmTelemetrySE3, "TelemetrySE3" );
        mastertel->AddCommandReadState( StateTable,
                                        prmTelemetrySE3,
                                        "GetPositionCartesian" );
    }

    mastercmd = AddInterfaceProvided( "Input" );
    if( mastercmd ){
        StateTable.AddData( prmCommandSE3, "Command" );
        mastercmd->AddCommandWriteState( StateTable,
                                         prmCommandSE3,
                                         "SetPositionCartesian" );
        StateTable.AddData( prmCommandWrench, "Wrench" );
        mastercmd->AddCommandWriteState( StateTable,
                                         prmCommandWrench,
                                         "SetWrench" );
    }

    slave = AddInterfaceRequired( "Slave" );
    if( slave ){
        slave->AddFunction( "GetPositionMSR", mtsGetPosition );
        slave->AddFunction( "SetPositionCMD", mtsSetPosition );
    }

}
  

void mtsUserStudySlave::Configure( const std::string& ){}
void mtsUserStudySlave::Startup(){
    osaCPUSetAffinity( OSA_CPU2 );
    Thread.SetPriority( 70 );
}

bool mtsUserStudySlave::GetPosition( vctVec& q ){

    // read the joint positions
    prmPositionJointGet prmq; 
    mtsGetPosition( prmq );
    q = prmq.Position();
    
    bool valid=false;
    prmq.GetValid( valid );

    return valid;
    
}

bool mtsUserStudySlave::GetPosition( vctFrm4x4& Rtwt ){

    // current joints
    vctVec q;
    bool valid = GetPosition( q );
    
    // current Cartesian pose
    Rtwt = robot.ForwardKinematics( q );
    
    return valid;

}

bool mtsUserStudySlave::GetWrench( osaJR3ForceSensor::Wrench& w ){
    
    vctFrm4x4 Rtwt;
    bool valid = GetPosition( Rtwt );
    
    // current orientation of the tool
    vctRot3 Rwt( Rtwt.Rotation() );
    // orientation of the sensor
    vctRot3 Rws( Rwt * Rts );
    jr3.Read( w, Rws, true, 3 );

    // filter the reading
    stdft.push_back( w );
    if( sg.size() < stdft.size() ) { stdft.pop_front(); }
    w = convolve( stdft, sg );
    /*
    if( 10 < fabs( w[2] ) )
        { std::cout << w[2] << std::endl; }
    */
    return valid;
    
}

void mtsUserStudySlave::SetTelemetry(){

    // set the joint telemetry 
    {
        vctVec q;
        bool valid = GetPosition( q );
        prmTelemetryRn.SetValid( valid );
        prmTelemetryRn.SetPosition( q );
    }

    // set the cartesian telemetry
    {
        vctFrm4x4 Rtwt;
        bool valid = GetPosition( Rtwt );
        vctFrm3 frm( vctRot3( Rtwt.Rotation() ), Rtwt.Translation() );
        prmTelemetrySE3.SetValid( valid );
        prmTelemetrySE3.SetPosition( frm );
    }

}

void mtsUserStudySlave::Run(){ 

    // Timing stuff
    static double t1 = osaGetTime();
    double t2 = osaGetTime();
    dt.push_back( t2 - t1 );
    t1 = t2;
    
    if( 1.0/GetPeriodicity() < dt.size() ){
        
        std::list<double>::iterator it=dt.begin();
        double avg=0.0;
        double max=0.0;
        for( ; it!=dt.end(); it++ ){
            avg += *it;
            if( max < *it ) max = *it;
        }
        //std::cout << fabs(GetPeriodicity() - avg / dt.size()) << " " 
        //        << fabs( GetPeriodicity() - max ) << std::endl;
        dt.clear();
    }

    
    /*
    cpu_set_t mask;
    sched_getaffinity( 0, sizeof( cpu_set_t ), &mask );
    std::cout << CPU_ISSET( 0, &mask ) << " "
              << CPU_ISSET( 1, &mask ) << " "
              << CPU_ISSET( 2, &mask ) << " "
              << CPU_ISSET( 3, &mask ) << std::endl;            
    */
    // get all the stuff 
    ProcessQueuedCommands(); 
    ProcessQueuedEvents(); 

    // always set telemetry cuz the master expects it
    SetTelemetry();

    // Read the master position command, if any, and update the trajectory
    // This updates Rtwtsold if the command is valid
    GetCommand();

    if( IsEnabled() ) 
        { HybridMotion(); }

    // if not enable update the old command to the robot position
    else{

        vctFrm4x4 Rt;
        if( GetPosition( Rt ) ) { 
            Rtwtsold = Rt; 
        }

        vctVec q;
        if( GetPosition( q ) ) { qsold = q; }

        // also resets the command wrench since it is not resets elsewhere
        //prmCommandWrench.SetForce( vct6( 0.0 ) );
    }

}

// Get the command from the master
void mtsUserStudySlave::GetCommand(){

    vctFrm4x4 Rtwts;

    // Try to get the new master command
    if( GetCommand( Rtwts ) ){

        // delete the old trajectory
        if( se3traj != NULL ) { delete se3traj; }
        
        // create a 100ms trajectory from current to desired position
        //vctFrm4x4 Rtwt;
        //GetPosition( Rtwt );
        se3traj = new robLinearSE3( Rtwtsold, Rtwts, 1.0/100.0 );

        Rtwtsold = Rtwts; // update the old command
        
        // this is the time that this trajectory was created
        timer = osaGetTime();
        
    }
    
}

bool mtsUserStudySlave::GetCommand( vctFrm4x4& Rtwts ){

    // Check if the command data is valid
    bool valid = false;
    prmCommandSE3.GetValid( valid ); // desired position

    // desired Cartesian pose increment
    vctFrm3 frm = prmCommandSE3.Position();
    
    // desired motion increment
    vctFrm4x4 Rttts( frm.Rotation(), frm.Translation() );
    
    // Desired pose is old command + increment command
    Rtwts = Rtwtsold * Rttts;

    // reset the command to false
    prmCommandSE3.SetValid( false );
    
    return valid;

}

bool mtsUserStudySlave::GetCommand( osaJR3ForceSensor::Wrench& ws ){

    // get the valid bit
    bool valid=false;
    prmCommandWrench.GetValid( valid );
    
    // desired force
    prmCommandWrench.GetForce( ws );

    //reset the command to false
    //prmCommandWrench.SetValid( false );

    return valid;
    
}

void mtsUserStudySlave::HybridMotion(){

    // desired pos/ori of the tool wrt world
    vctFrm4x4 Rtwts( Rtwtsold );  // initialize to the old desired position
    //vctFrm4x4 Rtwts;//( Rtwtold );  // initialize to the old desired position
    //GetPosition( Rtwts );

    // Evaluate the motion from the trajectory
    if( se3traj != NULL ){
        vct6 vw( 0.0 ), vdwd( 0.0 );
        se3traj->Evaluate( osaGetTime()-timer, Rtwts, vw, vdwd );
    }
    
    // Measured/command wrench
    osaJR3ForceSensor::Wrench w, ws;
    bool valid = GetWrench( w );
    GetCommand( ws ); 

    //-------------------------- RUNING RLS ESTIMATOR ------------------
    if(rls->Evaluate(w[2], w[0]) && !failstate){
        std::cout<<"Cutting Failure at Time: "<<osaGetTime() - startTime<<std::endl;
        failstate = true;
    }

    vctFixedSizeVector<double,2> xesti;
    double Festi;

    rls->GetEstimates(xesti,Festi);

    ofsForceData << osaGetTime() - startTime << ", "<< w[0]<< ", "<< w[2]<< ", "<< xesti[0]<< ", "<< xesti[1]<<", "<< Festi <<std::endl;



    //-----------------------------------------------------------------------

    {
        vctFrame4x4<double> Rt;
        GetPosition(Rt);
        vctVec q;
        GetPosition(q);
        ofs << "Time\n" << osaGetTime() - logtime << std::endl
            << "Cmd Cart\n" << Rtwtsold << std::endl
            << "Msr Cart\n" << Rt << std::endl
            << "Msr joints\n" << q << std::endl
            << "Cmd wrench\n" << ws << std::endl
            << "Msr wrench\n" << w << std::endl;
    }

    // small safeguard
    if( valid && w[2] < -5.0 ){ ws[2] = -4.0; }
    
    if( valid && 0 < fabs( ws[2] ) ){
        // Set the position/force mask
        osaHybridForcePosition::Mask mask( osaHybridForcePosition::POSITION );
        mask[2] = osaHybridForcePosition::FORCE; 
        hfp.SetMask( mask );
        
        // Evaluate the controller
        Rtwtsold = hfp.Evaluate( Rtwts, ws, w );
        Rtwts = Rtwtsold;
    }

    // desired joints (initialize to current joints );
    vctVec qs( qsold );
    robManipulator::Errno errno=robot.InverseKinematics( qs, Rtwts, 1e-6, 300 );
    
    // if ikin screwed up?
    if( errno != robManipulator::ESUCCESS ) { qs = qsold; }
    qsold = qs;

    // set the slave position 
    prmPositionJointGet prmq;    
    prmq.SetValid( true );
    prmq.Position() = qs;

    mtsSetPosition( prmq );
    
}
