/*-*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-   */
/*ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab:*/

/*
  $Id: osaFTControlWAMExample.cpp 3181 2011-11-15 15:41:28Z sleonard $

  Author(s):  Simon Leonard
  Created on: 2013-07-22

  (C) Copyright 2012 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstCommonXML.h>
#include <cisstCommon/cmnPath.h>
#include <sawKeyboard/mtsKeyboard.h>
#include <sawCANBus/osaSocketCAN.h>
#include <sawBarrett/osaWAM.h>

#include <sawControllers/mtsController.h>
#include <sawControllers/osaPIDAntiWindup.h>
#include <sawControllers/mtsGravityCompensation.h>

#include "mtsUserStudySlave.h"
#include <cisstParameterTypes.h>

class mtsParser : public mtsComponent {

private:

    cmnXMLPath config;
    
    mtsInterfaceRequired* keyboard;

    mtsInterfaceRequired* slave;
    mtsFunctionWrite mtsSetWrench;
    
    mtsInterfaceProvided* master;
    prmPositionCartesianGet prmPlane;
    mtsBool mtsHaptics;
    mtsDouble mtsDelay;
    mtsDouble mtsForce;
    
    int numtrials;
    int trialcounter;
    
    enum State{ PLAY, PAUSED };
    State state;
    
public:
    
    mtsParser( const std::string& name ) : 
        mtsComponent( name ),
        numtrials( 0 ),
        trialcounter( 0 ),
        state( PAUSED ){
        
        slave = AddInterfaceRequired( "Slave" );
        if( slave )
            { slave->AddFunction( "SetWrench", mtsSetWrench ); }
        
        
        master = AddInterfaceProvided( "Master" );
        if( master ){
            master->AddCommandRead( &mtsParser::ReadPosition, this,"GetPositionCartesian" );
            master->AddCommandRead( &mtsParser::ReadDelay, this,"GetDelay" );
            master->AddCommandRead( &mtsParser::ReadHaptics, this,"GetHaptics" );
            master->AddCommandRead( &mtsParser::ReadForce, this,"GetForce" );
        }
        
        keyboard = AddInterfaceRequired( "Keyboard" );
        if( keyboard ){
            keyboard->AddEventHandlerVoid(&mtsParser::StartPause, this, "StartStop");
        }
        
    }
    
    void ReadPosition( prmPositionCartesianGet& Rt ) const 
    { Rt = prmPlane; }

    void ReadHaptics( mtsBool& haptics ) const 
    { haptics = mtsHaptics; }

    void ReadDelay( mtsDouble& delay ) const 
    { delay = mtsDelay; }

    void ReadForce( mtsDouble& force ) const 
    { force = mtsForce; }

    void Configure( const std::string& filename ){
        
        config.SetInputSource( filename );
        config.Query("count(/experiment/*)", numtrials);
        std::cout << "Using experiment file: " << filename << std::endl;
        std::cout << "Number of trials: " << numtrials << std::endl;
        
    }
    
  void StartPause(){

      if( state == PAUSED ){
          
          if( trialcounter < numtrials ){
              state = PLAY;
              trialcounter++;
              
              std::stringstream context;
              context << "experiment/trial[" << trialcounter << "]";
              int id;
              std::string user;
              std::string name;
              config.GetXMLValue( context.str().c_str(), "@name", name );
              config.GetXMLValue( context.str().c_str(), "@user", user );
              
              vctFixedSizeVector<double,6> ft( 0.0 );
              config.GetXMLValue( context.str().c_str(), "@force", ft[2] );
              mtsForce = ft[2];
              mtsForce.SetValid( true );

              bool haptics=false;
              config.GetXMLValue( context.str().c_str(), "@haptics", haptics );
              mtsHaptics = haptics;
              mtsHaptics.SetValid( true );

              double delay=0.0;
              config.GetXMLValue( context.str().c_str(), "@delay", delay );
              mtsDelay = delay;
              mtsDelay.SetValid( true );

              vctQuaternionRotation3<double> q;
              try{
                  double x, y, z, w;
                  std::stringstream orientation_context;
                  orientation_context << context.str();
                  orientation_context << "/plane/orientation";
                  config.GetXMLValue(orientation_context.str().c_str(),"@x",x);
                  config.GetXMLValue(orientation_context.str().c_str(),"@y",y);
                  config.GetXMLValue(orientation_context.str().c_str(),"@z",z);
                  config.GetXMLValue(orientation_context.str().c_str(),"@w",w);
                  q=vctQuaternionRotation3<double>( x, y, z, w, VCT_NORMALIZE );

                  vctFixedSizeVector<double,3> t;
                  std::stringstream translation_context;
                  translation_context << context.str();
                  translation_context << "/plane/translation";
                  config.GetXMLValue(translation_context.str().c_str(),"@x",t[0]);
                  config.GetXMLValue(translation_context.str().c_str(),"@y",t[1]);   
                  config.GetXMLValue(translation_context.str().c_str(),"@z",t[2]);
                  std::cout << "Trial " << trialcounter << " of " << numtrials
                            << "\n Force: " << ft[2]
                            << "\n Delay: " << delay
                            << "\n haptics: " << (mtsHaptics ? "true" : "false")
                            << "\n Plane: " << q << " " << t << std::endl;

                  vctFrm3 frm( vctMatrixRotation3<double>(q), t );
                  prmPlane.SetPosition( frm );
                  prmPlane.SetValid( true );
              }
              catch(...){
                  std::cout << "Trial " << trialcounter << " of " << numtrials
                            << "\n Force: " << ft[2]
                            << "\n Delay: " << delay 
                            << "\n haptics: " << (mtsHaptics ? "true" : "false")
                            << "\n Plane: manual" << std::endl;
                  prmPlane.SetValid( false );
              }
              
              prmForceCartesianGet prmFT;
              prmFT.SetForce( ft );
              prmFT.SetValid( true );
              
              mtsSetWrench( prmFT );
          }
          
      }
      else{
 
          vctFixedSizeVector<double,6> ft( 0.0 );
          prmForceCartesianGet prmFT;
          prmFT.SetForce( ft );
          prmFT.SetValid( true );
          mtsSetWrench( prmFT );

          mtsDelay = 0.0;
          mtsDelay.SetValid( false );
          mtsHaptics = false;
          mtsHaptics.SetValid( false );

          std::cout << "Trial stoped" << std::endl; 
          if( numtrials == trialcounter )
              { std::cout << "Experiment finished" << std::endl; }
          state = PAUSED;      
      }
      
  }
    
};

class mtsPIDAntiWindup : public mtsController {
    
    osaPIDAntiWindup* pid;
    osaGravityCompensation* gc;
    osaWAM* wam;

    mtsInterfaceProvided* slave;
    prmPositionJointGet qs;
    prmPositionJointGet q;
    std::list<double> dt;

    bool gcOn;

public:

    mtsPIDAntiWindup( const std::string& name,
                      double period,
                      osaWAM* wam,
                      osaPIDAntiWindup* pid,
                      osaGravityCompensation* gc ) :
        mtsController( name, period, OSA_CPU4 ),
        pid( pid ),
        gc( gc ),
        wam( wam ),
        gcOn( true ){
        
        if( wam->GetPositions( q.Position() ) != osaWAM::ESUCCESS )
            { CMN_LOG_RUN_ERROR << "Failed to get positions" << std::endl; }
        qs.Position() = q.Position();

        ctl->AddEventHandlerVoid( &mtsPIDAntiWindup::Toggle, this, "StartStop" );

        slave = AddInterfaceProvided( "Slave" );
        if( slave ){
            StateTable.AddData( qs, "PositionCMD" );
            StateTable.AddData( q,  "PositionMSR" );
            slave->AddCommandWriteState( StateTable, qs, "SetPositionCMD" );
            slave->AddCommandReadState( StateTable, q,  "GetPositionMSR" );
        }
        
    }
    ~mtsPIDAntiWindup(){}

    void Toggle(){ 
        gcOn = !gcOn; 
        if( gcOn ) { std::cout << "mtsPIDAntiWindup: Gravity" << std::endl; }
        else       { std::cout << "mtsPIDAntiWindup: PID" << std::endl; }
    }

    void SendTorques( const vctVec& tau )
    { wam->SetTorques( tau ); }

    vctVec EvaluatePID( const vctVec& q, const vctVec& qs ){
        
        // period
        double dt = GetPeriodicity();

        vctVec qtmp( q );
        /*
        vctVec dq = q - qs;
        if( 0.1 < dq.Norm() ) { 
            std::cout << "Tracking error " << dq.Norm() << std::endl;
            exit(-1);
        }
        */  

        // evaluate the PID
        vctVec tau( q.size(), 0.0 );
        if( pid->Evaluate( qs, qtmp, tau, dt )  != osaPIDAntiWindup::ESUCCESS ){
            CMN_LOG_RUN_ERROR << "Failed to evaluate controller" << std::endl;
            exit(-1);
        }

        return tau;
        
    }
    
    vctVec EvaluateGC( const vctVec& q ){
        vctVec tau( q.size(), 0.0 );
        gc->Evaluate( q, tau );
        return tau;
    }

    void Configure( const std::string& ){}
    void Startup(){ 
        Thread.SetPriority( 80 ); 
        osaCPUSetAffinity( OSA_CPU3 );
    }
    void Cleanup(){}

    void Run(){ 
        
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
            //std::cout << dt.size() / avg << " " << fabs(GetPeriodicity() - avg / dt.size()) << " " 
            //        << fabs( GetPeriodicity() - max ) << std::endl;
            dt.clear();
        }

        ProcessQueuedCommands(); 
        ProcessQueuedEvents(); 

        // current joints
        if( wam->GetPositions( q.Position() ) != osaWAM::ESUCCESS ){
            CMN_LOG_RUN_ERROR << "Failed to get positions" << std::endl;
            q.SetValid( false );
        }
        else{ q.SetValid( true ); }

        if( IsEnabled() ){

            if( pid != NULL && !gcOn )
                { SendTorques( EvaluatePID( q.Position(), qs.Position() ) ); }
            
            if( gc != NULL && gcOn ){ 
                SendTorques( EvaluateGC( q.Position() ) ); 
                qs.Position() = q.Position();
                pid->Reset( qs.Position() );
            }

        }
        else{
            SendTorques( vctVec( 7, 0.0 ) );
            qs.Position() = q.Position();
            pid->Reset( qs.Position() );
        }

    }

};

int main( int argc, char** argv ){

    cmnLogger::SetMask( CMN_LOG_ALLOW_ALL );
    cmnLogger::SetMaskFunction( CMN_LOG_ALLOW_ALL );
    cmnLogger::SetMaskDefaultLog( CMN_LOG_ALLOW_ALL );
    
    if( argc != 5 ){
        std::cout << "Usage: " << argv[0] << " GCM robfile can0-1" <<std::endl;
        return -1;
    }

    std::string processname( "Slave" );
    mtsManagerLocal* taskManager = NULL;

    try{ taskManager = mtsTaskManager::GetInstance( argv[1], processname ); }
    catch( ... ){
        std::cerr << "Failed to connect to GCM: " << argv[1] << std::endl;
        return -1;
    }

    // initial joint position
    vctVec qinit( 7, 0.0 );
    qinit[1] = -cmnPI_2;
    qinit[3] =  cmnPI - 0.01;  
    qinit[5] = -cmnPI_2;

    // WAM part
    osaSocketCAN* osacan = NULL;
    osacan = new osaSocketCAN( argv[3], osaCANBus::RATE_1000 );
    if( osacan->Open() != osaCANBus::ESUCCESS ){
        std::cout << argv[0] << "Failed to open " << argv[3] << std::endl;
        return -1;
    }

    osaWAM* osawam = NULL;
    osawam = new osaWAM( osacan );
    osawam->Initialize();
    osawam->SetPositions( qinit );

    // PID part
    // gains (proportional, integral, derivative, anti-windup)
    vctDynamicVector<double> Kp(7,4000.0,3000.0,2000.0,1500.0,180.0,180.0,20.0);
    vctDynamicVector<double> Ki(7,3200.0,3200.0,2200.0,3200.0,200.0,200.0,30.0);
    vctDynamicVector<double> Kd(7,   8.0,   8.0,   8.0,   5.0,  0.5,  0.5,0.05);
    vctDynamicVector<double> Kt(7, 5.0);
    vctDynamicVector<double> limits(7,120.0,110.0,110.0, 50.0, 15.0, 15.0, 5.5);

    osaPIDAntiWindup* osapid = NULL;
    osapid = new osaPIDAntiWindup( Kp, Ki, Kd, Kt, limits, qinit );

    osaGravityCompensation* osagc = NULL;
    osagc = new osaGravityCompensation( argv[2], 
                                        vctFrm4x4( vctRot3(  0.0,  0.0, -1.0,
                                                             0.0,  1.0,  0.0,
                                                             1.0,  0.0,  0.0,
                                                             VCT_NORMALIZE ),
                                                   vct3( 0.0 ) ) );
    
    mtsPIDAntiWindup* mtspid = NULL;
    mtspid = new mtsPIDAntiWindup( "PID", 1.0/900.0, osawam, osapid, osagc );
    taskManager->AddComponent( mtspid );
    
    // Hybrid/GC controller
    mtsUserStudySlave* ctrl = NULL;
    ctrl = new mtsUserStudySlave( "Control", 1.0/500.0, argv[2] );
    taskManager->AddComponent( ctrl );

    mtsKeyboard kb;
    kb.SetQuitKey( 'q' );
    kb.AddKeyVoidEvent( 'E', "Control", "Enable" );
    kb.AddKeyVoidEvent( 'S', "Control", "StartStop" );
    kb.AddKeyVoidEvent( 't', "Control", "PrintTime");// press 't' to print current time to screen
    taskManager->AddComponent( &kb );

    if( !taskManager->Connect( processname, ctrl->GetName(), "Control",
                               processname, kb.GetName(),    "Control" ) ){
        std::cout << "Failed to connect: " 
                  << kb.GetName() << ":Keyboard to "
                  << ctrl->GetName() << ":Control"
                  << std::endl;
        return -1;
    }

    if( !taskManager->Connect( processname, mtspid->GetName(), "Control",
                               processname, kb.GetName(),    "Control" ) ){
        std::cout << "Failed to connect: " 
                  << kb.GetName() << ":Keyboard to "
                  << ctrl->GetName() << ":Control"
                  << std::endl;
        return -1;
    }

    if( !taskManager->Connect( processname, mtspid->GetName(), "Slave",
                               processname, ctrl->GetName(), "Slave") ){
        std::cout << "Failed to connect: " 
                  << mtspid->GetName() << ":Slave to "
                  << ctrl->GetName() << ":Slave"
                  << std::endl;
        return -1;
    }
    
    mtsParser* parser = new mtsParser( "Parser" );
    parser->Configure( argv[4] );
    taskManager->AddComponent( parser );
    
    if( !taskManager->Connect( parser->GetName(),   "Keyboard",
                               kb.GetName(),        "Control" ) ){
        std::cout << "Failed to connect to keyboard" << std::endl;
        return -1;
    }

    if( !taskManager->Connect( parser->GetName(), "Slave",
                               ctrl->GetName(), "Input" ) ){
        std::cout << "Failed to connect to controller" << std::endl;
        return -1;
    }

    taskManager->CreateAll();
    taskManager->StartAll();

    std::cout << "1) Press E to enable the controllers" << std::endl;
    std::cout << "2) Press S to start the first trial" << std::endl;
    std::cout << "3) Press S to stop the first trial" << std::endl;

    std::cout << "q) To quit then press CTRL-C" << std::endl;
    pause();
    
    return 0;
    
}


