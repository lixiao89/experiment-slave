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

#include <cisstCommon/cmnPath.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstOSAbstraction/osaGetTime.h>

#include <cisstNumerical/nmrSavitzkyGolay.h>

//#include <sawOpenSceneGraph/osaOSGWorld.h>
//#include <sawOpenSceneGraph/osaOSGMono.h>
//#include <sawOpenSceneGraph/osaOSGManipulator.h>

#include <sawCANBus/osaSocketCAN.h>
#include <sawBarrett/osaWAM.h>

#include <sawJR3ForceSensor/osaJR3ForceSensor.h>

#include <cisstNumerical/nmrLSMinNorm.h>

#include <sawControllers/osaPIDAntiWindup.h>
#include <sawControllers/osaHybridForcePosition.h>
#include <sawControllers/osaGravityCompensation.h>

#include <stdio.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

// Convolution function
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

// my own getch
int getch(){

    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt );
    newt = oldt;
    newt.c_lflag &= ~(ICANON);
    newt.c_lflag &= ~(ECHO);
    newt.c_cc[VMIN] = 0;
    newt.c_cc[VTIME] = 0;
    tcsetattr( STDIN_FILENO, TCSANOW, &newt );
    
    int c = getchar();
    
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
    
    return c;
    
}

int main( int argc, char** argv ){

    cmnLogger::SetMask( CMN_LOG_ALLOW_ALL );
    cmnLogger::SetMaskFunction( CMN_LOG_ALLOW_ALL );
    cmnLogger::SetMaskDefaultLog( CMN_LOG_ALLOW_ALL );
    
    if( argc != 3 ){
        std::cout << "Usage: " << argv[0] << " can[0-1] robfile" << std::endl;
        return -1;
    }

    // CAN bus
    osaSocketCAN can( argv[1], osaCANBus::RATE_1000 );
    if( can.Open() != osaCANBus::ESUCCESS ){
        CMN_LOG_RUN_ERROR << argv[0] << "Failed to open " << argv[1]
                          << std::endl;
        return -1;
    }
  
    // WAM
    osaWAM osaWAM( &can );
    if( osaWAM.Initialize() != osaWAM::ESUCCESS ){
        CMN_LOG_RUN_ERROR << "Failed to initialize WAM" << std::endl;
        return -1;
    }
    
    // Initial WAM position
    vctDynamicVector<double> qinit( 7, 0.0 );
    qinit[1] = -cmnPI_2;
    qinit[3] =  cmnPI;
    qinit[5] = -cmnPI_2;

    if( osaWAM.SetPositions( qinit ) != osaWAM::ESUCCESS ){
        CMN_LOG_RUN_ERROR << "Failed to set position: " << qinit << std::endl;
        return -1;
    }

    // orientation of the tool wrt FT sensor (about 18 degrees about +Y)
    vctMatrixRotation3<double> Rst( 0.9511,  0.0000,  -0.3090,
                                    0.0000,  1.0000,   0.0000,
                                    0.3090,  0.0000,   0.9511,
                                    VCT_NORMALIZE );

    // position of the tool wrt FT sensor (9cm along +z)
    vctFixedSizeVector<double,3> tst( 0.0, 0.0, 0.09 );
    vctFrame4x4<double> Rtst( Rst, tst );

    // mass and center of the tool (measured)
    double mass = 0.145;
    vctFixedSizeVector<double,3> com( 0.0, 0.0, 0.03 );

    // JR3 FT sensor
    osaJR3ForceSensor::Wrench zero( 0.0 );
    osaJR3ForceSensor jr3( "/dev/comedi0",
                           osaJR3ForceSensor::METRIC,
                           zero,
                           Rtst,     // tool offset
                           mass,     // tool mass
                           com );    // tool center of mass
    jr3.Open();
   
    // SG filter to filter FT
    vctDynamicVector<double>  sg = nmrSavitzkyGolay( 1, 0, 100, 0 );
    std::list< osaJR3ForceSensor::Wrench > stdft;    


    // Rotation of the WAM base (-90 degrees about +Y)
    vctMatrixRotation3<double> Rw0(  0.0,  0.0, -1.0,
                                     0.0,  1.0,  0.0,
                                     1.0,  0.0,  0.0 );
    vctFixedSizeVector<double,3> tw0(0.0);
    vctFrame4x4<double> Rtw0( Rw0, tw0 );

    // this is used to evaluate the kinematics and Jacobian
    robManipulator robWAM( argv[2], Rtw0 );

    // Tool to attach to the WAM
    // transform of the tool wrt WAM link 7 (18 deg about +Y)
    vctMatrixRotation3<double> R7t( Rst );
    // position of the TCP wrt WAM link 7
    vctFixedSizeVector<double,3> t7t( 0.0, 0.0, 0.15 );
    vctFrame4x4<double> Rt7t( R7t, t7t );

    // Create a tool and attach it to the WAM
    robManipulator robtool( Rt7t );
    robWAM.Attach( &robtool );

    // Evaluate the currrent position/orientation of the JR3 sensor wrt world
    vctFrame4x4<double> Rtwts;
    {
        // Get the joints
        vctDynamicVector<double> q;
        if( osaWAM.GetPositions( q ) != osaWAM::ESUCCESS ){
            CMN_LOG_RUN_ERROR << "Failed to get positions" << std::endl;
            return -1;
        }

        // Initial desired position/orientation
        Rtwts = robWAM.ForwardKinematics( q );

        // Evaluate the forward kin at the 7 joint of the WAM. Insists on the 
        // kinematics of the 7th link. Otherwise FK will return the 
        // position/orientation of the tool. Here we want the orientation of 
        // the JR3 to compensate for the tool mass.
        jr3.Zero( robWAM.ForwardKinematics( q, 7 ) );
    }

    // Gravity compensation controller
    osaGravityCompensation GC( argv[2], Rtw0 );
    
    // gains (proportional, integral, derivative, anti-windup)
    vctDynamicVector<double> Kp(7,1800.0,1800.0,1800.0,1800.0,180.0,180.0,40.0);
    vctDynamicVector<double> Ki(7,3200.0,3200.0,3200.0,3200.0,200.0,200.0,80.0);
    vctDynamicVector<double> Kd(7, 10.0, 10.0, 10.0, 10.0, 0.8, 0.8, 0.2);
    vctDynamicVector<double> Kt(7, 5.0);
    vctDynamicVector<double> tlimit(7, 20.0, 20.0, 20.0, 20.0, 5.0, 5.0, 1.5);
    //vctDynamicVector<double> tlimit(7, 40.0, 40.0, 40.0, 40.0, 15.0, 15.0, 6.5);
    osaPIDAntiWindup pid( Kp, Ki, Kd, Kt, tlimit, qinit );
    /*
    osaHybridForcePosition::Mask mask( osaHybridForcePosition::IDLE, 
                                       osaHybridForcePosition::IDLE, 
                                       osaHybridForcePosition::FORCE, 
                                       osaHybridForcePosition::IDLE, 
                                       osaHybridForcePosition::IDLE, 
                                       osaHybridForcePosition::IDLE );
    */
    osaHybridForcePosition::Mask mask( osaHybridForcePosition::POSITION, 
                                       osaHybridForcePosition::POSITION, 
                                       osaHybridForcePosition::POSITION, 
                                       osaHybridForcePosition::POSITION, 
                                       osaHybridForcePosition::POSITION, 
                                       osaHybridForcePosition::POSITION );
    vctFixedSizeVector<double,6> K( .01, .01, .01, .01, .01, .01 );
    osaHybridForcePosition hfp( mask, robWAM, robtool, K );
    
    char c = 'a';
    double t1 = osaGetTime();

    // qs is the desired joint positions
    vctDynamicVector<double> qs( qinit );

    while( c != 'q' ){
        
        char tmp = getch();
        if( tmp == 'g' ||  // gravity
            tmp == 'h' ||  // hold
            tmp == 'f' ||  // ft 
            tmp == 'i' ||  // idel
            tmp == 'q' ) { c = tmp; }
        
        // Get the positions
        vctDynamicVector<double> q;
        if( osaWAM.GetPositions( q ) != osaWAM::ESUCCESS ){
            CMN_LOG_RUN_ERROR << "Failed to get positions" << std::endl;
            return -1;
        }

        // position/orientation of the TCP
        vctFrame4x4<double> Rtwt = robWAM.ForwardKinematics( q );

        // orientation of the JR3 sensor
        vctMatrixRotation3<double> Rws( Rtwt.Rotation()*Rst.Inverse() );

        // Read a wrench in TCP frame (compensate for tool orientation)
        osaJR3ForceSensor::Wrench ft;
        jr3.Read( ft, Rws, true, 2 );

        // filter the wrench
        stdft.push_back( ft );
        if( sg.size() < stdft.size() ) { stdft.pop_front(); }
        ft = convolve( stdft, sg );

        double t2 = osaGetTime();
        double dt = t2 - t1; // time difference
        t1 = t2;

        switch( c ){
            
            // 'g'ravity compensation
        case 'g':
            {
                // torques
                vctDynamicVector<double> tau( q.size(), 0.0 );
                
                // evaluate the GC
                if( GC.Evaluate( q, tau ) != osaGravityCompensation::ESUCCESS ){
                    CMN_LOG_RUN_ERROR << "Failed to evaluate controller"
                                      << std::endl;
                    return -1;
                }

                // set the torques
                if( osaWAM.SetTorques( tau ) != osaWAM::ESUCCESS ){
                    CMN_LOG_RUN_ERROR << "Failed to set torques" << std::endl;
                    return -1;
                }
                
                // reset the desired hold position
                qs = q;	
                Rtwts = Rtwt;

            }
            break;
            
            // 'h'old the position
        case 'h':
            {
                // torques
                vctDynamicVector<double> tau( q.size(), 0.0 );

                // evaluate the PID
                if(pid.Evaluate(qs, q, tau, dt) != osaPIDAntiWindup::ESUCCESS){
                    CMN_LOG_RUN_ERROR << "Failed to evaluate controller" 
                                      << std::endl;
                    return -1;
                }

                // set the torques
                if( osaWAM.SetTorques( tau ) != osaWAM::ESUCCESS ){
                    CMN_LOG_RUN_ERROR << "Failed to set torques" << std::endl;
                    return -1;
                }
                
            }
            break;

            // 'f'orce/torque 
        case 'f':
            {
                // desired force
                vctDynamicVector<double> fts( 6, 0.0 );
                fts[2] = -1.0;  // -1N along z
                
                vctDynamicVector<double> qd( 7, 0.0 );
                hfp.Evaluate( qd, q, ft, fts, Rtwt, Rtwts );

                q = q + qd*dt;
                //std::cout << "dq: " << qd*dt << std::endl;
                //std::cout << qs << std::endl;
                //std::cout << q << std::endl;
                
                // evaluate the PID
                vctDynamicVector<double> tau( q.size(), 0.0 );
                if(pid.Evaluate(qs, q, tau, dt) != osaPIDAntiWindup::ESUCCESS){
                    CMN_LOG_RUN_ERROR << "Failed to evaluate controller" 
                                      << std::endl;
                    return -1;
                }

                /*
                // evaluate the GC
                if( GC.Evaluate( q, tau ) != osaGravityCompensation::ESUCCESS ){
                    CMN_LOG_RUN_ERROR << "Failed to evaluate controller"
                                      << std::endl;
                    return -1;
                }
                */
                //std::cout << "t: " << tau << std::endl;
                // set the torques
                if( osaWAM.SetTorques( tau ) != osaWAM::ESUCCESS ){
                    CMN_LOG_RUN_ERROR << "Failed to set torques" << std::endl;
                    return -1;
                }
                
            }
            break;
            
        default:
            {
                vctDynamicVector<double> tau( q.size(), 0.0 );
                if( osaWAM.SetTorques( tau ) != osaWAM::ESUCCESS ){
                    CMN_LOG_RUN_ERROR << "Failed to set torques" << std::endl;
                    return -1;
                }
            }
            
        }
        //camera->frame();
    }
    
    return 0;

}


    /*
    osg::ref_ptr< osaOSGWorld > world = new osaOSGWorld;
    // Create a camera
    int x = 0, y = 0;
    int width = 650, height = 480;
    double Znear = 0.1, Zfar = 10.0;
    osg::ref_ptr< osaOSGCamera > camera;
    camera = new osaOSGMono( world,
                             x, y, width, height,
                             55.0, ((double)width)/((double)height),
                             Znear, Zfar );
    camera->Initialize();

    cmnPath path;
    path.AddRelativeToCisstShare("/models");
    path.AddRelativeToCisstShare("/models/WAM");
    path.Add("/home/sleonard/src/wvu-jhu/trunk/models/WAM");

    std::vector< std::string > models;
    models.push_back( path.Find("l1.obj") );
    models.push_back( path.Find("l2.obj") );
    models.push_back( path.Find("l3.obj") );
    models.push_back( path.Find("l4.obj") );
    models.push_back( path.Find("l5.obj") );
    models.push_back( path.Find("l6.obj") );
    models.push_back( path.Find("l7jr3cutter.obj") );

    osg::ref_ptr< osaOSGBody > osgtcp;
    osgtcp = new osaOSGBody( path.Find( "axes.3ds" ), world,
                             vctFrame4x4<double>(), 0.1  );

    osg::ref_ptr<osaOSGManipulator> osgWAM;
    osgWAM = new osaOSGManipulator( models,
                                    world,
                                    Rtw0,
                                    path.Find("wam7.rob"),
                                    path.Find("l0.obj") );
    vctDynamicVector<double> qosg( qinit );
    */



        /*
        //osgtcp->SetTransform( Rtwt );
        q = qs;
        qosg = qs;
        osgtcp->SetTransform( robWAM.ForwardKinematics( qosg ) );
        osgWAM->SetPositions( qosg );
        */
