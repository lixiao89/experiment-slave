/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-  */
/*ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab:*/

/*
  $Id: osaHybridForcePosition.cpp 4416 2013-08-20 01:45:09Z sleonar7 $
  
  Author(s):  Simon Leonard
  Created on: 2013-12-20
  
  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights Reserved.
  
  --- begin cisst license - do not edit ---
  
  This software is provided "as is" under an open source license, with
  no warranty.  The complete license can be found in license.txt and
  http://www.cisst.org/cisst/license.txt.
  
--- end cisst license ---
*/

#include <cisstVector/vctRodriguezRotation3.h>
#include <cisstVector/vctMatrixRotation3.h>
#include <cisstNumerical/nmrLSMinNorm.h>

#include "osaHybridForcePosition.h"

osaHybridForcePosition::osaHybridForcePosition(  const Mask& mask,
                                                 const vct6& K ) :
    mask( mask ),
    K( K ){}

void osaHybridForcePosition::SetMask( const Mask& mask )
{ this->mask = mask; }

vctFrm4x4 osaHybridForcePosition::Evaluate( const vctFrm4x4& Rtwts, 
                                            const vct6& ws, 
                                            const vct6& w ){
    // wrench error
    vct6 ew = ws - w;

    // compute position increment
    vct3 v( 0.0 );
    for( size_t i=0; i<3; i++ ){
        switch( mask[i] ){
        case osaHybridForcePosition::IDLE:
        case osaHybridForcePosition::POSITION:
            break;
        case osaHybridForcePosition::FORCE:
            if( ew[i] < -1 )                         { v[i]= 0.00001; }
            if(     -1 <= ew[i] && ew[i] < 0 )       { v[i]= 0.00001*ew[i]; }
            if(      0 <= ew[i] && ew[i] < 1 )       { v[i]=-0.00001*ew[i]; }
            if(                        1 <= ew[i] )  { v[i]=-0.00001; }
            if( fabs( ew[i] ) < 0.5 )                { v[i]= 0.0; }
            break;
        }
    }

    // translation increment caused by the force control
    vctFrm4x4 Rttts( vctRot3(), v );

    // apply the force correction to the command position
    return Rtwts*Rttts;
    
}
