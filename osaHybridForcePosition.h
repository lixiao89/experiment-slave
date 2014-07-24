/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-  */
/*ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab:*/

/*
  $Id: osaHybridForcePosition.h 4416 2013-08-20 01:45:09Z sleonar7 $
  
  Author(s):  Simon Leonard
  Created on: 2013-12-20
  
  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights Reserved.
  
  --- begin cisst license - do not edit ---
  
  This software is provided "as is" under an open source license, with
  no warranty.  The complete license can be found in license.txt and
  http://www.cisst.org/cisst/license.txt.
  
--- end cisst license ---
*/

#ifndef _osaHybridForcePosition_h
#define _osaHybridForcePosition_h

#include <cisstVector.h>
#include <cisstRobot/robManipulator.h>

class CISST_EXPORT osaHybridForcePosition{

 public:

  enum Errno{ ESUCCESS, EFAILURE };
  enum Type { IDLE, POSITION, FORCE };

  typedef vctInt6 Mask;
  
 private:
  
  Mask mask;
  vct6 K;
  
 public:
  
  osaHybridForcePosition( const Mask& mask, const vct6& K );
  
  void SetMask( const Mask& mask );

  vctFrm4x4 Evaluate( const vctFrm4x4& Rtwts, const vct6& ws, const vct6& w );
  
};

#endif
