set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstCommonXML
  cisstVector
  cisstNumerical
  cisstOSAbstraction
  cisstMultiTask
  cisstRobot
  cisstParameterTypes)

find_package(cisst REQUIRED ${REQUIRED_CISST_LIBRARIES} )
      
if( cisst_FOUND_AS_REQUIRED )
  
  include( ${CISST_USE_FILE} )
  
  cisst_find_saw_component( sawKeyboard )
  cisst_find_saw_component( sawControllers )
  cisst_find_saw_component( sawJR3ForceSensor )
  cisst_find_saw_component( sawBarrett )
  cisst_find_saw_component( sawCANBus )
  
  if( sawKeyboard_FOUND       AND 
      sawJR3ForceSensor_FOUND AND 
      sawControllers_FOUND    AND
      sawBarrett_FOUND        AND
      sawCANBus_FOUND ) 

    include_directories( ${sawKeyboard_INCLUDE_DIR} )
    include_directories( ${sawControllers_INCLUDE_DIR} )
    include_directories( ${sawJR3ForceSensor_INCLUDE_DIR} )
    include_directories( ${sawBarrett_INCLUDE_DIR} )
    include_directories( ${sawCANBus_INCLUDE_DIR} )

    link_directories( ${sawKeyboard_LIBRARY_DIR} )
    link_directories( ${sawControllers_LIBRARY_DIR} )
    link_directories( ${sawJR3ForceSensor_LIBRARY_DIR} )
    link_directories( ${sawBarrett_LIBRARY_DIR} )
    link_directories( ${sawCANBus_LIBRARY_DIR} )
    
    add_executable( hybridctrl 
      main.cpp 
      osaHybridForcePosition.cpp 
      mtsUserStudySlave.cpp )

    target_link_libraries( 
      hybridctrl
      ${sawKeyboard_LIBRARIES}
      ${sawJR3ForceSensor_LIBRARIES}
      ${sawControllers_LIBRARIES}
      ${sawBarrett_LIBRARIES}
      ${sawCANBus_LIBRARIES}
      pthread)

    cisst_target_link_libraries(hybridctrl ${REQUIRED_CISST_LIBRARIES})
    
  endif( sawKeyboard_FOUND       AND 
         sawJR3ForceSensor_FOUND AND 
         sawControllers_FOUND    AND
         sawBarrett_FOUND        AND
         sawCANBus_FOUND ) 

    add_executable( parsexml parsexml.cpp ${HEADER_FILES} )
    cisst_target_link_libraries( parsexml ${REQUIRED_CISST_LIBRARIES} )

endif ( cisst_FOUND_AS_REQUIRED )
