#include <cisstCommonXML.h>
#include <cisstMultiTask.h>
#include <cisstParameterTypes.h>
#include <cisstVector.h>
#include <cisstOSAbstraction/osaSleep.h>

class mtsParser : public mtsComponent {

private:

  cmnXMLPath config;

  mtsInterfaceRequired* keyboard;

  mtsInterfaceRequired* slave;
  mtsFunctionWrite mtsSetWrench;

  mtsInterfaceRequired* master;
  mtsFunctionWrite mtsSetDelay;

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
    /*
      slave = AddInterfaceRequired( "Slave" );
      if( slave )
      { slave->AddFunction( "SetWrench", mtsSetWrench ); }
    */
    keyboard = AddInterfaceRequired( "Keyboard" );
    if( keyboard ){
      keyboard->AddEventHandlerVoid(&mtsParser::StartPause, this, "StartStop");
    }

  }

  void Configure( const std::string& filename ){
    
    config.SetInputSource( filename );
    config.Query("count(/experiment/*)", numtrials);
    std::cout << "Using experiment file: " << filename << std::endl;
    std::cout << "Number of trials: " << numtrials << std::endl;
    
  }

  void StartPause(){
#if 0
    if( state == PAUSED ){
      
      if( trialcounter < numtrials ){
	state = PLAY;
	trialcounter++;

	std::stringstream context;
	context << "experiment/trial[" << trialcounter << "]";
	int id;
	std::string name;
	config.GetXMLValue( context.str().c_str(), "@name", name );
	config.GetXMLValue( context.str().c_str(), "@id", id );

	vctFixedSizeVector<double,6> ft( 0.0 );
	config.GetXMLValue( context.str().c_str(), "@force", ft[2] );
	double delay = 0.0;
	config.GetXMLValue( context.str().c_str(), "@delay", delay );
	std::cout << "Trial " << trialcounter << " of " << numtrials
		  << "\n ID: " << id 
		  << "\n Force: " << ft[2]
		  << "\n Delay: " << delay << std::endl;
	
	prmForceCartesianGet prmFT;
	prmFT.SetForce( ft );

	// use the mask as "sticky"
	vctBool6 mask( false, false, true, false, false, false );
	prmFT.SetMask( mask );
	prmFT.SetValid( true );
	
	mtsSetWrench( prmFT );
	mtsSetDelay( delay );
	
	/*
	vctQuaternionRotation3<double> q;
	{
	  double x, y, z, w;
	  std::stringstream orientation_context;
	  orientation_context << context.str();
	  orientation_context << "/pose/orientation";
	  config.GetXMLValue( orientation_context.str().c_str(), "@x", x );
	  config.GetXMLValue( orientation_context.str().c_str(), "@y", y );
	  config.GetXMLValue( orientation_context.str().c_str(), "@z", z );
	  config.GetXMLValue( orientation_context.str().c_str(), "@w", w );
	  q = vctQuaternionRotation3<double>( x, y, z, w, VCT_NORMALIZE );
	}
	
	vctFixedSizeVector<double,3> t;
	{
	  double x, y, z;
	  std::stringstream translation_context;
	  translation_context << context.str();
	  translation_context << "/pose/translation";
	  config.GetXMLValue( translation_context.str().c_str(), "@x", t[0] );
	  config.GetXMLValue( translation_context.str().c_str(), "@y", t[1] );
	  config.GetXMLValue( translation_context.str().c_str(), "@z", t[2] );
	}
	*/	

      }
    
    }
    else{ 
      std::cout << "Trial stoped" << std::endl; 
      if( numtrials == trialcounter )
	{ std::cout << "Experiment finished" << std::endl; }
      state = PAUSED;      
    }
#endif
  }

};

int main( int argc, char** argv ){

  cmnLogger::SetMask( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskFunction( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskDefaultLog( CMN_LOG_ALLOW_ALL );

  if( argc != 2 ){
    std::cout << "Usage: " << argv[0] << " GCM" <<std::endl;
    return -1;
  }

  std::string processname( "Parser" );
  mtsManagerLocal* taskManager = NULL;
  try{ taskManager = mtsTaskManager::GetInstance( argv[1], processname ); }
  catch( ... ){
    std::cerr << "Failed to connect to GCM: " << argv[1] << std::endl;
    return -1;
  }

  mtsParser parser( "Parser" );
  //parser->Configure( "test.xml" );
  taskManager->AddComponent( &parser );

  osaSleep(0.1);
  if( !taskManager->Connect( processname, parser.GetName(),   "Keyboard",
			     "Slave",     "keyboard", "Control" ) ){
    std::cout << "Failed to connect to keyboard" << std::endl;
    return -1;
  }

  /*
  taskManager->Connect( processname, "Parser", "Slave",
			"Slave", "Control", "Input" );
  */
  taskManager->CreateAll();
  taskManager->StartAll();

  std::cout << "Press S on the slave to start" << std::endl;
  pause();
  
  return 0;

}
