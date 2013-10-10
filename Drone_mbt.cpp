//visp lib
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpMbEdgeTracker.h>
#include <visp/vpVideoReader.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImageTools.h>

//std lib
#include <stdio.h>
#include <iostream>
#include <string>
#include <fstream>


#include <visp/vpRzyxVector.h>
#include <visp/vpMath.h>

using namespace std;

const int   startFrame = 285;
const float metric_tower_height = 0.75;
const float metric_tower_length = 0.4;
const float scale = metric_tower_length / 0.4;
const float Theta = 0.1868/0.144;

void RotMatUpdate(float pitch, float roll, float yaw, vpRotationMatrix &update_R);

int main()
{
  //Image handler for both distorted and undistorted frames
  vpImage<unsigned char> distort_I;
  vpImage<unsigned char> undistort_I;

  //Video handler for reading the video
  vpVideoReader reader;
  
  //Image/frame grabber initialisation
  reader.setFileName("/home/shane/ViSP_Project/Drone_mbt/video.mpeg");
  reader.setFirstFrameIndex(startFrame - 2);

  reader.open(distort_I);
  reader.getFrame(distort_I, startFrame);
//  cout << reader.getFrameIndex() << endl;

  reader.acquire(distort_I);
    
  //info about the video
  cout << "video information:" << endl;
  cout << "First frame index from " << reader.getFrameIndex()-1 << endl; 
  cout << "Totally " <<  reader.getLastFrameIndex()+1 << " frames(ignorning the first frame)" << endl;

  //Display initialisation
  #if defined VISP_HAVE_X11
  vpDisplayX d;
  #elif defined VISP_HAVE_GDI
  vpDisplayGDI d;
  #elif defined VISP_HAVE_OPEN_CV
  vpDisplayOpenCV d;
  #endif

  vpMbEdgeTracker tracker1;
  vpMbEdgeTracker tracker2;

  vpHomogeneousMatrix observe_cMo;
  vpHomogeneousMatrix update_cMo;
  vpHomogeneousMatrix init_cMo;

  vpRotationMatrix observe_R;
  vpTranslationVector observe_t;
  vpTranslationVector t;

  vpRotationMatrix update_R;
  vpTranslationVector update_t;


  float timestamp, vx, vy, vz, pitch, roll, yaw, altitude;
  string line;



  // Load tracker config file (camera parameters and moving edge settings)
  tracker1.loadConfigFile("/home/shane/ViSP_Project/Drone_mbt/build/drone_camera.xml");
  tracker2.loadConfigFile("/home/shane/ViSP_Project/Drone_mbt/build/drone_camera.xml");



  // initialise an instance of vpCameraParameters with the parameters from the tracker
  vpCameraParameters undistort_cam;
  vpCameraParameters distort_cam;
  distort_cam.initPersProjWithDistortion(559.6, 557.7, 319.5, 187.2, -0.4886, 0.6002);
  tracker1.getCameraParameters(undistort_cam);
  tracker2.getCameraParameters(undistort_cam);

  
  // Load the 3D model (either a vrml file or a .cao file)
  tracker1.loadModel("CompleteTower.wrl");
  tracker2.loadModel("CompleteTower.wrl");

  //undistort the image
  vpImageTools::undistort(distort_I, distort_cam, undistort_I);
    
  d.init(undistort_I, 0, 0, "") ;

  //Initialize the tracking.
  tracker1.initClick(undistort_I, "CompleteTower.init", true);
  tracker2.initClick(undistort_I, "CompleteTower.init", true);


  //Get R and t from model-based tracking 

  cout << "This is the first frame camera pose:" << endl;
  observe_cMo.extract(observe_t);
  observe_t.print(cout, 15, "TranslationMatrix");
  cout << endl;

  cout << "Rotation:" << endl;
  observe_cMo.extract(observe_R);
  observe_R.print(cout, 15, "RotationMatrix");
  cout << endl;
 
  //update procedure
  //Next we gonna keep reading sensor data from the file, i.e pitch roll yaw 
  //open the file	
  ifstream fin("SyncFile.txt");

  //read the first piece of info
  
  //jump over the heading frames
  for(int i = 0; i < startFrame - 1; i++)
  {
  	getline(fin, line);
  }

  getline(fin, line);
  istringstream input_istring(line);

  input_istring >> timestamp >> vx >> vy >> vz >> pitch >> roll >> yaw >> altitude;

  vpDisplay::display(undistort_I);
 
  tracker1.track(undistort_I);
  tracker1.getPose(observe_cMo);

  //display the model
  tracker1.display(undistort_I, observe_cMo, undistort_cam, vpColor::red, 1);
  //vpDisplay::displayFrame (undistort_I, observe_cMo, undistort_cam, 0.05, vpColor::blue);

  
  tracker2.track(undistort_I);
  tracker2.getPose(update_cMo);
  update_cMo.extract(t);

  //display the model
  tracker2.display(undistort_I, update_cMo, undistort_cam, vpColor::blue, 1);

  vpDisplay::flush(undistort_I); 

  //wait for key event
  vpDisplay::getKeyboardEvent(undistort_I);

  //loop for the whole video frames
  while ( reader.getFrameIndex() <= reader.getLastFrameIndex() )
  {
    //get the frames and display
    reader.acquire(distort_I);
	vpImageTools::undistort(distort_I, distort_cam, undistort_I);

    vpDisplay::display(undistort_I);


    tracker1.track(undistort_I);
    tracker2.track(undistort_I);

    tracker1.getPose(observe_cMo);
    tracker2.getPose(update_cMo);

	//Display the projected model
    tracker1.display(undistort_I, observe_cMo, undistort_cam, vpColor::red, 1);
    tracker2.display(undistort_I, update_cMo, undistort_cam, vpColor::blue, 1);


    //vpDisplay::displayFrame (undistort_I, update_cMo, undistort_cam, 0.05, vpColor::blue);
	vpDisplay::flush(undistort_I);

	update_cMo.extract(update_t);

	// read nav data
    getline(fin, line);
  	istringstream input_istring(line);
 	input_istring >> timestamp >> vx >> vy >> vz >> pitch >> roll >> yaw >> altitude;

	pitch = pitch / 1000;
    roll = roll / 1000;
    yaw  = yaw / 1000;

	RotMatUpdate(pitch, roll, yaw, update_R);
	vpTranslationVector delta = update_t - t;
	t = update_t;
	update_t = update_t + delta;


	update_cMo.insert(update_R);
	update_cMo.insert(update_t);
	
	tracker2.modifyPose(update_cMo);
	//Waiting for key event
	vpDisplay::getKeyboardEvent(undistort_I);
  }
  
  fin.close();

  return 0;
}


void RotMatUpdate(float pitch, float roll, float yaw, vpRotationMatrix &update_R)
{
	vpRzyxVector rzyx23;
	rzyx23[0] = vpMath::rad( -90.5f);//-91.5
	rzyx23[1] = vpMath::rad( 15.5f);//14.5
	rzyx23[2] = vpMath::rad( -95.f);//-94
	vpRotationMatrix R23(rzyx23);

	vpRzyxVector rzyx34;
    rzyx34[0] = vpMath::rad(yaw);
	rzyx34[1] = vpMath::rad(pitch);
	rzyx34[2] = vpMath::rad(roll);
	vpRotationMatrix R34(rzyx34);

	vpRzyxVector rzyx45;
    rzyx45[0] = vpMath::rad( 90.f);
	rzyx45[1] = vpMath::rad( 0.f);
	rzyx45[2] = vpMath::rad( 90.f);
	vpRotationMatrix R45(rzyx45);

	vpRotationMatrix R;
	R = R23 * R34 * R45;
    R.inverse(update_R);
}
