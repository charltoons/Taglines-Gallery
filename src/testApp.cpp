#include "testApp.h"


//--------------------------------------------------------------
void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	kinect.init(false, false);
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	kinect.open();
	
	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	
	nearThreshold = 230;
	farThreshold = 100;
	bThreshWithOpenCV = true;
	
	ofSetFrameRate(60);
	
	// zero the tilt on startup
	angle = 18;
	kinect.setCameraTiltAngle(angle);
	
	// start from the front
	bDrawPointCloud = false;
    
    //Blob Area Threshold
    blobAreaThreshold = 10000;
    
    //some path, may be absolute or relative to bin/data
    string path = "images";
    ofDirectory dir(path);
    //only show png files
    //dir.allowExt("png");
    //populate the directory object
    dir.listDir();
    
    //go through the images folder and load all the images
    for(int i = 0; i < dir.numFiles(); i++){
        ofImage newPortrait;
        newPortrait.loadImage(dir.getPath(i));
        portraits.push_back(newPortrait);
    }
    
    showKinect = false;
}

//--------------------------------------------------------------
void testApp::update() {
	
	ofBackground(100, 100, 100);
	
	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
		
		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
		
		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
        grayThreshNear = grayImage;
        grayThreshFar = grayImage;
        grayThreshNear.threshold(nearThreshold, true);
        grayThreshFar.threshold(farThreshold);
        cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		
		// update the cv images
		grayImage.flagImageChanged();
		
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);
    
        //empty previous people
        people.clear();
    
        //add blobs who meet area threshold back to people vecotr
        for( int i = 0; i < contourFinder.blobs.size(); i++){
            if (contourFinder.blobs.at(i).area > blobAreaThreshold) people.push_back(contourFinder.blobs.at(i));
        }
    
        //we're gonna draw the blobs later, so just reset with people
        contourFinder.blobs = people;
	}
}

//--------------------------------------------------------------
void testApp::draw() {
	
	ofSetColor(255, 255, 255);
	if (showKinect){
        grayImage.draw(0, 0);
        contourFinder.draw(0, 0);
    }
	
	// draw instructions
//	ofSetColor(255, 255, 255);
//	stringstream reportStream;
//	reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
//	<< ofToString(kinect.getMksAccel().y, 2) << " / "
//	<< ofToString(kinect.getMksAccel().z, 2) << endl
//	<< "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
//	<< "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
//	<< "set near threshold " << nearThreshold << " (press: + -)" << endl
//	<< "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
//	<< ", fps: " << ofGetFrameRate() << endl
//	<< "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl
//	<< "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl;
//	ofDrawBitmapString(reportStream.str(),20,652);
    
    if (people.size() > 0) {
        portraits.at(0).draw(people.at(0).centroid.x,10, 100, 100);
    }
}

void testApp::drawPointCloud() {
	int w = 640;
	int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
				mesh.addColor(kinect.getColorAt(x,y));
				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
			}
		}
	}
	glPointSize(3);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards' 
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit
	glEnable(GL_DEPTH_TEST);
	mesh.drawVertices();
	glDisable(GL_DEPTH_TEST);
	ofPopMatrix();
}

//--------------------------------------------------------------
void testApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {
		case ' ':
//			bThreshWithOpenCV = !bThreshWithOpenCV;
			break;
			
		case'p':
//			bDrawPointCloud = !bDrawPointCloud;
			break;
			
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
            cout << "Far Thresh: " << farThreshold << endl;
			break;
			
		case '<':
		case ',':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
            cout << "Far Thresh: " << farThreshold << endl;
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
            cout << "Near Thresh: " << farThreshold << endl;
			break;
			
		case '-':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
            cout << "Near Thresh: " << farThreshold << endl;
			break;
			
		case 'w':
//			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
			
		case 'o':
//			kinect.setCameraTiltAngle(angle); // go back to prev tilt
//			kinect.open();
			break;
			
		case 'c':
//			kinect.setCameraTiltAngle(0); // zero the tilt
//			kinect.close();
			break;
        case 'x':
            showKinect = !showKinect;
            cout << "showKinect: " << showKinect << endl;
            break;
			
		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
            cout << "Kinect Angle: " << angle << endl;
			break;
			
		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
            cout << "Kinect Angle: " << angle << endl;
			break;
	}
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}
