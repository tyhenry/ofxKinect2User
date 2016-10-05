#pragma once
#include "ofMain.h"
#include "ofxKinectForWindows2.h"
#include "Kinect.h"

class User {
public:
	struct HandStates {
		HandState left = HandState_Unknown;
		HandState right = HandState_Unknown;
	};
	struct JointData {
		ofVec2f pos2d = ofVec2f(0, 0);
		ofVec3f pos3d = ofVec3f(0, 0, 0);
		TrackingState state = TrackingState_NotTracked;
	};

	typedef map<JointType, JointData> JointMap;
	typedef const ofxKFW2::Data::Body kBody;

	User(ICoordinateMapper* coordinateMapperPtr = nullptr)
		: _coordMapperPtr(coordinateMapperPtr) {
		// for mesh generator:
		_depthToColorCoords.resize(512 * 424); // depth size
		_bodyImg.allocate(512, 424, OF_IMAGE_COLOR);
	}

	void setCoordinateMapper(ICoordinateMapper* coordinateMapperPtr)
	{
		_coordMapperPtr = coordinateMapperPtr;
	}

	void setWorldScale(ofVec3f scale) { _worldScale = scale; }
	void setWorldScale(float scale) { setWorldScale(ofVec3f(scale)); }
	const ofVec3f getWorldScale() const { return _worldScale; }
	void setWorldTranslate(ofVec3f translate) { _worldTranslate = translate; }
	const ofVec3f getWorldTranslate() const { return _worldTranslate; }
	void setMirrorX(bool mirror) { _bMirrorX = mirror; }
	const bool getMirrorX() const { return _bMirrorX; }

	bool setBody(kBody* body); // returns false if same body
	bool update(); // false is _bodyPtr is nullptr

	bool jointExists(JointType type, bool prev = false);
	ofVec2f getJoint2dPos(JointType type, bool prev=false);
	ofVec3f getJoint3dPos(JointType type, bool prev = false); // in color space
	TrackingState getTrackingState(JointType type, bool prev=false);
	const HandState getLeftHandState(bool prev = false) const;
	const HandState getRightHandState(bool prev = false) const;
	bool isRightHandUp();
	bool isLeftHandUp();
	const bool hasBody() const { return (_bodyPtr != nullptr); }
	bool hasCoordinateMapper() { return (_coordMapperPtr != nullptr); }
	const JointMap getJointMap() const { return _joints; }

	void draw();
	void drawJoints2d();
	void drawJoints3d();
	void drawHandStates();
	void drawHandState(JointType hand);

	ofMesh& buildMesh(Kinect* kinect);
	// extracts body shape on color img from kinect
	// uses color->world coords to generate mesh, transformed by worldScale & worldTranslate
	
	void clear();

	kBody* getBodyPtr() { return _bodyPtr; };

protected:

	kBody* _bodyPtr = nullptr;
	ICoordinateMapper* _coordMapperPtr;
	ofVec3f _worldScale = ofVec3f(1, 1, 1);
	ofVec3f _worldTranslate = ofVec3f(0, 0, 0);
	bool _bMirrorX = false;

	JointMap _joints; // joints positions in 2d & 3d
	JointMap _pJoints; // previous frame
	HandStates _handStates; // left, right
	HandStates _pHandStates; // previous frame

	ofMesh _userMesh;
	vector<ofVec2f> _depthToColorCoords;
	ofImage _bodyImg;
};
