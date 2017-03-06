#pragma once
#include "ofMain.h"
#include "ofxKinectForWindows2.h"
#include "Kinect.h"

namespace ofxKinectForWindows2 {

	class User : public ofNode {
	public:
		struct HandStates {
			HandState left = HandState_Unknown;
			HandState right = HandState_Unknown;
		};
		struct JointData {
			ofVec2f pos2d;				// color coords (can be reflected around x)
			ofVec2f& posRgb = pos2d;
			ofVec3f pos3d;				// transformed by node
			ofVec3f pos3dRaw;			// kinect raw
			ofQuaternion orientation;	
			ofQuaternion orientationRaw;
			JointData* parent = nullptr;
			TrackingState state = TrackingState_NotTracked;
		};

		typedef map<JointType, JointData> JointMap;
		typedef const ofxKFW2::Data::Body kBody;

		User(ICoordinateMapper* coordinateMapperPtr = nullptr)
			: _coordMapperPtr(coordinateMapperPtr) 
		{
			// for mesh generator:
			_depthToColorCoords.resize(512 * 424); // depth size
			_depthToCameraCoords.resize(512 * 412);

			// make x reflection matrix
			ofVec4f yzPlane(1,0,0,0);
			reflectionX = reflectionMatrix(yzPlane);
		}
		//User(Kinect& kinect) { User(kinect.getCoordinateMapper()); }

		bool setCoordinateMapper(ICoordinateMapper* coordinateMapperPtr) {
			_coordMapperPtr = coordinateMapperPtr;
			return _coordMapperPtr;
		}
		bool setKinect(Kinect& kinect) {
			return setCoordinateMapper(kinect.getCoordinateMapper());
		}

		void setWorldScale(ofVec3f scale)			{ setScale(scale); }
		void setWorldScale(float scale)				{ setScale(scale); }
		void setWorldTranslate(ofVec3f translate)	{ setPosition(translate); }
		const ofVec3f getWorldScale() const			{ return getScale(); }
		const ofVec3f getWorldTranslate() const		{ return getPosition(); }

		//void setWorldScale(ofVec3f scale) { _worldScale = scale; }
		//void setWorldScale(float scale) { setWorldScale(ofVec3f(scale)); }
		//const ofVec3f getWorldScale() const { return _worldScale; }
		//void setWorldTranslate(ofVec3f translate) { _worldTranslate = translate; }
		//const ofVec3f getWorldTranslate() const { return _worldTranslate; }

		void setReflection(ofVec4f plane)	{ reflection = reflectionMatrix(plane); _bMirrorX = false; }
		ofMatrix4x4 getReflection()			{ return reflection; }

		void setMirrorX(bool mirror)	{ reflection = mirror ? reflectionX : ofMatrix4x4(); _bMirrorX = mirror; }
		const bool getMirrorX() const	{ return _bMirrorX; }

		bool setBody(kBody* body);	// returns true if change to user
		bool update(); // false if _bodyPtr is nullptr

		bool jointExists(JointType type, bool prev = false);
		ofVec2f getJoint2dPos(JointType type, bool prev = false);
		ofVec3f getJoint3dPos(JointType type, bool prev = false); // in color space
		ofVec3f getJoint3dPosRaw(JointType type, bool prev = false);
		ofQuaternion getJointOrientation(JointType type, bool prev = false);
		ofQuaternion getJointOrientationRaw(JointType type, bool prev = false);
		const JointMap getJointMap() const { return _joints; }

		TrackingState getTrackingState(JointType type, bool prev = false);
		const HandState getLeftHandState(bool prev = false) const;
		const HandState getRightHandState(bool prev = false) const;
		bool isRightHandUp();
		bool isLeftHandUp();

		ofVec3f getPosOnFloor(Kinect* kinect, bool world=true);
		ofVec2f getPosOnFloorPlane(Kinect* kinect)	{ return getPosOnFloor(kinect,false); }

		const bool hasBody() const { return (_bodyPtr != nullptr); }
		bool hasCoordinateMapper() { return (_coordMapperPtr != nullptr); }

		void draw(bool b3d = false, int alpha = 255);
		void drawJoints(bool b3d = false, int alpha = 255);
		void drawJoints2d(int alpha = 255) { drawJoints(false, alpha); }
		void drawJoints3d(int alpha = 255) { drawJoints(true, alpha); }
		void drawHandStates();
		void drawHandState(JointType hand);

		bool buildMesh(Kinect* kinect, int step = 1, float facesMaxLength = 0.1);
		void drawMeshWireframe();
		void drawMeshFaces();
		// extracts body shape on color img from kinect
		// uses color->world coords to generate mesh, transformed by worldScale & worldTranslate

		void clear();

		kBody* getBodyPtr() { return _bodyPtr; };
		float getUserTime() { return (_startTime > 0 ? ofGetElapsedTimef() - _startTime : 0); }
		float getUserStartTime() { return _startTime; }	// 0 if no body

	protected:

		kBody* _bodyPtr = nullptr;
		ICoordinateMapper* _coordMapperPtr;
		ofVec3f _worldScale = ofVec3f(1, 1, 1);
		ofVec3f _worldTranslate = ofVec3f(0, 0, 0);
		bool _bMirrorX = false;

		ofMatrix4x4 reflection;
		
		float _startTime = 0; // time when new user init'ed

		JointMap _joints; // joints positions in 2d & 3d
		JointMap _pJoints; // previous frame
		HandStates _handStates; // left, right
		HandStates _pHandStates; // previous frame

		ofMesh _userMesh;
		vector<ofVec2f> _depthToColorCoords;
		vector<ofVec3f> _depthToCameraCoords;

		ofMatrix4x4 reflectionMatrix(ofVec4f plane);

	private:

		ofMatrix4x4 reflectionX;

	};

}
