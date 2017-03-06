#include "User.h"

namespace ofxKinectForWindows2 {

	bool User::setBody(kBody* bodyPtr) { // returns true if change to user
		
		if ((!bodyPtr && !_bodyPtr) || ( bodyPtr == _bodyPtr)) return false;
		
		_bodyPtr = bodyPtr;
		_startTime = bodyPtr ? ofGetElapsedTimef() : 0; // 0 if null body
		ofLogVerbose("ofxKFW2::User") << "set new body - ptr: " << (bodyPtr ? ofToString(bodyPtr) : "null");
		return true;
	}

	// update
	// ---------------------------------------------------------------------------

	bool User::update() {

		// save and clear joint positions
		_pJoints = _joints;
		_joints.clear();

		// save and clear hand states
		_pHandStates = _handStates;
		_handStates = HandStates();

		if (!hasBody()) {
			ofLogVerbose("ofxKFW2::User") << "can't update, no body";
			return false;
		}
		if (!hasCoordinateMapper()) {
			ofLogVerbose("ofxKFW2::User") << "can't update, no coordinate mapper";
			return false;
		}

		auto& joints = _bodyPtr->joints; // raw joints from kinect

		// calc new joint positions (world > color coords)
		for (auto& joint : joints) {

			// grab basic joint data

			ofVec3f& p3dRaw		= _joints[joint.first].pos3dRaw			= joint.second.getPosition();
			ofQuaternion& oRaw	= _joints[joint.first].orientationRaw	= joint.second.getOrientation();
			ofVec3f& p3d		= _joints[joint.first].pos3d;
			ofQuaternion& ori	= _joints[joint.first].orientation;
			ofVec2f& p2d		= _joints[joint.first].pos2d			= joint.second.getProjected(_coordMapperPtr);
								  _joints[joint.first].state			= joint.second.getTrackingState();

			// transform 

			// position
			
			p3d = p3dRaw * reflection * getGlobalTransformMatrix();

			// orientation

			ori = oRaw ;//* getGlobalOrientation();


			if (_bMirrorX) {
				// mirror 2d
				p2d.x = 1920 - p2d.x; // flip within color space

				//ofVec3f unitAxis(1,1,1);
				//unitAxis.normalize();

				//ofVec3f uAxisRaw = unitAxis * oRaw;
				//ofVec3f uAxisReflect = uAxisRaw * reflection;
				//ofQuaternion oRef;
				//oRef.makeRotate(uAxisRaw,uAxisReflect);

				//ori = ori * oRef;
			}
		}

		// get new hand states
		_handStates.left = _bodyPtr->leftHandState;
		_handStates.right = _bodyPtr->rightHandState;

		return true;
	}

	bool User::jointExists(JointType type, bool prev) {
		JointMap& jts = prev ? _pJoints : _joints;
		return jts.find(type) != jts.end();
	}

	ofVec2f User::getJoint2dPos(JointType type, bool prev) {
		ofVec2f pos;
		if (jointExists(type, prev))
			pos = (prev ? _pJoints : _joints)[type].pos2d;
		return pos;
	}

	ofVec3f User::getJoint3dPos(JointType type, bool prev) {
		ofVec3f pos;
		if (jointExists(type, prev))
			pos = (prev ? _pJoints : _joints)[type].pos3d;
		return pos;
	}

	ofVec3f User::getJoint3dPosRaw(JointType type, bool prev) {
		ofVec3f pos;
		if (jointExists(type, prev))
			pos = (prev ? _pJoints : _joints)[type].pos3dRaw;
		return pos;
	}

	ofQuaternion User::getJointOrientation(JointType type, bool prev) {
		ofQuaternion q;
		if (jointExists(type, prev))
			q = (prev ? _pJoints : _joints)[type].orientation;
		return q;
	}

	ofQuaternion User::getJointOrientationRaw(JointType type, bool prev) {
		ofQuaternion q;
		if (jointExists(type, prev))
			q = (prev ? _pJoints : _joints)[type].orientationRaw;
		return q;
	}

	TrackingState User::getTrackingState(JointType type, bool prev) {
		TrackingState state = TrackingState_NotTracked;
		if (jointExists(type, prev))
			state = (prev ? _pJoints : _joints)[type].state;
		return state;
	}

	const HandState User::getLeftHandState(bool prev) const {
		return prev ? _pHandStates.left : _handStates.left;
	}
	const HandState User::getRightHandState(bool prev) const {
		return prev ? _pHandStates.right : _handStates.right;
	}

	bool User::isRightHandUp() {
		if (!jointExists(JointType_HandRight) || !jointExists(JointType_Head)
			|| !hasBody())
			return false;
		if (getJoint2dPos(JointType_HandRight).y < getJoint2dPos(JointType_Head).y)
			return true;
		return false;
	}

	bool User::isLeftHandUp() {
		if (!jointExists(JointType_HandLeft) || !jointExists(JointType_Head)
			|| !hasBody())
			return false;
		if (getJoint2dPos(JointType_HandLeft).y < getJoint2dPos(JointType_Head).y)
			return true;
		return false;
	}

	ofVec3f User::getPosOnFloor(Kinect* kinect, bool world)
	{
		if (!hasBody() || !kinect) return ofVec3f();

		ofVec3f lFoot = _bodyPtr->joints.at(JointType_FootLeft).getPosition();
		ofVec3f rFoot = _bodyPtr->joints.at(JointType_FootRight).getPosition();
		ofVec3f cFoot = lFoot.getMiddle(rFoot);
		if (world) return kinect->getClosestPtOnFloor(cFoot);
		return kinect->getClosestPtOnFloorPlane(cFoot);
	}

	// draw
	// ---------------------------------------------------------------------------

	void User::draw(bool b3d, int alpha) {

		if (!hasBody()) {
			ofLogVerbose("ofxKFW2::User") << "can't draw, no body";
			return;
		}

		drawJoints(b3d, alpha);
		drawHandStates();
	}

	void User::drawJoints(bool b3d, int alpha) {

		if (!hasBody()) {
			ofLogVerbose("ofxKFW2::User") << "can't draw 3d joints, no body";
			return;
		}

		ofPushStyle();
		// draw joints
		for (auto& joint : _joints) {

			const auto& type			= joint.first;
			const auto& state			= joint.second.state;
			const ofVec3f pos			= b3d ? joint.second.pos3d : joint.second.pos2d;
			const auto& orient			= joint.second.orientation;

			if (state == TrackingState_NotTracked) continue; // don't draw if not tracked

			bool certain		= state != TrackingState_Inferred;
			int rad				= b3d ? (certain ? 2 : 4) : (certain ? 5 : 10);
			ofColor color		= certain ? ofColor(0,255,0,alpha) : ofColor(255, 255, 0, alpha);
			ofSetColor(color);
			ofDrawSphere(pos, rad);

			if (b3d) {
				ofMatrix4x4 r, t;
				r.makeRotationMatrix(orient);
				t.makeTranslationMatrix(pos);
				ofPushMatrix();
				ofMultMatrix(r*t);
				ofDrawAxis(certain ? 5 : 10);
				ofPopMatrix();
			}

		}
		ofPopStyle();
	}

	void User::drawHandStates() {
		if (!hasBody()) return;
		drawHandState(JointType_HandLeft);//, pos, width, height);
		drawHandState(JointType_HandRight);// , pos, width, height);
	}

	void User::drawHandState(JointType hand) {
		if (!hasBody()) return;
		HandState hState = HandState_Unknown; ofVec2f hPos(0, 0);

		// determine hand
		if (hand == JointType_HandLeft) hState = _handStates.left;
		else if (hand == JointType_HandRight) hState = _handStates.right;
		else {
			ofLogError("User::drawHandState") << "JointType parameter not a hand!";
			return;
		}
		// get pos
		if (jointExists(hand)) hPos = _joints[hand].pos2d;
		else return;

		// determine state
		ofColor col;
		switch (hState) {
			case HandState_NotTracked:
				return;
			case HandState_Unknown:
				col = ofColor(255, 0, 0, 80);
				break;
				//return;
			case HandState_Open:
				col = ofColor(0, 255, 0, 80);
				break;
			case HandState_Closed:
				col = ofColor(255, 255, 0, 80);
				break;
			case HandState_Lasso:
				col = ofColor(0, 255, 255, 80);
				break;
		}
		// draw
		int radius = 50;
		ofPushStyle();
		ofEnableAlphaBlending();
		ofSetColor(col);
		ofDrawCircle(hPos, radius);
		ofDisableAlphaBlending();
		ofPopStyle();
	}

	bool User::buildMesh(Kinect* kinect, int step, float facesMaxLength) {

		// get body id
		if (_bodyPtr == nullptr) {
			ofLogError("User::buildMesh") << "can't build mesh, no user / body!";
			return false;
		}
		const auto& bodyId = _bodyPtr->bodyId;
		if (bodyId < 0 || bodyId > 5) {
			ofLogError("User::buildMesh") << " can't build mesh, invalid bodyId: " << bodyId;
			return false;
		}

		// get kinect
		if (kinect == nullptr) {
			ofLogError("User::buildMesh") << "can't build mesh, kinect is null";
			return false;
		}
		// check coordinate mapper
		if (_coordMapperPtr == nullptr) {
			ofLogError("User::buildMesh") << "can't build mesh, no coordinate mapper";
			return false;
		}

		// depth, body index and color sources
		auto& depthPix = kinect->getDepthSource()->getPixels();
		auto& bodyIdxPix = kinect->getBodyIndexSource()->getPixels();
		auto& colorPix = kinect->getColorSource()->getPixels();
		if (!depthPix.size()) {
			ofLogError("User::buildMesh") << "can't build mesh, no depth pixels read";
			return false;
		}
		if (!bodyIdxPix.size()) {
			ofLogError("User::buildMesh") << "can't build mesh,  body index source not allocated";
			return false;
		}
		if (!colorPix.size()) {
			ofLogError("User::buildMesh") << "can't build mesh, color index source not allocated";
			return false;
		}

		// get color coords of depth pixels
		// maybe just do this once?
		_coordMapperPtr->MapDepthFrameToColorSpace(512 * 424, (UINT16*)depthPix.getPixels(), 512 * 424, (ColorSpacePoint*)_depthToColorCoords.data());

		// loop through body idx pix
		//
		// mesh triangle formatting:
		//  tl.____t.
		//    |\   /|
		//    |  X  |
		//  l.|/___\|
		//          *i
		//
		bool isBody[512 * 424]; // tracks body status in depth img

		// MESH generator

		// prep mesh
		_userMesh.clear();
		_userMesh.setMode(OF_PRIMITIVE_TRIANGLES);

		// add color img texture coords
		_userMesh.addTexCoords(_depthToColorCoords);

		// add depth->camera space vertices
		_userMesh.getVertices().resize(512 * 424);
		_coordMapperPtr->MapDepthFrameToCameraSpace(512 * 424, (UINT16*)depthPix.getPixels(), 512 * 424, (CameraSpacePoint*)_userMesh.getVerticesPointer());
		auto vertices = _userMesh.getVerticesPointer();

		// loop through body idx px, find body px, add indices to mesh
		for (int y = 0; y <= 424 - step; y += step) {
			for (int x = 0; x <= 512 - step; x += step) {

				// indices in body idx px
				int i = y * 512 + x;
				int t = i - 512 * step;
				int l = i - step;
				int tl = l - 512 * step;

				// check if body idx color is this body
				int val = roundf(bodyIdxPix[i]);
				if (val == bodyId) {
					isBody[i] = true; // part of the body
				}
				else isBody[i] = false;

				if (t < 0 || l < 0) continue; // other points outside frame, move on

				// check for triangles within body

				// camera space points
				const ofVec3f & v = vertices[i];
				const ofVec3f & vT = vertices[t];
				const ofVec3f & vL = vertices[l];
				const ofVec3f & vTL = vertices[tl];

				if (isBody[i] && !isBody[tl]) { // only one triangle option

					if (isBody[t] && isBody[l]) {
						// make triangle:
						//   /|
						// /__|

						// check distance
						if (v.z > 0 && vT.z > 0 && vL.z > 0
							&& abs(v.z - vT.z) < facesMaxLength
							&& abs(v.z - vL.z) < facesMaxLength) {
							const ofIndexType indices[3] = { i, t, l };
							_userMesh.addIndices(indices, 3);
						}
					}
				}
				else if (isBody[tl] && !isBody[i]) { // only one triangle option

					if (isBody[t] && isBody[l]) {
						// make triangle:
						// ____
						// |  /
						// |/

						// check distance
						if (vTL.z > 0 && vT.z > 0 && vL.z > 0
							&& abs(vTL.z - vT.z) < facesMaxLength
							&& abs(vTL.z - vL.z) < facesMaxLength) {
							const ofIndexType indices[3] = { tl, t, l };
							_userMesh.addIndices(indices, 3);
						}
					}
				}
				else { // both i and tl are body, try inverted triangles
					if (isBody[l]) {
						// make triangle:
						// |\
						// |__\

						// check distance
						if (v.z > 0 && vTL.z > 0 && vL.z > 0
							&& abs(v.z - vTL.z) < facesMaxLength
							&& abs(v.z - vL.z) < facesMaxLength) {
							const ofIndexType indices[3] = { i, tl, l };
							_userMesh.addIndices(indices, 3);
						}
					}
					if (isBody[t]) {
						// make triangle:
						// ____
						// \  |
						//   \|

						// check distance
						if (v.z > 0 && vTL.z > 0 && vT.z > 0
							&& abs(v.z - vTL.z) < facesMaxLength
							&& abs(v.z - vT.z) < facesMaxLength) {
							const ofIndexType indices[3] = { i, tl, t };
							_userMesh.addIndices(indices, 3);
						}
					}
				}
			}
		} // end loop through body idx pixels

		return true;
	}

	void User::drawMeshFaces() {
		_userMesh.drawFaces();
	}

	void User::drawMeshWireframe() {
		_userMesh.drawWireframe();
	}

	void User::clear() {
		_bodyPtr = nullptr;
		_joints.clear();
		_pJoints.clear();
		_handStates = HandStates();
		_pHandStates = HandStates();
	}

	ofMatrix4x4 User::reflectionMatrix(ofVec4f plane)
	{
		ofMatrix4x4 ref;

		ref(0,0) = (1. - 2. * plane[0] * plane[0]);
		ref(1,0) = (-2. * plane[0] * plane[1]);
		ref(2,0) = (-2. * plane[0] * plane[2]);
		ref(3,0) = (-2. * plane[3] * plane[0]);

		ref(0,1) = (-2. * plane[1] * plane[0]);
		ref(1,1) = (1. - 2. * plane[1] * plane[1]);
		ref(2,1) = (-2. * plane[1] * plane[2]);
		ref(3,1) = (-2. * plane[3] * plane[1]);

		ref(0,2) = (-2. * plane[2] * plane[0]);
		ref(1,2) = (-2. * plane[2] * plane[1]);
		ref(2,2) = (1. - 2. * plane[2] * plane[2]);
		ref(3,2) = (-2. * plane[3] * plane[2]);

		ref(0,3) = 0.;
		ref(1,3) = 0.;
		ref(2,3) = 0.;
		ref(3,3) = 1.;

		return ref;
	}

}

