#include "User.h"

bool User::setBody(kBody* bodyPtr) { // returns true if new body
	if (_bodyPtr != bodyPtr) {
		_bodyPtr = bodyPtr; // set
		return true;
	}
	return false;
}

bool User::update() {
	if (!hasBody()) return false;
	if (!hasCoordinateMapper()) return false;

	auto& joints = _bodyPtr->joints;

	// save previous joint positions
	_pJoints = _joints;
	// calc new joint positions (world > color coords)
	_joints.clear();
	for (auto& joint : joints) {
		ofVec2f& p2d = _joints[joint.second.getType()].pos2d = ofVec2f();
		ofVec3f& p3d = _joints[joint.second.getType()].pos3d = joint.second.getPosition();
		p3d += _worldTranslate;
		p3d *= _worldScale;

		TrackingState& state = _joints[joint.second.getType()].state
			= joint.second.getTrackingState();

		if (state == TrackingState_NotTracked) continue;

		p2d.set(joint.second.getProjected(_coordMapperPtr)); // world to color img

		if (_bMirrorX) {
			// mirror 2d
			p2d.x = 1920 - p2d.x; // within color space
			// mirror 3d
			p3d.x *= -1; // flip across y axis
		}
		
	}
	// save previous hand states
	_pHandStates = _handStates;
	// get new hand states
	_handStates.left = _bodyPtr->leftHandState;
	_handStates.right = _bodyPtr->rightHandState;
	return true;
}

bool User::jointExists(JointType type, bool prev) {
	if (prev) return (_pJoints.find(type) != _pJoints.end());
	return (_joints.find(type) != _joints.end());
}

ofVec2f User::getJoint2dPos(JointType type, bool prev) {
	ofVec2f pos(0, 0);
	if (prev && jointExists(type,true)) pos = _pJoints[type].pos2d;
	else if (!prev && jointExists(type)) pos = _joints[type].pos2d;
	return pos;
}
ofVec3f User::getJoint3dPos(JointType type, bool prev) {
	ofVec3f pos(0, 0);
	if (prev && jointExists(type, true)) pos = _pJoints[type].pos3d;
	else if (!prev && jointExists(type)) pos = _joints[type].pos2d;
	return pos;
}
TrackingState User::getTrackingState(JointType type, bool prev) {
	TrackingState state = TrackingState_NotTracked;
	if (prev && jointExists(type, true)) state = _pJoints[type].state;
	else if (!prev && jointExists(type)) state = _joints[type].state;
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

void User::draw() {
	if (!hasBody()) return;
	drawJoints2d();
	drawHandStates();
}

void User::drawJoints3d() {
	if (!hasBody()) return;

	ofPushStyle();
	// draw joints
	for (auto& joint : _joints) {

		ofVec3f& jPos3d = joint.second.pos3d;
		TrackingState& jState = joint.second.state;

		if (jState == TrackingState_NotTracked) continue;

		int rad = jState == TrackingState_Inferred ? 3 : 5;
		ofSetColor(ofColor::green);
		ofDrawSphere(jPos3d, rad);
	}
	ofPopStyle();
}

void User::drawJoints2d() {
	if (!hasBody()) return;
	// draw joints
	for (auto& joint : _joints) {

		const JointType& jType = joint.first;
		ofVec2f& jPos = joint.second.pos2d;
		TrackingState& jState = joint.second.state;

		if (jState == TrackingState_NotTracked) continue;

		int rad = jState == TrackingState_Inferred ? 2 : 8;
		ofPushStyle();
		ofSetColor(ofColor::green);
		ofDrawCircle(jPos, rad);
		ofPopStyle();
	}
}

void User::drawHandStates() {
	if (!hasBody()) return;
	drawHandState(JointType_HandLeft);//, pos, width, height);
	drawHandState(JointType_HandRight);// , pos, width, height);
}

void User::drawHandState(JointType hand) {
	if (!hasBody()) return;
	HandState hState = HandState_Unknown; ofVec2f hPos(0,0);

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
		col = ofColor(255,0,0,80);
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
	_coordMapperPtr->MapDepthFrameToColorSpace(512*424, (UINT16*)depthPix.getPixels(), 512*424, (ColorSpacePoint*)_depthToColorCoords.data());

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
	for (int y = 0; y <= 424-step; y+=step) {
		for (int x = 0; x <= 512-step; x+=step) {

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
