#include "Kinect.h"

namespace ofxKinectForWindows2 {

	void Kinect::init(bool bColor, bool bBody, bool bDepth, bool bBodyIdx, bool bIR, bool bIRLong) {

		if (bColor) initColorSource();
		if (bBody) initBodySource();
		if (bDepth) initDepthSource();
		if (bBodyIdx) initBodyIndexSource();
		if (bIR) initInfraredSource();
		if (bIRLong) initLongExposureInfraredSource();

		if (getSensor()->get_CoordinateMapper(&_coordinateMapper) < 0) {
			ofLogError("Kinect") << "couldn't acquire coordinate mapper!";
		}
		else {
			ofLogVerbose("Kinect") << "acquired coordinate mapper";
		}
		setUseTextures(true); // not sure if necessary
	}

	kBody* Kinect::getBodyPtrByIndex(int bodyIndex) {

		auto& bodies = getBodySource()->getBodies();
		if (bodyIndex > 0 && bodyIndex < bodies.size()) {
			return &(bodies[bodyIndex]);
		}
		return nullptr;
	}

	int Kinect::getCentralBodyIndex(float bodyQualityThreshold) {

		auto & bodies = getBodySource()->getBodies();

		int closestIdx = -1;
		float closestDist = 0.7; // meters
		float zThresh = 3;
		for (int i = 0; i < bodies.size(); i++) {
			if (!bodies[i].tracked) continue; // skip untracked body
			float dist = abs(bodies[i].joints.at(JointType_SpineBase).getPosition().x);
			float z = abs(bodies[i].joints.at(JointType_SpineBase).getPosition().z);
			if (dist < closestDist && z < zThresh) {
				closestIdx = i;
				closestDist = dist;
			}
		}
		return closestIdx;
	}

	ofMatrix4x4 Kinect::getFloorTransform()
	{
		auto fcp = getFloorClipPlane();
		ofVec3f normal(fcp.x, fcp.y, fcp.z);
		float distance = fcp.w;

		ofMatrix4x4 rot, trans;

		// check for floor clip plane data
		if (normal != ofVec3f(0) || distance != 0) {
			rot.makeRotationMatrix(ofVec3f(0, 1, 0), ofVec3f(normal)); // rotation of plane
			trans.makeTranslationMatrix(normal*-distance); // origin of plane
		}

		return floorTransform = rot*trans;
	}

	ofVec3f Kinect::getClosestPtOnFloor(ofVec3f pos)
	{
		ofMatrix4x4 floor = getFloorTransform();
		ofVec3f o =			floor.getTranslation();	// floor origin
		ofVec3f n =			ofVec3f(0, 1, 0) * floor.getRotate();	// floor normal

		float floorDist = 
			ofVec3f(o.x - pos.x, o.y - pos.y, o.z - pos.z).dot(n);
			// distance to floor = dot(orig-pt, normal)

		return pos + n*floorDist;

		//ofVec2f pt;
		//
		//ofVec3f& p = worldPos;
		//Vector4 fcp = getFloorClipPlane();
		//ofVec3f fN = ofVec3f(fcp.x,fcp.y,fcp.z);
		//float d = fcp.w;

		//// pt = pos (x,y,z) + s * planeNormalVec
		//// find c

		//// pt = (x,y,z) + s * (pX,pY,pZ) ->
		//// pt = ((x + s*pX), (y + s*pY), (z + s*zY))
		//// using plane equation Ax+By+Cz+D = 0...
		//// pX * (x + c*pX) + pY * (y + c*zY) + pZ * (z + c*pZ) + D = 0

		//float dist = (p.x*fN.x + p.y*fN.y + p.z*fN.z) / (fN.x*fN.x + fN.y*fN.y + fN.z*fN.z);
		//pt = p - fN*dist;

		//return pt;
	}

	ofVec3f Kinect::getClosestPtOnFloorPlane(ofVec3f pos)
	{
		return getClosestPtOnFloor(pos) * getFloorTransform().getInverse();
	}

	ofVec2f Kinect::getClosestPtOnFloorPlaneXY(ofVec3f pos)
	{
		ofVec3f pt = getClosestPtOnFloor(pos) * getFloorTransform().getInverse();
		return ofVec2f(pt.x,pt.z);
	}

	const vector<Data::Body>& Kinect::getBodies()
	{
		return getBodySource()->getBodies();
	}

	vector<kBody*> Kinect::getTrackedBodies()
	{
		auto & bodies = getBodySource()->getBodies();
		vector<kBody*> tBodies;
		for (auto& body : bodies) {
			if (body.tracked) tBodies.push_back(&body);
		}
		return tBodies;
	}

	vector<kBody*> Kinect::getBodiesWithinBounds(ofRectangle floorBounds) {

		auto bodies = getTrackedBodies();
		map<float, kBody*> bodiesInByDist;

		for (auto body : bodies) {

			if (!body || !body->tracked) continue; // skip untracked body

			auto& spine = body->joints.at(JointType_SpineBase);
			if (spine.getTrackingState() == TrackingState_NotTracked) continue;

			ofVec3f floorPos = getClosestPtOnFloorPlane(spine.getPosition()); // x,z
			ofVec2f floorXY(floorPos.x, floorPos.z);

			if (floorBounds.inside(floorXY)) {
				bodiesInByDist[floorXY.y] = body;
			}
		}

		vector<kBody*> bodiesIn;
		for (auto body : bodiesInByDist) {
			bodiesIn.push_back(body.second);
		}
		return bodiesIn;
	}


	kBody* Kinect::getCentralBodyPtr(float threshold) {
		int bodyIdx = getCentralBodyIndex();
		if (bodyIdx < 0) return nullptr;
		return getBodyPtrByIndex(bodyIdx);
	}


	int Kinect::getNumTrackedBodies() {
		int n = 0;
		for (auto& body : getBodySource()->getBodies()) {
			if (body.tracked) n++;
		}
		return n;
	}

	void Kinect::drawColor(ofVec3f pos, float w, float h, bool vFlip, bool hFlip) {
		if (hFlip) {
			pos.x = pos.x + w;
			w *= -1;
		}
		if (vFlip) {
			_flipFbo.begin();
			getColorTexture().draw(0, 0, 1920, 1080);
			_flipFbo.end();
			_flipFbo.getTexture().draw(pos.x, pos.y, pos.z, w, h);
		}
		else
			getColorTexture().draw(pos.x, pos.y, pos.z, w, h);
	}

	void Kinect::drawColorSubsection(float x, float y, float w, float h,
									 float sx, float sy, float sw, float sh) {

		getColorTexture().drawSubsection(x, y, w, h, sx, sy, sw, sh);
	}
	void Kinect::drawFloorBounds(ofRectangle floorBounds)
	{
		ofPushMatrix();

		ofMultMatrix(getFloorTransform());
		ofRotate(90, 1, 0, 0); // x-y coords to x-z

		ofDrawRectangle(floorBounds);

		ofPopMatrix();
	}
		//ofVec3f tl, tr, bl, br;
		//ofMatrix4x4 m = getFloorTransform().getInverse();
		//ofRectangle& b = floorBounds;
		//tl = b.getTopLeft() * m;
		//tr = b.getTopRight() * m;
		//bl = b.getBottomLeft() * m;
		//br = b.getBottomRight() * m;

		//ofDrawLine(tl,tr); ofDrawLine(bl, br); ofDrawLine(tl,bl); ofDrawLine(tr,br);
		//ofMatrix4x4 m = getFloorTransform();
		//ofPushMatrix();
		//ofMultMatrix(m);
		//ofPushStyle();
		//ofSetColor(255,255,0,150);
		//ofDrawRectangle(floorBounds);
		//ofPopStyle();
		//ofPopMatrix();

}
