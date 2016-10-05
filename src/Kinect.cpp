#include "Kinect.h"

void Kinect::init(bool bColor, bool bBody, bool bDepth, bool bBodyIdx, bool bIR, bool bIRLong) {

	if (bColor) initColorSource();
	if (bBody) initBodySource();
	if (bDepth) initDepthSource();
	if (bBodyIdx) initBodyIndexSource();
	if (bIR) initInfraredSource();
	if (bIRLong) initLongExposureInfraredSource();

	if (getSensor()->get_CoordinateMapper(&_coordinateMapper) < 0) {
		ofLogError("Kinect") << "couldn't aquire coordinate mapper!";
	}
	else {
		ofLogVerbose("Kinect") << "aquired coordinate mapper";
	}
	setUseTextures(true); // not sure if necessary
}

Kinect::kBody* Kinect::getBodyPtrByIndex(int bodyIndex) {

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

Kinect::kBody* Kinect::getCentralBodyPtr(float threshold) {
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
