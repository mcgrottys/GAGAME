// ================================================================================================
//  Pose - M12 step 5a: THE SESSION'S POSE MAPS AS PURE FUNCTIONS. FrameLoop::Session() used to
//  hold these as lambdas (poseMotor, motorPose, orbPose, planetToFlatPose, flatToPlanetPose and
//  the globe start camera's four lines); the lambdas now delegate here, bodies VERBATIM, so
//  the scene's placement sugar (scene/Props.h) can be pinned against the code path it names --
//  {x, alt, z, az, pitch} is SetFromCompass then FromCamera, {lat, lon, alt, lookAt} is the
//  orbit key -- by scenetest calling the same functions the session calls.
//
//  Step 5b's View folds FromCamera/ToCamera into its own ToCamera()/FromCamera() (the plan's
//  "motorPose moved verbatim"), and Level(pose, up) arrives beside them; nothing here changes
//  meaning. The tangent rows a pose map takes are the SurfaceFrame's (east, up, north), which
//  the session derives from the anchor -- PoseFrame::FromAnchor is that derivation for a
//  caller with no session (the test), the same eight lines.
// ================================================================================================
#pragma once

#include "core/Pga.h"
#include "scene/Props.h"

namespace ga {
class Camera;
}

namespace ga::scene {

// poseMotor: a camera's pose as ONE motor (position and aim together).
Motor FromCamera(const Camera& c);
// motorPose: the motor back into a camera; any interpolated roll is dropped (horizon level).
void ToCamera(const Motor& m, Camera& c);
// orbPose: stand at (lat, lon, altM) on a planet of radius planetR, aim at a surface target.
Camera OrbitPose(double lat, double lon, double altM, double tLat, double tLon, double planetR);
// The globe start camera: at (lat, lon, altM), aimed at the planet's centre (--globe-cam).
Camera GlobeCamera(double lat, double lon, double altM, double planetR);
// The one-frame conversions: a planet-frame pose into the tangent (flat) frame and back.
Camera PlanetToFlatPose(const Camera& g, const double east0[3], const double up0[3],
                        const double north0[3], double planetR);
Camera FlatToPlanetPose(const Camera& f, const double east0[3], const double up0[3],
                        const double north0[3], double planetR);

// The tangent rows at an anchor (the session's derivation: east = d(dir)/dlon, north = east x
// up), as a PoseFrame. Returns an invalid frame when the triple is not a proper rotation.
PoseFrame FrameFromAnchor(double latDeg, double lonDeg, double planetR);

}  // namespace ga::scene
