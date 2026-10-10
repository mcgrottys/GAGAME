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
// THE LAT/LON SPELLING, ONE FUNCTION (it stood in three places): with lookAt, stand at the site and
// aim at a second place; else the SITE MOTOR CHAIN, said in the camera's own axes (forward +x, up
// +y, right -z) --
//     M = T(site) R_site R_heading R_tilt R_roll T(-range along the forward)
// R_site turns the forward onto the site's down and the own up onto its north; heading turns about
// the site's up (clockwise from north, as a compass), tilt about the own right (0 looks at the
// planet's centre, 90 at the horizon -- KML's tilt), roll about the forward, and range stands the
// eye back along it, so (lat, lon, alt) is the point looked at. All zero: at the site, looking at
// the centre, north up the screen. Planet frame; PlanetToFlatPose takes it into the session's.
Camera LatLonPose(double lat, double lon, double altM, bool lookAt, double tLat, double tLon,
                  double headingDeg, double tiltDeg, double rollDeg, double rangeM, double planetR);
// orbPose: stand at (lat, lon, altM) on a planet of radius planetR, aim at a surface target.
Camera OrbitPose(double lat, double lon, double altM, double tLat, double tLon, double planetR);
// The globe start camera: at (lat, lon, altM), aimed at the planet's centre (--globe-cam): the
// site chain with every angle zero, north up the screen.
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
