// #include "projector/UTM.h"

// using namespace usrg_utm;

// UtmProjector::UtmProjector(sensor_msgs::NavSatFix& origin) : m_isInNorthernHemisphere(false), m_origin(origin)
// {
//   double x = 0;
//   double y = 0;
//   GeographicLib::UTMUPS::Forward(m_origin.latitude, m_origin.longitude, m_zone,
//                                  m_isInNorthernHemisphere, x, y);

// }

// geometry_msgs::Pose2D UtmProjector::forward(const sensor_msgs::NavSatFix& gps) const {

//   try {
//     GeographicLib::UTMUPS::Forward(gps.latitude, gps.longitude, zone, northp, m_utmXY.x, m_utmXY.y);
//   } catch (GeographicLib::GeographicErr& e) {
//     ROS_WARN("GeographicLib FORWARD ERROR!");
//   }

//   if (zone != zone_ || northp != isInNorthernHemisphere_) {
//     // try to transfer to the desired zone
//     double xAfterTransfer = 0;
//     double yAfterTransfer = 0;
//     int zoneAfterTransfer = 0;
//     try {
//       GeographicLib::UTMUPS::Transfer(zone, northp, utm.x, utm.y, zone_, isInNorthernHemisphere_, xAfterTransfer,
//                                       yAfterTransfer, zoneAfterTransfer);
//     } catch (GeographicLib::GeographicErr& e) {
//       ROS_WARN("UTM TRANSFER ERROR");
//     }

//     if (zoneAfterTransfer != zone_) {
//       ROS_WARN("You have left the padding area of the UTM zone!");
//     }
//     utm.x = xAfterTransfer;
//     utm.y = yAfterTransfer;
//   }

//   if (useOffset_) {
//     utm.x -= xOffset_;
//     utm.y -= yOffset_;
//   }

//   return utm;
// }

// GpsData UtmProjector::reverse(const UtmData& utm) const {
//   GpsData gps{0., 0., utm.alt};
//   try {
//     GeographicLib::UTMUPS::Reverse(zone_, isInNorthernHemisphere_, useOffset_ ? utm.x + xOffset_ : utm.x,
//                                    useOffset_ ? utm.y + yOffset_ : utm.y, gps.lat, gps.lon);
//   } catch (GeographicLib::GeographicErr& e) {
//     ROS_WARN("UTM Reverse Error");
//   }

//   if (throwInPaddingArea_) {
//     // for zone compliance testing:
//     try {
//       forward(gps);
//     } catch (GeographicLib::GeographicErr& e) {
//       ROS_WARN("UTM Forward Error");
//     };
//   }
//   return gps;
// }
