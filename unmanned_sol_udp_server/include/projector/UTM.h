// #include <GeographicLib/UTMUPS.hpp>
// #include <ros/ros.h>
// #include <sensor_msgs/NavSatFix.h>
// #include <geometry_msgs/Pose2D.h>

// namespace usrg_utm {

// class UtmProjector 
// {
//  public:
//   UtmProjector(sensor_msgs::NavSatFix& origin);
//   geometry_msgs::Pose2D forward(const sensor_msgs::NavSatFix& gps) const;
//   sensor_msgs::NavSatFix reverse(const geometry_msgs::Pose2D& utm) const;

//   sensor_msgs::NavSatFix m_origin;
//   sensor_msgs::NavSatFix m_GpsRaw;
//   geometry_msgs::Pose2D m_utmXY;

//  private:
//   int m_zone;
//   bool m_isInNorthernHemisphere;
// };

// } //usrg_utm 