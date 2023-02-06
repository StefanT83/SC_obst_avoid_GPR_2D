/* Obstacle avoidance shared control algo for usage on teleoperated robots
Inputs:
- lidar: Velodyne VLP16: distance data corresp to selected rings
- joypad: Sony DS4: right joystick data
- odometry: linear velocity, angular velocity

Outputs:
- "virtual" joypad: shared control algorithm .output

Tuning parameters:
- phiUbLmisalign
- distSensorToEdgeOfRobot
- d0, dmax

 Sys coordinates conventions:
 - frame rviz [moving frame attached to sensor]: how data is stored in topic /velodyne_points, and wrt which we visualize data in rviz (the red-green-blue axes)
 - frame vld [moving frame attached to sensor]: obtained by rotating rviz by 90 deg, see drawing below
         xUrvizLobst == yUvldLobst   ^    an obstacle situated to the right of the sensor
                                     |   /
                                     | /
                  yUrvizLobst <------+-------> xUevldLobst
- frame b [moving frame attached to sensor]: obtained by rotating frame rviz on the same plane as follows: summing 90 deg (thus obtaining frame vld) + phiUbLmisalign, where phiUbLmisalign is the angle between xUrviz and yUb, expressed wrt/in-coord-of frame b

Tested on Ubuntu 20.04.1 + ROS Noetic

Code resources:
 https://github.com/ros-drivers/velodyne/blob/master/velodyne_laserscan/src/
 https://github.com/StefanT83/ASC_policy_iter/blob/main/ROS/src/shared_control_policy_iter_PMspeed5_node.cpp

 https://wiki.ros.org/pcl/Overview
 https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html
 https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html
*/

#include <ros/ros.h>

#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>

#include <vector>

// a point on an obstacle identified by velodyne
struct PointObst {
   double xUbLobst, yUbLobst, distUbLobst, angleUbLobst, intensity;
}; //struct PointObst

class SharedControl_velodyne {
private:
   // // Part 1 - velodyne config: notations largely inspired by https://github.com/ros-drivers/velodyne/blob/master/velodyne_laserscan/src/velodyne_laserscan.cpp
   static constexpr float RESOLUTION {0.007}; //[rad] angle increments for storing ring data; value cf https://github.com/ros-drivers/velodyne/blob/master/velodyne_laserscan/cfg/VelodyneLaserScan.cfg , also verified with a running VLP-16 by adding code ROS_INFO("%f",RESOLUTION) inside velodyne_laserscan.cpp
   static const size_t SIZE = 2.0*M_PI/RESOLUTION; //[-] narrowing conversion (loss of data); number of bins i.e. data points corresp to one data ring; size_t is "long unsigned int" namely "%lu" specifier in printf() and ROS_INFO()
   
   // Choose the rings to be used for obstacle detection, among the 16 rings of the VLP-16: ring \in [0,15]; ring=0 corresponds to the bottom lowest ring (below the horizontal plane of the sensor) assoc to angle=-15[deg]; ring=15 corresponds to the upper highest ring (above the horizontal plane of the sensor) assoc to angle=+15[deg] ;; VLP-16, the notation 'ring' is used in https://github.com/ros-drivers/velodyne/blob/master/velodyne_laserscan/src/velodyne_laserscan.cpp ;; see exp collected data in dataCollected_IMG_20220923_230809.jpg.txt
   const std::vector<uint16_t> vec_ring {7,8}; // 
   
   // Details of the topic /velodyne_points: it consists of sensor_msgs/PointCloud2 packets (cf $rostopic info /velodyne_points). 
   // Example of one packet's content (cf $ rostopic echo /velodyne_points |head -n 44):
   //  .height     = 1 [-] means cloud data is unordered cf http://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html
   //  .width      = 28910 [points] represents how many points are stored inside the 'data' field 
   //  .point_step = 22 [bytes] represents how many successive bytes form one point inside the 'data' field 
   //  .row_step   = 636020 [bytes] totalising one reading (theree are 10 readings every second \nu=10 Hz);; 636020 =reprez= 28910 x 22  ;; row_step := width x point_step ;; row_step seems to vary from one measurement to another, possibly because failed readings are not included in the message 
   //  .fields     := [x y z intensity ring time] forms one 3D point, in total point_step=22 [bytes]  4 bytes (offset=0, FLOAT32 i.e. datatype=7) for "x" + 4 bytes (offset=4, FLOAT32) for "y" + 4 bytes (offset=4, FLOAT32) for "z" + 4 bytes (offset=12, FLOAT32) for "intensity" + 2 bytes (offset=16, UINT16 i.e. datatype=4) for "ring" + 4 bytes (offset=18, FLOAT32) for "time"
   //
   static constexpr int offset_x         {0}; //[-] byte number in the succession [x y z intensity ring time]
   static constexpr int offset_y         {4}; //[-] idem above
   static constexpr int offset_z         {8}; //[-] idem above
   static constexpr int offset_intensity {12}; //[-] idem above
   static constexpr int offset_ring      {16}; //[-] idem above

   // // Part 2: physical qtts
          const double SD2_absvmax_fwd  {0.4}; //[m/s] robot maximum linear velocity when advancing forward
          const double SD2_absvmax_bwd  {-SD2_absvmax_fwd}; //[m/s] to-do: check experimentally the value; robot maximum linear velocity when advancing backwards
          const double SD2_chassisWidth {inch2m(20.0)}; //[m] chassisWidth = 20 inch cf 'PDF Drawing' on https://www.superdroidrobots.com/robots/prebuilt-robots/sold-custom-robots/product=2815
          const double SD2_omegamax     {2.0*SD2_absvmax_fwd/SD2_chassisWidth}; // [rad/s] used formula angle*radius=length, with radius = SD2_chassisWidth/2, then differentiate; see also whc_parameters1.m ;; symmetrical max angular velocity meaning there is no distinction between max left and max right 
          const double SD2_distSensorToEdgeOfRobot {.20}; //[m] dist from velodyne sensor to the fwd edge of the chassis width

   // choose/define accordingly
   static constexpr double sigma_omega_nz   {0.30};  //[-] td: change value to 0.03 after implementing sharedControlAlgo() > Case2
   static constexpr double thresh_omegad_nz {0.05};  //[-] threshold beyond which SC is NOT enabled

          const double phiUbLmisalign {deg2rad(0.0)}; //[rad] see def above, this angle captures the fact that the velodyne sensor axis pointing fwd (namely xUrviz == yUvld) is not perfectly parallel to the lenght of the SD2 robot

   // // Part 3 - data members
   double vd_nz, omegad_nz; //normalized values
   double v, omega; //odometry: v [m/s]; omega [rad/s]

   // define rings of laser data
   std::vector<std::vector<PointObst>> scan; // data on multiple rings of the 3D lidar: scan[idx_vec_ring][bin]

   // ROS
   ros::NodeHandle nh_; // in C++ the naming convention of a variable with an underscore usually indicates a private member variable: see B2017Newman, Comment p55
   ros::Subscriber velodyne_sub_, joypad_sub_, odom_sub_;
   ros::Publisher  joypad_SC_pub_;

   // create a temporary 'empty' PointObst
   PointObst temp_onePointObst {nan(""),nan(""),nan(""),nan(""),nan("")};

      // member function
   void joypadSubscCallback(const sensor_msgs::JoyConstPtr& msg) {
     // using axis convention (v_joy,omega_joy) from article [TeZhCa2020] i.e. art_whc2
      double v_joy = msg->axes[4]; //[-] \in [-1,1] the 5th element inside array axes represents advancing bwd/fwd user intention (max fwd = +1; max bwd = -1)
      double omega_joy = msg->axes[3]; // \in [-1,1]  the 4th element inside array axes represents turning right/left user intention (max left = +1; max right=-1)

      // conclude
      vd_nz     = v_joy; //[-]
      omegad_nz = omega_joy; //[-]

      // V&V
      ROS_INFO("joypad data: %f -- %f",vd_nz,omegad_nz);
   } //void velodyneSubscCallback(.)

   // member function
   void odometrySubscCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    v = msg->twist.twist.linear.x;  //[m/s]
    omega = msg->twist.twist.angular.z; //[rad/s]
    //ROS_INFO("Raw     : Lin velo: v = %.2f [m/s]; Angular velo: omega = %.2f [rad/s]",v,omega); //V&V

    // postprocess: saturate: safety reasons (data consistency)
    v = std::min(SD2_absvmax_fwd, std::max(SD2_absvmax_bwd, v)); //overwrite
    omega = std::min(SD2_omegamax, std::max(-SD2_omegamax, omega)); //overwrite
    //ROS_INFO("Postproc: Lin velo: v = %.2f [m/s]; Angular velo: omega = %.2f [rad/s]",v,omega); //V&V
   } //void odometrySubscCallback(.)

   // member function
   void velodyneSubscCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
     // // Step. disregard previous measurements: reset all scan[id_vec_ring][bin] to NaN
     for (std::vector<PointObst>& scan_elem:scan) std::fill(scan_elem.begin(), scan_elem.end(), temp_onePointObst); //note the reference symbol &, otherwise it won't work

     // // Step. main
     ROS_INFO_ONCE("VelodyneLaserScan: Extracting ring %u", vec_ring[0]);

     const size_t id_xUrvizLobst = 0; //=offset_x/4
     const size_t id_yUrvizLobst = 1; //=offset_y/4
     const size_t id_intensity = offset_intensity/4; // 4 =represents= sizeof(float)
     const size_t id_ring = offset_ring/4;           // 4 =represents= sizeof(float)

     for (sensor_msgs::PointCloud2ConstIterator<float> it(*msg, "x"); it != it.end(); ++it)	{
       const uint16_t ring_act = *((const uint16_t*)(&it[id_ring]));  // actual ring

       //V&V
       //std::cout << "ring_act=" << ring_act << '\n';
       //sleep(.01);

       for (size_t id_vec_ring{0}; id_vec_ring < vec_ring.size(); ++id_vec_ring)
       if (ring_act == vec_ring[id_vec_ring]) {
         const float xUrvizLobst = it[id_xUrvizLobst];  // Axes convention (viewed from above): xUrvizLobst is fwd (red in rviz), yUrvizLobst is towards left (green in rviz), zUbLobst is upwards (green in rviz); this coord sys is different than the one in the manual [VLP-16-User-Manual.pdf, \S 9, p54]
         const float yUrvizLobst = it[id_yUrvizLobst];  // see axes convention above

         // conseq ;; Note: putting the data in the format with bin as index is suitable for laserscan representation (a uniformly spaced grid): more generally, for our algorithm this is not required and other ways of representing the data can be thought of (e.g. a non-uniform grid)  
         const int bin = (atan2(yUrvizLobst, xUrvizLobst) + static_cast<double>(M_PI))/RESOLUTION; // bin=0 corresp to the 3D point located straight behind the sensor, i.e. along xUrvizLobst-axis but looking towards the negative values

         //V&V
         //ROS_INFO("%f -- %f -- %d",atan2(yUrvizLobst,xUrvizLobst) + static_cast<float>(M_PI),RESOLUTION,bin);

         if ((bin >= 0) && (bin < static_cast<int>(SIZE))) { //bin is within meaningful range
           // met1. just overwrite whatever value already exists on scan[id_vec_ring][bin] with the newly available data
           transform_rviz2b(scan[id_vec_ring][bin].xUbLobst, scan[id_vec_ring][bin].yUbLobst, xUrvizLobst, yUrvizLobst);

           scan[id_vec_ring][bin].distUbLobst  = sqrtf(pow(scan[id_vec_ring][bin].xUbLobst,2.0) + pow(scan[id_vec_ring][bin].yUbLobst,2.0));
           scan[id_vec_ring][bin].angleUbLobst = atan2(scan[id_vec_ring][bin].yUbLobst, scan[id_vec_ring][bin].xUbLobst); //[rad] angleUbLobst=0 corresp to obstacle being on xUbLobst-axis; angle defined wrt coord sys (oUbLobst,xUbLobst,yUbLobst,zUbLobst) described above
           scan[id_vec_ring][bin].intensity    = it[id_intensity];

           //met2. [WIP] worse-case approach: compare the newly available data with the one already existing on scan[id_vec_ring][bin] and store the one corresponding to closest distance to obstacle

         } // if((bin >= ..)

       } // if (ring_act == ...)
     } // for (.)

     /*
     //V&V
     ROS_INFO("Min dist to obstacle on this ring is %f",*std::min_element(scan_distUbLobst.begin(), scan_distUbLobst.end()));
     ROS_INFO("Max dist to obstacle on this ring is %f",*std::max_element(scan_distUbLobst.begin(), scan_distUbLobst.end()));

     ROS_INFO("Min intensity on this ring is %f",*std::min_element(scan_intensity.begin(), scan_intensity.end()));
     ROS_INFO("Max intensity on this ring is %f",*std::max_element(scan_intensity.begin(), scan_intensity.end()));

     ROS_INFO("Min angle on this ring is %f",*std::min_element(scan_angle.begin(), scan_angle.end()));
     ROS_INFO("Max angle on this ring is %f",*std::max_element(scan_angle.begin(), scan_angle.end()));

     for (float x:scan_angle) printf(" %0.2f",x); //std::cout << x << " ";
     */

     // testing-purpose only
     //scan[0][1].xUbLobst = nan("");

     /* Herebelow code suffers from a logic error (it's not a good idea to artificially create data, better keep nan where nothing can be concluded): I kept it commented future code reuse 
     // check stored data: handle any nan present in the data (recall nan was used to initialize scan[all][all].X): wcs approach by artificially adding harmless data;
     for (size_t id_vec_ring{0}; id_vec_ring < vec_ring.size(); ++id_vec_ring)  
     for (auto bin=0; bin<scan[id_vec_ring].size(); bin++) {
       if ( isnan(scan[id_vec_ring][bin].xUbLobst) || isnan(scan[id_vec_ring][bin].yUbLobst) || isnan(scan[id_vec_ring][bin].angleUbLobst) || isnan(scan[id_vec_ring][bin].distUbLobst) || isnan(scan[id_vec_ring][bin].intensity)) {
         ROS_INFO("Found nan within scan, so action will be taken:");

         // associate wcs values
         scan[id_vec_ring][bin].distUbLobst  = 33.0; //(close-to) max range of velodyne sensor
         scan[id_vec_ring][bin].intensity    = 1.0;  //lowest intensity value


         // conseq: extract (xUbLobst,yUbLobst) from (distUbLobst,angle)
         double xUrvizLobst_temp, yUrvizLobst_temp;
         inv_atan2(xUrvizLobst_temp, yUrvizLobst_temp, static_cast<double>(bin)*static_cast<double>(RESOLUTION) - static_cast<double>(M_PI), scan[id_vec_ring][bin].distUbLobst);
         transform_rviz2b(scan[id_vec_ring][bin].xUbLobst, scan[id_vec_ring][bin].yUbLobst, xUrvizLobst_temp, yUrvizLobst_temp);

         // conseq: extract angle
         scan[id_vec_ring][bin].angleUbLobst = atan2(scan[id_vec_ring][bin].yUbLobst,scan[id_vec_ring][bin].xUbLobst); // alternative method: the index-value i.e. bin value embeds info about the angle

         ROS_INFO(" The point (obstacle) values were artificially adjusted to: angleUbLobst=%f [deg], distUbLobst=%f [m], xUbLobst=%f [m], yUbLobst=%f [m], intensity=%f [?]",rad2deg(scan[id_vec_ring][bin].angleUbLobst), scan[id_vec_ring][bin].distUbLobst, scan[id_vec_ring][bin].xUbLobst, scan[id_vec_ring][bin].yUbLobst, scan[id_vec_ring][bin].intensity);
       } // if
      } //for (.)
       */

     /*
     //V&V: visualize scan
     ROS_INFO("scan.size()=%lu", scan.size());
     for (unsigned long bin=0; bin<scan.size(); bin+=10) {
       ROS_INFO("scan[%lu].distUbLobst=%f",bin,scan[id_vec_ring][bin].distUbLobst);
     } //for (auto bin=0; ..)
     */

     // call: as soon as we have new info about obstacles
     sharedControlAlgo();

   } //void velodyneSubscCallback

   // member function
   void sharedControlAlgo() {
     // Output: (vr_nz,omegar_nz)
     ROS_INFO("Entered sharedControlAlgo()");

     // ini: by default assume no need to enable ASC
     double vr_nz {vd_nz};
     double omegar_nz {omegad_nz};

     // V&V
     //publish_joy_SC(vr_nz,omegar_nz);

     // // // Main Algo
     // normalize values
     double v_nz     = v>0 ? v/SD2_absvmax_fwd : v/SD2_absvmax_bwd;
     double omega_nz = omega/SD2_omegamax;

     // // case1. vehicle advancing straight ahead (= fwd + wo rotation)
     ROS_INFO("Case1: check conditions: v_nz=%.3f; omega_nz=%.3f; vd_nz=%.3f",v_nz,omega_nz,vd_nz);
     if (     (v_nz>=0) && (abs(omega_nz)<=3*sigma_omega_nz)                           // combines 2 cases: case1) vehicle advancing fwd + wo rotation (v>0,omega=0); case2) vehicle at standstill (v=0,omega=0)
           && ((vd_nz>0) || ((vd_nz==0) && (abs(omegad_nz)<=thresh_omegad_nz)) )) {    // user expresses intention to advance fwd or stand still;
        //then it is meaningful/makes-sense to apply ASC
        ROS_INFO("Case1: entered");

        // // step1. find closest obstacle
        // ini: setting up point to far away corresponds to assuming/saying no danger lies ahead
        PointObst temp_PointObstStraightAhead_farAway {0, 33, 33, M_PI/2, 1};
        PointObst closestPointObstStraightAhead {temp_PointObstStraightAhead_farAway};

        // check all points on vec_ring and seek the closest obstacle 
        for (size_t id_vec_ring{0}; id_vec_ring < vec_ring.size(); ++id_vec_ring)  
        for (auto bin=0; bin<scan[id_vec_ring].size(); bin++ ) { 
          if (!std::isnan(scan[id_vec_ring][bin].distUbLobst) &&   // make sure to use meaningful data, otherwise in case scan[id_vec_ring][bin].distUbLobst == nan, then SC is to make use of the initialized closestPointObstStraightAhead 
               scan[id_vec_ring][bin].distUbLobst<closestPointObstStraightAhead.distUbLobst) { //then we have found a candidate for closestPointObst
            //ROS_INFO("Case1: Found closer point (obstacle): angleUbLobst=%f [deg], distUbLobst=%f [m], xUbLobst=%f [m], yUbLobst=%f [m], intensity=%f [?]",rad2deg(closestPointObstStraightAhead.angleUbLobst), closestPointObstStraightAhead.distUbLobst, closestPointObstStraightAhead.xUbLobst, closestPointObstStraightAhead.yUbLobst, closestPointObstStraightAhead.intensity);

            //check whether the candidate for closestPointObst is located straight ahead
            if (abs(scan[id_vec_ring][bin].xUbLobst) <= SD2_chassisWidth/2.0)  closestPointObstStraightAhead = scan[id_vec_ring][bin];
          } // if
        } //for (auto bin=..)

        ROS_INFO("Case1: The closest point (obstacle) is: angleUbLobst=%f [deg], distUbLobst=%f [m], xUbLobst=%f [m], yUbLobst=%f [m], intensity=%f [?]",rad2deg(closestPointObstStraightAhead.angleUbLobst), closestPointObstStraightAhead.distUbLobst, closestPointObstStraightAhead.xUbLobst, closestPointObstStraightAhead.yUbLobst, closestPointObstStraightAhead.intensity);

        // // step2. calc (vr_nz,omega_nz) = fct(distUbLobst)
        vr_nz     = std::min(calc_vrMax_nz(closestPointObstStraightAhead.distUbLobst-SD2_distSensorToEdgeOfRobot), vd_nz); // note xUvldnLobst := closestPointObstStraightAhead.distUbLobst; see the rela between axes xUvldn and xUobst, given below after the definition: double calc_vrMax_nz(.)
        omegar_nz = omegad_nz * vr_nz/vd_nz;

        ROS_INFO("Case1: User desires (vd_nz=%.3f,omegad_nz=%.3f), and SC issues (vr_nz=%.3f,omegar_nz=%.3f)", vd_nz,omegad_nz, vr_nz,omegar_nz);
      } //if ((v_nz>=0 ..)
      // otherwise keep (vr_nz,omegar_nz) unchanged

      // // case2. vehicle advancing fwd + w rotation
      // WIP..

      // // conclude
      publish_joy_SC(vr_nz,omegar_nz);


/*
     // // Algo: find closest obstacle on one ring and show its angle
     // choose
     id_vec_ring = 0;

     // ini
     PointObst closestPointObst = scan[id_vec_ring][0];

     // check all points
     for (auto bin=1; bin<scan[id_vec_ring].size(); bin++ ) { // index starts at 1 because the ini point was set to scan[id_vec_ring][0]
       if (!std::isnan(scan[id_vec_ring][bin].distUbLobst) &&
             scan[id_vec_ring][bin].distUbLobst<closestPointObst.distUbLobst) {
         closestPointObst = scan[id_vec_ring][bin];
       } // if
     } //for (.)

     // print result
     ROS_INFO("The closest point (obstacle) is: angleUbLobst=%f [deg], distUbLobst=%f [m], xUbLobst=%f [m], yUbLobst=%f [m], intensity=%f [?]",rad2deg(closestPointObst.angleUbLobst), closestPointObst.distUbLobst, closestPointObst.xUbLobst, closestPointObst.yUbLobst, closestPointObst.intensity);
*/

   } //void sharedControlAlgo()

   // member function: helper/utility function
   double calc_vrMax_nz(double xUobstLco) {  // relation between axes: xUvldn = xUobst + distSensorToEdgeOfRobot ; 'co' stands for 'closest obstacle'
     // Models the relation between vrUmax_nz and xUobstLco, as a mathematical function
     // Output: vrUmax_nz [-]

     // //[var1] linear fct fct(x)=a*x+b, intersecting the points (x=d0,vrMax=0) and (x=dmax,vrMax=1)
     // Choose
     double d0   = 0.2+0.1; //[m] d0 is where fct=fct(xUobst) starts ramping up; d0 defined along xUobst
     double dmax = 0.62; //[m] dmax is the distance-to-obstacle where the SC kicks in; dmax defined along xUobst

     // ini: safe value
     double vrMax_nz = nan("");

     if (xUobstLco<0.0) {
       ROS_ERROR("Please check xUobstLco");
     } else if (xUobstLco<=d0) {
       vrMax_nz = 0.0;
     } else if (xUobstLco<dmax) {
       vrMax_nz = (xUobstLco-d0)/(dmax-d0);
     } else {
       vrMax_nz = 1.0;
     } //if (.)

     //conclude
     return vrMax_nz;
   } // double calc_vrMax_nz(.)

   // member function: helper/utility function
   void publish_joy_SC(double vr_nz, double omegar_nz) {
      sensor_msgs::Joy joy_SC_msg;

      //ROS
      joy_SC_msg.header.stamp = ros::Time::now();

      joy_SC_msg.axes.push_back(0.0); //axes[0]
      joy_SC_msg.axes.push_back(0.0); //axes[1]
      joy_SC_msg.axes.push_back(0.0); //axes[2]
      joy_SC_msg.axes.push_back(omegar_nz); //axes[3] ru
      joy_SC_msg.axes.push_back(vr_nz); //axes[4]
      joy_SC_msg.axes.push_back(0.0); //axes[5]
      joy_SC_msg.axes.push_back(0.0); //axes[6]
      joy_SC_msg.axes.push_back(0.0); //axes[7]

      for (auto i=0; i<13; i++) joy_SC_msg.buttons.push_back(0);

      // conclude
      joypad_SC_pub_.publish(joy_SC_msg);
   } //void publish_joy_SC()

   // member function: helper/utility function
   void transform_rviz2b(double& xUbLobst, double& yUbLobst, const double& xUrvizLobst, const double& yUrvizLobst) {
    // Inputs: (xUrvizLobst,yUrvizLobst);   Outputs: (xUbLobst,yUbLobst)
    // pUb = RUbLrviz*pUrviz, where RUbLrviz:=R_{z,90[deg]+phiUbLmisalign} using notations [SpHuVi2005,p57]; pUb=def=[xUb; yUb]; pUrviz=def=[xUrviz; yUrviz];
    double phi_temp = M_PI/2 + phiUbLmisalign;
    xUbLobst = cos(phi_temp)*xUrvizLobst - sin(phi_temp)*yUrvizLobst;
    yUbLobst = sin(phi_temp)*xUrvizLobst + cos(phi_temp)*yUrvizLobst;
   } //void transform_rviz2b(.)

  // member function: helper/utility function
  template<typename T>
  bool isFoundElem(std::vector<T> v, T elem) {   //V&V1: isFoundElem(vec_ring, ring_act)
    //ini & by default assume elem is not part of v 
    bool isFound = false;

    for (auto it = std::begin(v); it != std::end(v); ++it)
        if (*it == elem) isFound = true;

    return isFound;
  } //bool isFoundElem(.)


   // static member function
   static void transform_b2rviz(double& xUrvizLobst, double& yUrvizLobst, const double& xUbLobst, const double& yUbLobst) {
    // Inouts: (xUrvizLobst,yUrvizLobst);   Outputs: (xUbLobst,yUbLobst)
    // pUb = RUbLrviz*pUrviz, where RUbLrviz:=R_{z,phi=90 deg}=def=[0 -1; 1 0]  using notations [SpHuVi2020]
    yUrvizLobst = -xUbLobst;
    xUrvizLobst = yUbLobst;
   } //void transform_rviz2b(.)

   // static member function
   static inline void inv_atan2(double& x, double& y, const double& theta, const double& len) {
     // Given (theta,len) previously computed using atan2(y,x), now extract (x,y)
     // Inputs (angle,len);   Outputs: (x,y)
     x = len*cos(theta);
     y = len*sin(theta);
   } // void inv_atan2(.)

public:
   // constructor
   explicit SharedControl_velodyne(ros::NodeHandle& nh): nh_(nh), vd_nz(0.0), omegad_nz(0.0) { //wo 'explicit' keyword, single-argument constructors have the potential to be wrongly used as conversion constructors, see B2022Deitel, C++ (3rd), section 11.9
      // // initialize vector scan accordingly      

      //V&V
      //ROS_INFO("temp=(%.2f,%.2f,%.2f,%.2f,%.2f)", temp.xUbLobst, temp.yUbLobst, temp.distUbLobst, temp.angleUbLobst, temp.intensity);

      // create a temporary 1D vector of dimension SIZE and populated by 'empty' PointObst
      std::vector<PointObst> temp_oneRow(static_cast<unsigned long int>(SIZE),temp_onePointObst);

      // set the correct size of scan[idx_vec_ring][bin] and initialize it with 'empty' PointObst
      scan.resize(vec_ring.size(), temp_oneRow);

      // // ROS subscribers and publishers
      velodyne_sub_  = nh_.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &SharedControl_velodyne::velodyneSubscCallback, this);
      joypad_sub_    = nh_.subscribe<sensor_msgs::Joy>("/joy_raw", 1, &SharedControl_velodyne::joypadSubscCallback, this);
      odom_sub_      = nh_.subscribe<nav_msgs::Odometry>("/wheel_odom", 1, &SharedControl_velodyne::odometrySubscCallback, this);
      joypad_SC_pub_ = nh_.advertise<sensor_msgs::Joy>("/joy_out",1);

   } // SharedControl_velodyne(.)

   // destructor
   ~SharedControl_velodyne() {
	    ROS_INFO(" Ended Shared Control!\n");
   } // ~SharedControl_veldyne()

   // static member function
   constexpr static inline double rad2deg(double rad) {
     return rad*180.0/M_PI;
   } //double rad2deg(double rad)

   // static member function
   constexpr static inline double deg2rad(double deg) {
     return deg*M_PI/180.0;
   } //double deg2rad(double deg)

     // static member function
     constexpr static inline double inch2m(double inch) {
       return 0.0254*inch;
     } //double inch2m(double inch)

}; // class SharedControl_velodyne


// ++++++++++++++++++++++++++++++++++++++
int main(int argc, char** argv) {
    //ROS_INFO("Test: %f; %f; %lu",M_PI,SharedControl_velodyne::RESOLUTION, SharedControl_velodyne::SIZE);

    ros::init(argc, argv, "shared_control_velodyne_node");
    ros::NodeHandle nh;
    SharedControl_velodyne sc(nh);
    ros::spin();

    return 0;
} // int main(.)
