/* Obstacle avoidance shared control algo for usage on teleoperated robots
Inputs:
- depth camera: Realsense D435: depth data corresp to selected vertical angles on the camera image matrix
- joypad: Sony DS4: right joystick data
- odometry: linear velocity, angular velocity


Code resources:
 http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html : for sensor_msgs/Image.msg
 http://docs.ros.org/en/api/image_geometry/html/c++/index.html 
 https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html : PinholeCameraModel, camera coordinate frame, doc opencv (used by image_geometry libr)

*/

#include <ros/ros.h>

#include <sensor_msgs/Image.h>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>

#include <vector>

#include <numeric>  // for std::iota
#include <iomanip>  // std::setw

// a point on an obstacle identified by velodyne
struct PointObst {
   double xUbLobst, yUbLobst, distUbLobst, angleUbLobst;
}; //struct PointObst

class SharedControl_realsense {
private:
	
    // // Part 1 - realsense config
    static constexpr size_t SIZE {848}; //this number needs to coincide with depth_msg->width 
    static constexpr double ZUcLobst_min {0.050}; // [m] arbitrary threshold; interpretation: consider meaningful only sensor readings depth->msg[:] >= ZUcLobst_min    

    ros::NodeHandle pnh_; // Private nodehandle used to generate the transport hints in the connectCb.
    image_transport::ImageTransport it_; // Subscribes to synchronized Image CameraInfo pairs.
    image_transport::CameraSubscriber sub_; // Subscriber for image_transport

    image_geometry::PinholeCameraModel cam_model_; // image_geometry helper class for managing sensor_msgs/CameraInfo messages ;; PinholeCameraModel, camera coordinate frame: see doc opencv (used by image_geometry libr): https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html ;;  


    // Choose the vertical angles 
    const std::vector<double> alphaLvertUdesired { deg2rad(+1), deg2rad(-1) }; // [rad]

    // // Part 2: physical qtts
    const double SM_absvmax_fwd  {0.3 * 3.0}; //[m/s] (tolerated/allowed for SC purpose) robot maximum linear velocity when advancing forward ;; 3 m/s shown on /odom when vehicle off load (suspended, with wheels up in the air), a gain factor is used to lower the vehicle's max ability 
    const double SM_absvmax_bwd  {SM_absvmax_fwd}; //[m/s] robot maximum linear velocity when advancing backwards
    const double SM_dimens_width {0.59}; //[m] Scout Mini distance between the outside of the wheels
    const double SM_absomegamax  {2.5};  //[rad/s] (tolerated/allowed for SC purpose) 2.5 rad/s shown on /odom when vehicle off load (suspended, with wheels up in the air); exerimentally I identified approx 1 rotation in 3 seconds meaning 2*pi/3 =approx= 2.1 [rad/s] ;; symmetrical max angular velocity meaning there is no distinction between max left and max right 
    const double SM_distSensorToEdgeOfRobot {.11}; //[m] dist from realsense sensor to the front of the robot('s enveloppe/footprint)

    // choose/define accordingly
    static constexpr double sigma_omega_nz {.30}; //[-] td: change value to 0.03 after implementing sharedControlAlgo() > Case2
    static constexpr double thresh_omegad_nz {.05}; //[-] threshold beyond which SC is NOT enabled

    // // Part 3 - data members
    double vd_nz, omegad_nz; //normalized values
    double v, omega; //odometry: v [m/s]; omega [rad/s]

    std::vector<std::vector<PointObst>> scan; // data on multiple rows of the image plane matrix;; scan[id_alphaLvertUdesired][u] 

    // // ROS
    ros::NodeHandle nh_; // in C++ the naming convention of a variable with an underscore usually indicates a private member variable: see B2017Newman, Comment p55
    ros::Subscriber joypad_sub_, odom_sub_;
    ros::Publisher  cmd_vel_pub_;

    // // Other
    // create a temporary 'empty' PointObst
    const PointObst temp_onePointObst {nan(""),nan(""),nan(""),nan("")};


    void depthCallback(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg){ 
        // // // Assumption for this work/code: depth_msg->data is rectified and placed on a uniform grid (i.e. (uv)-axes coincide to (u_rect,v_rect)-axes) 

        cam_model_.fromCameraInfo(info_msg);
        //descriptive_recvdData(depth_msg, info_msg);     

        // // Step. disregard previous measurements: reset all scan[id_alphaLvertUdesired][u] to NaN
        // Met1: compact
        for (std::vector<PointObst>& scan_elem:scan) std::fill(scan_elem.begin(),scan_elem.end(),temp_onePointObst ); //note the reference symbol otherwise it won't work

        /*
        //Met2. less compact: uses 2 for-each
        for (std::vector<PointObst>& scan_elem:scan) 
            for (PointObst& scan_elem_elem:scan_elem) 
                scan_elem_elem = temp_onePointObst;
        */


        /*
        // V&V: check whether any non-NaN info subsists/still-exists
        for (size_t idx_alphaLvertUdesired = 0; idx_alphaLvertUdesired < scan.size(); idx_alphaLvertUdesired++) for (size_t u=0; u<(size_t)depth_msg->width; u++)  
           if ( isfinite(scan[idx_alphaLvertUdesired][u].xUbLobst) || isfinite(scan[idx_alphaLvertUdesired][u].yUbLobst) || isfinite(scan[idx_alphaLvertUdesired][u].distUbLobst) || isfinite(scan[idx_alphaLvertUdesired][u].angleUbLobst) )  {
                print_onePointObst(scan[idx_alphaLvertUdesired][u], "Failed to reset this point:"); 
                std::cout << "idx_alphaLvertUdesired=" << idx_alphaLvertUdesired << ", u=" << u << "\n";
                ROS_ERROR("Resetting scan[:][:] failed");
                exit(0);
           } // if (.)
        */   

        // // Step. compute vertical angles alphaLvert = alphaLvert(v) on the intersection of the plane OcYcZc with the image plane 
        // def
        std::vector<double> alphaLvert((size_t)depth_msg->height); //alphaLvert = alphaLvert(v) 

        // populate alphaLvert        
        for (size_t v=0; v<(size_t)depth_msg->height; v++)  {  //[pixel]
            alphaLvert[v] = atan2((double)cam_model_.cy() - (double)v, cam_model_.fx()); // atan2(data along Yc-axis [px], data along Zc-axis [px])
        } // for (v=..)
        //print_vector_wIndex(alphaLvert,"alphaLvert [deg]",&rad2deg ); //V&V

        // // Step. identify which alphaLvert[idx_alphaLvert] is closest to each alphaLvertUdesired[:], and keep note of idx_alphaLvert
        for (size_t idx_alphaLvertUdesired = 0; idx_alphaLvertUdesired < alphaLvertUdesired.size(); idx_alphaLvertUdesired++ ) {
            // Limitation: assumption: alphaLvert is strictly monotonic, s.t. the intersection with a horizontal line gives a point, not a line segment
            std::cout << "\nidx_alphaLvertUdesired =" << idx_alphaLvertUdesired << "; alphaLvertUdesired[idx_alphaLvertUdesired] =" << rad2deg(alphaLvertUdesired[idx_alphaLvertUdesired]) << "[deg]";

            std::vector<double> vi_vec;
            std::vector<size_t> idi_range_v;

            std::vector<double> range_v(depth_msg->height);
            std::iota(range_v.begin(), range_v.end(), 0.0); 
            //print_vector(range_v,"range_v"); //V&V

            fzero_curve_new(vi_vec,idi_range_v, range_v,alphaLvert, 0, alphaLvertUdesired[idx_alphaLvertUdesired] );
            print_vector(idi_range_v,"idi_range_v"); //V&V
            size_t one_idi_range_v = idi_range_v[0]; //there should be just one idi_range_v anyway otherwise there might be a logic error
            
            // conseq: store the closest matching v 
            size_t v_assoc_to_curr_alphaLvertUdesired = range_v[one_idi_range_v]; // v associated to current alphaLvertUdesired[idx_alphaLvertUdesired]   
            std::cout << "v_assoc_to_curr_alphaLvertUdesired = " << v_assoc_to_curr_alphaLvertUdesired << "\n"; //V&V
            //ROS_INFO("%.2zu", v_assoc_to_curr_alphaLvertUdesired);

            // conseq: move cursor at the begining or row v=v_assoc_to_curr_alphaLvertUdesired in the camera image matrix  
            const uint16_t* depth_data_1D = reinterpret_cast<const uint16_t*>(&depth_msg->data[0]); 
            unsigned long int idx = (unsigned long int)v_assoc_to_curr_alphaLvertUdesired * (unsigned long int)(depth_msg->width);  //jump at the beginning of the row corresponding to v_assoc_to_curr_alphaLvertUdesired, on the image plane matrix ;; note that pointer arithmetic is defined wrt integer types, not size_t, thus conversions are necessary to be able to perform addition cf below
            //std::cout << "\nidx = " << idx; //V&V                 
            depth_data_1D += idx; // now cursor starts at the beginning of row 

            //V&V: display depth_data_1D corresponding to a row on the image plane matrix  
            //for (unsigned int u = 0; u < (unsigned int)depth_msg->width; ++u) std::cout << "\ndepth_data_1D[" << u << "]=" << depth_data_1D[u] ;

            // Step. populate scan[idx_alphaLvertUdesired][u] 
            for (size_t u=0; u<(size_t)depth_msg->width; u++)  { // [pixel]
                // Interpretation: consider meaningful only sensor readings depth->msg[:] >= ZUcLobst_min    
                if (mm2m((double)depth_data_1D[u]) >= ZUcLobst_min) {
                    double ZUcLobst = mm2m((double)depth_data_1D[u]); //[m] extract ZUcLobst
                    double XUcLobst = ZUcLobst*((double)u - cam_model_.cx())/cam_model_.fx(); //[m] note: cam_model.cx() and .cy() represent the projection of the optical centre onto the image plane, both expressed in (uv)-axes coordinates
                    double YUcLobst = ZUcLobst*((double)v_assoc_to_curr_alphaLvertUdesired - cam_model_.cy())/cam_model_.fx(); //[m] 
                    double alpha_c = atan2(XUcLobst,ZUcLobst); // based on projection onto OcXcZc plane

                    //V&V
                    //std::cout << "\n u=" << u << "; XUcLobst=" << XUcLobst << "[m]; YUcLobst=" << YUcLobst << "[m]; ZUcLobst=" << ZUcLobst << "[m]; alpha_c=" << rad2deg(alpha_c) << "[deg]";
                    
                    //conclude
                    scan[idx_alphaLvertUdesired][u].xUbLobst     = XUcLobst;
                    scan[idx_alphaLvertUdesired][u].yUbLobst     = ZUcLobst;
                    scan[idx_alphaLvertUdesired][u].distUbLobst  = hypot(XUcLobst,YUcLobst,ZUcLobst);
                    scan[idx_alphaLvertUdesired][u].angleUbLobst = M_PI/2 - alpha_c; // this angleUbLobst respects the same convention as used in art_GP1, namely angle span starting from Xc-axis and positive increasing towards Zc-axis   
                    
                } // if (mm2m(.)..)
                // otherwise, keep nan inside scan[idx_alphaLvertUdesired][u].* according to constructor's initialization

            } // for (size_t u=..)        
        } //for (size_t idx_alphaLvertUdesired =..)    
          
    // call: as soon as we have new info about obstacles
    sharedControlAlgo();

    } // void depthCallback(.)    

    // member function
    void sharedControlAlgo() {
        // Output: (vr_nz,omega_nz)
        ROS_INFO("Entered sharedControlAlgo()");

        // ini: by default assume no need to enable ASC
        double vr_nz {vd_nz};
        double omegar_nz {omegad_nz};

        // V&V
        //publish_joy_SC(vr_nz,omegar_nz);

        // // // Main Algo
        // normalize values
        double v_nz     = v>0 ? v/SM_absvmax_fwd : v/SM_absvmax_bwd;
        double omega_nz = omega/SM_absomegamax;

        // // case1. vehicle advancing straight ahead (= fwd + wo rotation)
        ROS_INFO("Case1: check conditions: v_nz=%.3f; omega_nz=%.3f; vd_nz=%.3f",v_nz,omega_nz,vd_nz);
        if (  (v_nz>=0) && (abs(omega_nz)<=3*sigma_omega_nz)                           // combines 2 cases: case1) vehicle advancing fwd + wo rotation (v>0,omega=0); case2) vehicle at standstill (v=0,omega=0)
           && ((vd_nz>0) || ((vd_nz==0) && (abs(omegad_nz)<=thresh_omegad_nz)) )) {    // user expresses intention to advance fwd or stand still
            //then it is meaningful/makes-sense to apply ASC
            ROS_INFO("Case1: entered");

            // // step1. find closest obstacle
            // ini: setting up point to far away corresponds to assuming/saying no danger lies ahead
            PointObst temp_PointObstStraightAhead_farAway {0, 33, 33, M_PI/2};
            PointObst closestPointObstStraightAhead {temp_PointObstStraightAhead_farAway};
            print_onePointObst(closestPointObstStraightAhead,"Case1: Initial values for closestPointObstStraightAhead: ");


            // check all points on alphaLvertUdesired and seek the closest obstacle 
            for (size_t id_alphaLvertUdesired{0}; id_alphaLvertUdesired < alphaLvertUdesired.size(); ++id_alphaLvertUdesired)  
            for (size_t u=0; u<(size_t)scan[id_alphaLvertUdesired].size(); u++) { 
            if (!std::isnan(scan[id_alphaLvertUdesired][u].distUbLobst) &&   // make sure to use meaningful data, otherwise in case scan[id_alphaLvertUdesired][u].distUbLobst == nan, then SC is to make use of the initialized closestPointObstStraightAhead 
                scan[id_alphaLvertUdesired][u].distUbLobst<closestPointObstStraightAhead.distUbLobst) { //then we have found a candidate for closestPointObst
                //ROS_INFO("Case1: Found closer point (obstacle): angleUbLobst=%f [deg], distUbLobst=%f [m], xUbLobst=%f [m], yUbLobst=%f [m]",rad2deg(closestPointObstStraightAhead.angleUbLobst), closestPointObstStraightAhead.distUbLobst, closestPointObstStraightAhead.xUbLobst, closestPointObstStraightAhead.yUbLobst);

                //check whether the candidate for closestPointObst is located straight ahead
                if (abs(scan[id_alphaLvertUdesired][u].xUbLobst) <= SM_dimens_width/2.0)  closestPointObstStraightAhead = scan[id_alphaLvertUdesired][u];
            } // if
                                                                                                                                } //for (size_t u=..)
            print_onePointObst(closestPointObstStraightAhead,"Case1: The closest point (obstacle) is: ");
            
            // // step2. calc (vr_nz,omega_nz) = fct(distUbLobst)
            vr_nz     = std::min(calc_vrMax_nz(closestPointObstStraightAhead.distUbLobst-SM_distSensorToEdgeOfRobot), vd_nz); // note xUrealsenseLobst := closestPointObstStraightAhead.distUbLobst; see the rela between axes xUrealsense and xUobst, given below after the definition: double calc_vrMax_nz(.) 
            omegar_nz = omegad_nz * vr_nz/vd_nz;


        } //if ((v_nz>= ..)
        // otherwise keep (vr_nz,omega_nz) unchanged

        // // case2. vehicle advancing fwd + w rotation
        // WIP..

        // // conclude
        publish_cmd_vel(vr_nz, omegar_nz);
    } // void sharedControlAlgo()

    // member function: helper/utility function
    double calc_vrMax_nz(double xUobstLco) {  // relation between axes: xUrealsense = xUobst + distSensorToEdgeOfRobot ; 'co' stands for 'closest obstacle'
        // Models the relation between vrUmax_nz and xUobstLco, as a mathematical function
        // Output: vrUmax_nz [-]

        // //[var1] linear fct fct(x)=a*x+b, intersecting the points (x=d0,vrMax=0) and (x=dmax,vrMax=1)
        // Choose
        double d0   = 0.20-0.05; //[m] d0 is where fct=fct(xUobst) starts ramping up; d0 defined along xUobst
        double dmax = 0.62-0.05; //[m] dmax is the distance-to-obstacle where the SC kicks in; dmax defined along xUobst

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

    // member function
    void joypadSubscCallback(const sensor_msgs::JoyConstPtr &msg) {
        // using axis convention (v_joy,omega_joy) from article [TeZhCa2020] i.e. art_whc2
        double v_joy = msg->axes[4];     //[-] \in [-1,1] the 5th element inside array axes represents advancing bwd/fwd user intention (max fwd = +1; max bwd = -1)
        double omega_joy = msg->axes[3]; // \in [-1,1]  the 4th element inside array axes represents turning right/left user intention (max left = +1; max right=-1)

        // conclude
        vd_nz = v_joy;         //[-]
        omegad_nz = omega_joy; //[-]        
        ROS_INFO("joypad data: %f -- %f", vd_nz, omegad_nz); // V&V
    } // void velodyneSubscCallback(.)

    // member function
    void odometrySubscCallback(const nav_msgs::Odometry::ConstPtr &msg)  {
        v = msg->twist.twist.linear.x;      //[m/s]
        omega = msg->twist.twist.angular.z; //[rad/s]
        //ROS_INFO("Raw measurem: Lin velo: v = %.2f [m/s]; Angular velo: omega = %.2f [rad/s]",v,omega); //V&V

        // postprocess: saturate: safety reasons (data consistency)
        v = std::min(SM_absvmax_fwd, std::max(-SM_absvmax_bwd, v));    // overwrite
        omega = std::min(SM_absomegamax, std::max(-SM_absomegamax, omega));  // overwrite
        //ROS_INFO("Postproc:     Lin velo: v = %.2f [m/s]; Angular velo: omega = %.2f [rad/s]",v,omega); //V&V
    } // void odometrySubscCallback(.)

    // member function: helper/utility function
    void publish_cmd_vel(double vr_nz, double omegar_nz) {
      geometry_msgs::Twist cmd_vel_msg;

      // ROS
      cmd_vel_msg.linear.x  = vr_nz > 0 ? vr_nz*SM_absvmax_fwd : vr_nz*SM_absvmax_bwd;   //[m/s]
      cmd_vel_msg.angular.z = omegar_nz*SM_absomegamax;  //[rad/s] symmetrical max angular velocity i.e. there is no distinction between max left and max right

      // conclude
      //ROS_INFO("Will publish cmd_vel_msg.linear.x=%f, cmd_vel_msg.angular.z=%f", cmd_vel_msg.linear.x, cmd_vel_msg.angular.z);
      cmd_vel_pub_.publish(cmd_vel_msg);
    } //void publish_cmd_vel(.)

    void descriptive_recvdData(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg) { 
        ROS_INFO("Received sensor_msgs::Image with the content:");
        std::cout << ".height=" << depth_msg->height << "; .width=" << depth_msg->width << "; .encoding=" << depth_msg->encoding << "; .is_bigendian=" << depth_msg->is_bigendian << "; .step=" << depth_msg->step << "\n";

        ROS_INFO("Received sensor_msgs::CameraInfo used to build a Pinhole Camera Model with the content:");
        std::cout << "optical centre: .cx=" << cam_model_.cx() << " [px] .cy=" << cam_model_.cy() << " [px]\n" ; 
        std::cout << "focal lengths:  .fx=" << cam_model_.fx() << " [px] .fy=" << cam_model_.fy() << " [px]" ;

        // // Calculate angle_left and angle_right by measuring angles between the left ray and optical center ray, and right ray and optical center ray, respectively.
        // 1. calculate ray_left
        cv::Point2d raw_pixel_left(0, cam_model_.cy()); // Point2d specified by its coordinates (u,v) [px]; naming convention 'left' because it sits on the left-hand side of plane OcZcYc in the Pinhole Camera Model image from https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html   
        std::cout << "raw_pixel_left=" << raw_pixel_left << "\n";

        cv::Point2d rect_pixel_left = cam_model_.rectifyPoint(raw_pixel_left); // Point2d specified by its coordinates (u,v) [px]; 
        std::cout << "rect_pixel_left=" << rect_pixel_left << "\n";
        
        cv::Point3d ray_left = cam_model_.projectPixelTo3dRay(rect_pixel_left); // [-] normalized coordinates s.t. Zc=fixed=1; .projectPixelTo3dRay returns a 3d ray in the camera coordinate frame OcXcYcZc passing through (u,v), with coordinates s.t. Zc =fixed= 1 ;; doc member fct .projectPixelTo3dRay(): http://docs.ros.org/en/api/image_geometry/html/c++/classimage__geometry_1_1PinholeCameraModel.html#a40ad5d3b964d63f41232428fb96376fe ;; the word 'project' is misleading, instead it associates a Point3d, say p, on the ray going from the origin of the camera coordinate frame intersecting (u,v), this Point3d p has arbitrary z =fixed= 1.0, meaning units of measurement can be anything [m], [mm], etc. 
        std::cout << "ray_left=" << ray_left << "\n";

        // 2. calculate ray_right
        cv::Point2d raw_pixel_right(depth_msg->width-1, cam_model_.cy()); // Point2d specified by its coordinates (u,v) [px]; naming convention 'right' because it sits on the right-hand side of plane OcZcYc in the Pinhole Camera Model image from https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html   
        cv::Point2d rect_pixel_right = cam_model_.rectifyPoint(raw_pixel_right); // Point2d specified by its coordinates (u,v) [px]; 
        cv::Point3d ray_right = cam_model_.projectPixelTo3dRay(rect_pixel_right); // [-] normalized coordinates s.t. Zc=fixed=1; 
        std::cout << "ray_right=" << ray_right << "\n";

        // 3. calculate ray_up
        cv::Point2d raw_pixel_up(cam_model_.cx(),0); // Point2d specified by its coordinates (u,v) [px]; 
        cv::Point2d rect_pixel_up = cam_model_.rectifyPoint(raw_pixel_up); // Point2d specified by its coordinates (u,v) [px]; 
        cv::Point3d ray_up = cam_model_.projectPixelTo3dRay(rect_pixel_up); // [-] normalized coordinates s.t. Zc=fixed=1; 
        std::cout << "ray_up=" << ray_up << "\n";

        // 4. calculate ray_down
        cv::Point2d raw_pixel_down(cam_model_.cx(),depth_msg->height-1); // Point2d specified by its coordinates (u,v) [px]; 
        cv::Point2d rect_pixel_down = cam_model_.rectifyPoint(raw_pixel_down); // Point2d specified by its coordinates (u,v) [px]; 
        cv::Point3d ray_down = cam_model_.projectPixelTo3dRay(rect_pixel_down); // [-] normalized coordinates s.t. Zc=fixed=1; 
        std::cout << "ray_down=" << ray_down << "\n";

        // 5. calculate ray_center
        cv::Point2d raw_pixel_center(cam_model_.cx(), cam_model_.cy());
        cv::Point2d rect_pixel_center = cam_model_.rectifyPoint(raw_pixel_center);
        cv::Point3d ray_center = cam_model_.projectPixelTo3dRay(rect_pixel_center); // .projectPixelTo3dRay returns a 3d ray in the camera coordinate frame OcXcYcZc passing through (u,v), with coordinates s.t. Zc =fixed= 1 
        std::cout << "ray_center=" << ray_center << "\n";

        // 6. compute viewing angles
        const double angle_left  = angle_between_rays(ray_left, ray_center);   // positive, this would correspond to a laserscan message's angle_max 
        const double angle_right = -angle_between_rays(ray_right, ray_center); // negative, this would correspond to a laserscan message's angle_min 
        const double angle_up    = angle_between_rays(ray_up, ray_center);     // positive
        const double angle_down  = -angle_between_rays(ray_down, ray_center);  // negative

        // 7. visualize results
        ROS_INFO("Boundaries viewing angle:");        
        ROS_INFO(" angle_left=%f [rad], angle_right=%f [rad]", angle_left, angle_right);
        ROS_INFO(" angle_left=%f [deg], angle_right=%f [deg]", rad2deg(angle_left), rad2deg(angle_right));
        ROS_INFO(" angle_up=%f [rad], angle_down=%f [rad]", angle_up, angle_down);
        ROS_INFO(" angle_up=%f [deg], angle_down=%f [deg]", rad2deg(angle_up), rad2deg(angle_down));

        // 10. study .encoding
        ROS_INFO("Study .encoding");
        if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) std::cout << " .encoding is type <uint16_t>\n";	  
        if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) std::cout << " .encoding is type <float>\n";	  

    } //void descriptive_info(.)
 
    double angle_between_rays(const cv::Point3d& ray1, const cv::Point3d& ray2) const{
        const double dot_product = ray1.x*ray2.x + ray1.y*ray2.y + ray1.z*ray2.z;
        const double magnitude1 = magnitude_of_ray(ray1);
        const double magnitude2 = magnitude_of_ray(ray2);;
        return acos(dot_product / (magnitude1 * magnitude2));
    } // double angle_between_rays(.)

    double magnitude_of_ray(const cv::Point3d& ray) const{
        return sqrt(pow(ray.x, 2.0) + pow(ray.y, 2.0) + pow(ray.z, 2.0));
    } // double magnitude_of_ray(.)

public:
    // constructor
    explicit SharedControl_realsense(ros::NodeHandle& nh, ros::NodeHandle& pnh): nh_(nh), pnh_(pnh), it_(nh) {
        // // initialize vector scan accordingly          
        // create a temporary 1D vector of dimension SIZE and populated by 'empty' PointObst
        const std::vector<PointObst> temp_oneRow(static_cast<unsigned long int>(SIZE),temp_onePointObst);
        
        // set the correct size of scan[id_alphaLvertUdesired][u] and initialize it with 'empty' PointObst
        scan.resize(alphaLvertUdesired.size(), temp_oneRow);
        

        // ROS subscribers and publishers
        image_transport::TransportHints hints("raw", ros::TransportHints(), pnh_);
        sub_ = it_.subscribeCamera("/camera/depth/image_rect_raw", 10, &SharedControl_realsense::depthCallback, this, hints);

        joypad_sub_    = nh_.subscribe<sensor_msgs::Joy>("/joy_raw", 1, &SharedControl_realsense::joypadSubscCallback, this);
        odom_sub_      = nh_.subscribe<nav_msgs::Odometry>("/odom", 1, &SharedControl_realsense::odometrySubscCallback, this);
        cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);

	} //explicit SharedControl_realsense(.)

    // member function: helper/utility function
    constexpr static inline double rad2deg(double rad) {
      return rad*180.0/M_PI;
    } //double rad2deg(double rad)

    // member function: helper/utility function
    constexpr static inline double deg2rad(double deg) {
      return deg*M_PI/180.0;
    } //double deg2rad(double deg)
  
    // member function: override
    template <typename T>
    void print_vector(std::vector<T>& v, std::string name ) {  // fct with 2 arguments
        std::cout << "\n" << name << " = [";
        for (T v_elem:v)  std::cout << " " << v_elem;
        std::cout << "]\n";
    } // T print_vector(.)  

    // member function: override
    template <typename T>
    void print_vector(std::vector<T>& v, std::string name, T (*func)(T) ) {  // fct with 3 arguments
        std::cout << "\n" << name << " = [";
        for (T v_elem:v)  std::cout << " " << func(v_elem);
        std::cout << "]\n";
    } // T print_vector(.)

    // member function
    template <typename T>
    void print_vector_wIndex(std::vector<T>& v, std::string name, T (*func)(T) ) {  // fct with 3 arguments; 
    // Display a 2-column table
        std::cout << "\n" << std::setw(5) << "idx" <<  "|" << std::setw(5) << name << "\n";
        for (size_t id=0; id<v.size(); id++) std::cout << std::setw(5) << id << "|" << func(v.at(id)) << "\n";
        std::cout << "\n";
    } // T print_vector(.)   

    // member function
    template <typename T>
    T hypot(T x, T y, T z) { // the version of hypot with 3d point input is only available in C++17 https://en.cppreference.com/w/cpp/numeric/math/hypot
        return sqrt(pow(x,2.0)+pow(y,2.0)+pow(z,2.0));
    } // T hypot(.)

    template <typename T>
    constexpr static inline T mm2m(T mm) { return .001*mm;}

    // static member function
    constexpr static inline double inch2m(double inch) {
      return 0.0254*inch;
    } //double inch2m(double inch)

    template <typename T> 
    int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    } // int sgn(.)
    
    void fzero_curve_new(std::vector<double>& xi_vec, std::vector<size_t>& id_x_vec_cl2intersPct, std::vector<double> x_vec, std::vector<double> f_vec, double b, double c, bool doPlot = false) {
    /* Returns the (interpolated) x-coordinates of the intersection points between an enclined line segment g(x)=b*x+c, namely of slope b and intercept, and a curve f(x) specified by a discrete number of points (x_vec,f_vec)   
        Outputs: xi_vec is a vector of the intersection points
                id_x_vec_cl2intersPct are the indices inside x_vec corresponding to x_vec values closest to xi_vec

        Inputs: pair of vectors (x_vec,f_vec) having same size, defining a discrete set of points on the curve f(x)
                the slope b and the intercept c, defining the line g(x) = b*x+c
        Limitations: Assumptions: f(x) and g(x) intersect each other in (one or more) points, but do not overlap over a line of segment i.e. no 2 or more successive points on f_vec overlapping g(x)=b*x+c 
        
        Ex1. fzero_curve_new(xi_vec, id_x_vec_cl2intersPct, { 0, 1, 2, 3 }, { -1, 1, -2, 1 }, 0, 0); // yields three intersection points xi_vec=[.5 1.(3) 2.(6)] and corresponding id_x_vec_cl2intersPct=[0 1 3];
        Ex2. fzero_curve_new(xi_vec, id_x_vec_cl2intersPct, { 0, 1, 2, 3 }, { -1, 0, -2, 1 }, 0, 0); // yields two intersection point xi_vec=[1 2.(6)] and corresponding id_x_vec_cl2intersPct=[1 3];
        Ex3. fzero_curve_new(xi_vec, id_x_vec_cl2intersPct, { 0, 1, 2}, { 0, 1, -2 }, 0, 0);   // yields two intersection point xi_vec=[0 1.(3)] and corresponding id_x_vec_cl2intersPct=[0 1]
        Ex4. fzero_curve_new(xi_vec, id_x_vec_cl2intersPct, { 0, 1, 2 }, { -1, 0, -2 }, 0, 0); // yields one intersection point xi_vec=1 and corresponding id_x_vec_cl2intersPct=1 
        Ex5. fzero_curve_new(xi_vec, id_x_vec_cl2intersPct, { 0, 1, 2, 3 }, { -1, 0, 0, 1 }, 0, 0); // this situation corresponds to an infinite nr of intersection points i.e. a line segment and is beyond the assumptions/limitations of this code; as such, the output is undefined
    */

        std::vector<double> e_vec(x_vec.size());
        for (size_t id = 0; id < x_vec.size(); id++) e_vec[id] = b * x_vec[id] + c - f_vec[id];

        for (size_t id = 0; id < x_vec.size()-1; id++) {
            // case1
            if (sgn(e_vec[id + 1]) == 0) continue; //idea: mark the intersection point e_vec[id+1] == 0 at the next iteration
            // otherwise pursue hereafter

            // case2
            if (sgn(e_vec[id]) != sgn(e_vec[id+1])) { // then we've got an intersection point
                xi_vec.push_back( x_vec[id] - e_vec[id]*(x_vec[id+1]-x_vec[id])/(e_vec[id+1]-e_vec[id]) );

                // ini & by default assume that x_vec[id] is closest to the intersection point xi_vec[id]
                size_t temp_cl2intersPct = id;

                if ( std::abs(x_vec[id] - xi_vec.at(xi_vec.size()-1)) > std::abs(x_vec[id + 1] - xi_vec.at(xi_vec.size()-1)) ) temp_cl2intersPct = id + 1;

                // store
                id_x_vec_cl2intersPct.push_back(temp_cl2intersPct);

            } // if (sgn((.)..)

        } //for (size_t id; ..)

        // Visualize results:
        if (doPlot) {
            std::cout << "\nfzero_curve(.): Output: \nxi_vec_elem: [";
            for (auto xi_vec_elem : xi_vec) std::cout << " " << xi_vec_elem;
            std::cout << "] \nid_x_vec_cl2intersPct_elem: [";
            for (auto id_x_vec_cl2intersPct_elem : id_x_vec_cl2intersPct) std::cout << " " << id_x_vec_cl2intersPct_elem;
            std::cout << "]\n";    
        } //if (doPlot)
    } //void fzero_curve_new(.)

    void print_onePointObst(PointObst pointObst, std::string msg) {
        std::cout << msg << "angleUbLobst=" << rad2deg(pointObst.angleUbLobst) << "[deg], distUbLobst=" << pointObst.distUbLobst << "[m], xUbLobst=" << pointObst.xUbLobst << "[m], yUbLobst=" << pointObst.yUbLobst << "[m]\n";
    } // void print_omePointObst

    }; // class SharedControl_realsense


// ++++++++++++++++++++++++++++++++++++++
int main(int argc, char** argv) {

    ros::init(argc, argv, "shared_control_velodyne_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    SharedControl_realsense sc(nh,pnh);
    ros::spin();

    return EXIT_SUCCESS;
} // int main(.)