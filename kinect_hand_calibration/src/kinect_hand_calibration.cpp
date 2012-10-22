#include <ros/ros.h>
#include <math.h>
#include <image_transport/image_transport.h>
#include "kinect_hand_calibration/DoCalibration.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <eigen3/Eigen/Eigen>
#include <joint_motion_service/move_joints_service.h>
#include <tinyxml.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace enc = sensor_msgs::image_encodings;

using namespace cv;

static const float kinectFocalLength = 530;
static const int PATTERNWIDTH = 8;
static const int PATTERNHEIGHT = 6;

static const int IGNOREOUTERCOLROWS = 1;

class KinectHandCalibration
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber colorImgSub;
  image_transport::Subscriber depthImgSub;
  ros::Subscriber calibSub;
  //image_transport::Publisher image_pub_;
  ros::ServiceClient jointMotionServiceClient;

  bool captureColorImg;
  bool captureDepthImg;

  // parameters
  string colorTopicName;
  string depthTopicName;

  string calibrationFile;

  string parentFrame;
  string childFrame;
  string turnAxisFrame;

  double tfXOffset;

  // captured images
  cv::Mat colorImg;
  cv::Mat depthImg;
  string sensorFrame;

  // current transform
  tf::Transform currentTransform;

  tf::TransformListener tfListener;

public:
  KinectHandCalibration()
    : it_(nh_)
  {
    captureColorImg = false;
    captureDepthImg = false;
    //image_pub_ = it_.advertise("out", 1);
    calibSub = nh_.subscribe("in_calibrate", 1, &KinectHandCalibration::doCalibrationCb, this);
    jointMotionServiceClient = nh_.serviceClient<joint_motion_service::move_joints_service>("move_joints_service");

    ros::NodeHandle prnh("~");
    prnh.param<std::string>("in_color", colorTopicName, "/camera/rgb/image_color");
    prnh.param<std::string>("in_depth", depthTopicName, "/camera/depth_registered/image");
    prnh.param<std::string>("xml_calibration_file_path", calibrationFile, "kinectHandCalibration.xml");

    ROS_INFO("Config path: %s", calibrationFile.c_str());

    prnh.param<std::string>("parent_frame", parentFrame, "/CameraMount");
    prnh.param<std::string>("child_frame", childFrame, "/camera_link");
    prnh.param<std::string>("turn_axis_frame", turnAxisFrame, "/Gripper");

    prnh.param<double>("tf_x_offset", tfXOffset, 0);



    FILE *fp = fopen(calibrationFile.c_str() , "r");
    if (fp)
    {
      fclose(fp);
      ROS_INFO("Calibration found. Loading file...");
      readTransformFromFile();
    }
    else
    {
      ROS_INFO("Calibration not found. Creating new file...");
      currentTransform.setOrigin(tf::Vector3(tfXOffset, 0, 0));
      tf::Quaternion q;
      q.setRPY(0, 0, 0);
      currentTransform.setRotation(q);
      writeTransformToFile();
    }
    ROS_INFO("Finished initialization of Kinect-Hand-Calibrator");
  }

  ~KinectHandCalibration()
  {
  }

  void colorImageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    if (captureColorImg)
    {
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg);
        colorImg = cv_ptr->image;
        sensorFrame = msg->header.frame_id;
        captureColorImg = false;
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
    }
  }

  void depthImageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    if (captureDepthImg)
    {
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, "32FC1");
        depthImg = cv_ptr->image;
        captureDepthImg = false;
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
    }
  }

  void doCalibrationCb(const kinect_hand_calibration::DoCalibrationConstPtr& msg)
  {
    ROS_INFO("Activating subscriptions to kinect topics!");
    colorImgSub = it_.subscribe(colorTopicName, 1, &KinectHandCalibration::colorImageCb, this);
    depthImgSub = it_.subscribe(depthTopicName, 1, &KinectHandCalibration::depthImageCb, this);

    performCalibration();

    ROS_INFO("Desubscribing from kinect topics!");
    colorImgSub.shutdown();
    depthImgSub.shutdown();
  }

  void performCalibration()
  {
    ROS_INFO("Turning to 90°...");
    if (!rotateSingleJoint(4, M_PI_2, 500))
    {
      ROS_INFO("Rotation failed!");
      return;
    }

    ROS_INFO("Finished turning, starting capture...");

    captureColorImg = true;
    captureDepthImg = true;
    while(captureColorImg && nh_.ok())
    {
      ros::spinOnce();
    }

    if (!nh_.ok())
    {
      return;
    }

    ROS_INFO("Finished waiting for ColorImage");

    Mat grey;
    cvtColor(colorImg, grey, cv::COLOR_BGR2GRAY);

    Size patternsize(PATTERNWIDTH, PATTERNHEIGHT);
    vector<cv::Point2f> corners;
    bool foundCorners = findChessboardCorners(grey, patternsize, corners);

    while(captureDepthImg && nh_.ok())
    {
      ros::spinOnce();
    }

    if (!nh_.ok())
    {
      return;
    }

    ROS_INFO("Finished waiting for DepthImage");

    Point2f pMatrixLt[PATTERNWIDTH][PATTERNHEIGHT];
    float dMatrixLt[PATTERNWIDTH][PATTERNHEIGHT];
    float rowSlopesLt[PATTERNHEIGHT];
    float colSlopesLt[PATTERNWIDTH];

    if (foundCorners)
    {
      for (int i = 0; i < patternsize.height; i++)
      {
        // rows
        for (int j = 0; j < patternsize.width; j++)
        {
          Point2f cp = corners[i * patternsize.width + j];
          float curAvgDepth = getAvgDepth(cp, 1);

          float x = (cp.x - (grey.cols >> 1)) * curAvgDepth / kinectFocalLength;
          float y = (cp.y - (grey.rows >> 1)) * curAvgDepth / kinectFocalLength;

          pMatrixLt[j][i] = Point2f(x, y);
          dMatrixLt[j][i] = curAvgDepth;

          // points
          //ROS_INFO("Point (%i, %i): %f - %f: %f", i, j, x, y, curAvgDepth);
        }
      }
    }
    else
    {
      ROS_INFO("No calibration chart detected!");
      return;
    }

    //cv::imshow(WINDOW, markedImg);
    //cv::waitKey(3);

    ROS_INFO("Turning to -90°...");

    if (!rotateSingleJoint(4, -M_PI_2, 500))
    {
      ROS_INFO("Rotation failed!");
      return;
    }

    ROS_INFO("Finished turning, starting capture...");

    captureColorImg = true;
    captureDepthImg = true;
    while(captureColorImg && nh_.ok())
    {
      ros::spinOnce();
    }

    if (!nh_.ok())
    {
      return;
    }

    ROS_INFO("Finished waiting for ColorImage");

    cvtColor(colorImg, grey, cv::COLOR_BGR2GRAY);

    foundCorners = findChessboardCorners(grey, patternsize, corners);

    while(captureDepthImg && nh_.ok())
    {
      ros::spinOnce();
    }

    if (!nh_.ok())
    {
      return;
    }

    ROS_INFO("Finished waiting for DepthImage");

    Point2f pMatrixRt[PATTERNWIDTH][PATTERNHEIGHT];
    float dMatrixRt[PATTERNWIDTH][PATTERNHEIGHT];
    float rowSlopesRt[PATTERNHEIGHT];
    float colSlopesRt[PATTERNWIDTH];

    if (foundCorners)
    {

      for (int i = 0; i < patternsize.height; i++)
      {
        // rows
        for (int j = 0; j < patternsize.width; j++)
        {
          Point2f cp = corners[i * patternsize.width + j];
          float curAvgDepth = getAvgDepth(cp, 1);

          float x = (cp.x - (grey.cols >> 1)) * curAvgDepth / kinectFocalLength;
          float y = (cp.y - (grey.rows >> 1)) * curAvgDepth / kinectFocalLength;

          pMatrixRt[j][i] = Point2f(x, y);
          dMatrixRt[j][i] = curAvgDepth;

          // points
          //ROS_INFO("Point (%i, %i): %f - %f: %f", i, j, x, y, curAvgDepth);
        }
      }
    }
    else
    {
      ROS_INFO("No calibration chart detected!");
      return;
    }
    calcColumnSlopes(pMatrixLt, dMatrixLt, colSlopesLt);
    calcColumnSlopes(pMatrixRt, dMatrixRt, colSlopesRt);

    // calculate pitch (x front, y left, z up)
    double pitchCorrection = 0;
    for (int col = IGNOREOUTERCOLROWS; col < PATTERNWIDTH - IGNOREOUTERCOLROWS; col++)
    {
      double angle = (atan(colSlopesLt[col]) + atan(colSlopesRt[PATTERNWIDTH - IGNOREOUTERCOLROWS - col])) / 2;
      pitchCorrection += angle;
      //ROS_INFO("Pitch angle %i: %f", col, angle);
    }
    pitchCorrection /= PATTERNWIDTH - (IGNOREOUTERCOLROWS * 2);
    ROS_INFO("Pitch correction: %f", pitchCorrection);

    // first correct pitch
    for (int col = 0; col < PATTERNWIDTH; col++)
    {
      for (int row = 0; row < PATTERNHEIGHT; row++)
      {
        double xLt = pMatrixLt[col][row].x;
        double yLt = pMatrixLt[col][row].y;
        double zLt = dMatrixLt[col][row];
        double xRt = pMatrixRt[col][row].x;
        double yRt = pMatrixRt[col][row].y;
        double zRt = dMatrixRt[col][row];

        double yLtS = cos(-pitchCorrection) * yLt - sin(-pitchCorrection) * zLt;
        double yRtS = cos(-pitchCorrection) * yRt - sin(-pitchCorrection) * zRt;

        double zLtS = sin(-pitchCorrection) * yLt + cos(-pitchCorrection) * zLt;
        double zRtS = sin(-pitchCorrection) * yRt + cos(-pitchCorrection) * zRt;

        pMatrixLt[col][row].y = yLtS;
        dMatrixLt[col][row] = zLtS;
        pMatrixRt[col][row].y = yRtS;
        dMatrixRt[col][row] = zRtS;
      }
    }

    calcRowSlopes(pMatrixLt, dMatrixLt, rowSlopesLt);
    calcRowSlopes(pMatrixRt, dMatrixRt, rowSlopesRt);

    // calculate yaw (x front, y left, z up)
    double yawCorrection = 0;
    for (int row = IGNOREOUTERCOLROWS; row < PATTERNHEIGHT - IGNOREOUTERCOLROWS; row++)
    {
      double angle = (atan(rowSlopesLt[row]) + atan(rowSlopesRt[PATTERNHEIGHT - IGNOREOUTERCOLROWS - row])) / 2;
      yawCorrection += angle;
      //ROS_INFO("yaw angle %i: %f", row, angle);
    }
    yawCorrection /= PATTERNHEIGHT - (IGNOREOUTERCOLROWS * 2);
    ROS_INFO("yaw correction: %f", yawCorrection);

    // correct yaw
    for (int col = 0; col < PATTERNWIDTH; col++)
    {
      for (int row = 0; row < PATTERNHEIGHT; row++)
      {
        double xLt = pMatrixLt[col][row].x;
        double yLt = pMatrixLt[col][row].y;
        double zLt = dMatrixLt[col][row];
        double xRt = pMatrixRt[col][row].x;
        double yRt = pMatrixRt[col][row].y;
        double zRt = dMatrixRt[col][row];

        double xLtT = cos(-yawCorrection) * xLt - sin(-yawCorrection) * zLt;
        double xRtT = cos(-yawCorrection) * xRt - sin(-yawCorrection) * zRt;

        double zLtT = sin(-yawCorrection) * xLt + cos(-yawCorrection) * zLt;
        double zRtT = sin(-yawCorrection) * xRt + cos(-yawCorrection) * zRt;

        pMatrixLt[col][row].x = xLtT;
        dMatrixLt[col][row] = zLtT;
        pMatrixRt[col][row].x = xRtT;
        dMatrixRt[col][row] = zRtT;
      }
    }

    double sumTransX = 0;
    double sumTransY = 0;
    int cnt = 0;

    for (int col = IGNOREOUTERCOLROWS; col < PATTERNWIDTH - IGNOREOUTERCOLROWS; col++)
    {
      for (int row = IGNOREOUTERCOLROWS; row < PATTERNHEIGHT - IGNOREOUTERCOLROWS; row++)
      {
        double xLt = pMatrixLt[col][row].x;
        double yLt = pMatrixLt[col][row].y;
        //double zLt = dMatrixLt[col][row];
        double xRt = pMatrixRt[PATTERNWIDTH - IGNOREOUTERCOLROWS - col][PATTERNHEIGHT - IGNOREOUTERCOLROWS - row].x;
        double yRt = pMatrixRt[PATTERNWIDTH - IGNOREOUTERCOLROWS - col][PATTERNHEIGHT - IGNOREOUTERCOLROWS - row].y;
        //double zRt = dMatrixRt[PATTERNWIDTH - col - 1][PATTERNHEIGHT - row - 1];

        // first correct pitch
//        double xLtS = xLt;
//        double xRtS = xRt;
//
//        double yLtS = cos(-pitchCorrection) * yLt - sin(-pitchCorrection) * zLt;
//        double yRtS = cos(-pitchCorrection) * yRt - sin(-pitchCorrection) * zRt;
//
//        double zLtS = sin(-pitchCorrection) * yLt + cos(-pitchCorrection) * zLt;
//        double zRtS = sin(-pitchCorrection) * yRt + cos(-pitchCorrection) * zRt;

        // correct yaw
//        double xLtT = cos(-yawCorrection) * xLtS - sin(-yawCorrection) * zLtS;
//        double xRtT = cos(-yawCorrection) * xRtS - sin(-yawCorrection) * zRtS;
//
//        double yLtT = yLtS;
//        double yRtT = yRtS;

        //double zLtT = -sin(-yawCorrection) * xLtS + cos(-yawCorrection) * zLtS;
        //double zRtT = -sin(-yawCorrection) * xRtS + cos(-yawCorrection) * zRtS;


        //ROS_INFO("Point (%i, %i): %f - %f: %f", row, col, xLtT, yLtT, zLtT);
        //ROS_INFO("Translation (elevation; shift; depth): %f; %f; %f", (yLtT + yRtT)/2, (xLtT + xRtT)/2, (zLtT - zRtT)/2);
        sumTransX += (yLt + yRt)/2;
        sumTransY += (xLt + xRt)/2;
        cnt++;
      }
    }
    ROS_INFO("Translation (elevation; shift): %f; %f", sumTransX/cnt, sumTransY/cnt);

    buildTargetTransform(sumTransX/cnt, sumTransY/cnt, yawCorrection, pitchCorrection);
    writeTransformToFile();
  }

  void buildTargetTransform(double elevation, double shift, double yaw, double pitch)
  {
    tf::StampedTransform axisToParent, childToSensor;
    // "how to get from parentFrame to turnAxisFrame"
    tfListener.lookupTransform(parentFrame, turnAxisFrame, ros::Time(0), axisToParent);
    ROS_INFO("Axis to Parent: %f; %f; %f (x, y, z)", axisToParent.getOrigin().getX(), axisToParent.getOrigin().getY(), axisToParent.getOrigin().getZ());
    tfListener.lookupTransform(childFrame, sensorFrame, ros::Time(0), childToSensor);
    double y = shift + axisToParent.getOrigin().getY() - childToSensor.getOrigin().getY();
    double z = elevation + axisToParent.getOrigin().getZ() - childToSensor.getOrigin().getZ();
    tfScalar ctsYaw, ctsPitch, ctsRoll, atpYaw, atpPitch, atpRoll;
    tf::Matrix3x3 ctsRotMat(childToSensor.getRotation());
    ctsRotMat.getEulerYPR(ctsYaw, ctsPitch, ctsRoll);
    tf::Matrix3x3 atpRotMat(childToSensor.getRotation());
    atpRotMat.getEulerYPR(atpYaw, atpPitch, atpRoll);

    currentTransform.setOrigin(tf::Vector3(currentTransform.getOrigin().getX(), y, z));
    tf::Quaternion rq;
    rq.setRPY(0,
              pitch, // + atpPitch,
              -yaw); // + atpYaw);
    currentTransform.setRotation(rq);
  }

  double getAvgDepth(const Point2f& point, int kernelSize)
  {
    double sum = 0, curVal;
    int cnt = 0;
    double xPos = point.x * depthImg.cols / colorImg.cols;
    double yPos = point.y * depthImg.rows / colorImg.rows;
    for (int i = (int)floor(xPos + 0.5) - kernelSize + 1; i < (int)floor(xPos + 0.5) + kernelSize; i++)
    {
      for (int j = (int)floor(yPos + 0.5) - kernelSize + 1; j < (int)floor(yPos + 0.5) + kernelSize; j++)
      {
        curVal = depthImg.at<float>(j, i);
        if (curVal > 0.1)
        {
          sum += curVal;
          cnt ++;
        }
      }
    }
    return sum / max(cnt, 1);
  }

  void calcRowSlopes(const Point2f pMatrix[PATTERNWIDTH][PATTERNHEIGHT], const float dMatrix[PATTERNWIDTH][PATTERNHEIGHT],
                     float results[PATTERNHEIGHT])
  {
    float x, d;
    for (int i = 0; i < PATTERNHEIGHT; i++)
    {
      // rows
      // Summation of all X and Y values
      double sumX = 0;
      double sumY = 0;
      // Summation of all X*Y values
      double sumXY = 0;
      // Summation of all X^2 and Y^2 values
      double sumXs = 0;
      double sumYs = 0;

      for (int j = 0; j < PATTERNWIDTH; j++)
      {
        x = pMatrix[j][i].x;
        d = dMatrix[j][i];
        sumX = sumX + x;
        sumY = sumY + d;

        sumXY = sumXY + (x * d);

        sumXs = sumXs + (x * x);
        sumYs = sumYs + (d * d);
      }
      double Xs = sumX * sumX;
      //double Ys = sumY * sumY;

      // Calculate slope, m
      float slope = (PATTERNWIDTH * sumXY - sumX * sumY) / (PATTERNWIDTH * sumXs - Xs);
      results[i] = slope;
    }
  }

  void calcColumnSlopes(const Point2f pMatrix[PATTERNWIDTH][PATTERNHEIGHT], const float dMatrix[PATTERNWIDTH][PATTERNHEIGHT],
                       float results[PATTERNWIDTH])
    {
      float x, d;
      for (int i = 0; i < PATTERNWIDTH; i++)
      {
        // columns
        // Summation of all X and Y values
        double sumX = 0;
        double sumY = 0;
        // Summation of all X*Y values
        double sumXY = 0;
        // Summation of all X^2 and Y^2 values
        double sumXs = 0;
        double sumYs = 0;

        for (int j = 0; j < PATTERNHEIGHT; j++)
        {
          x = pMatrix[i][j].y;
          d = dMatrix[i][j];
          sumX = sumX + x;
          sumY = sumY + d;

          sumXY = sumXY + (x * d);

          sumXs = sumXs + (x * x);
          sumYs = sumYs + (d * d);
        }
        double Xs = sumX * sumX;
        //double Ys = sumY * sumY;

        // Calculate slope, m
        float slope = (PATTERNHEIGHT * sumXY - sumX * sumY) / (PATTERNHEIGHT * sumXs - Xs);
        results[i] = slope;
      }
    }

  bool rotateSingleJoint(int jointId, double targetAngle, int waitTime)
  {
    joint_motion_service::move_joints_service srvCall;
    srvCall.request.joint_ids.push_back(jointId);
    srvCall.request.positions.push_back(targetAngle);
    ROS_INFO("Calling service");
    jointMotionServiceClient.call(srvCall);
    ROS_INFO("Service call finished");
    ros::Duration dur(waitTime / 1000.0);
    try
    {
      dur.sleep();
    }
    catch (int e)
    {
      ROS_ERROR("Exception while sleeping: %d", e);
    }
    ROS_INFO("Service sleep finished");
    return srvCall.response.positions_ok != 0;
  }

  void readTransformFromFile()
  {
    TiXmlDocument conf(calibrationFile);
    bool loadOkay = conf.LoadFile();

    if (loadOkay)
    {
      TiXmlElement* transform = conf.FirstChildElement("transform");
      if (transform)
      {
        TiXmlElement* origin = transform->FirstChildElement("origin");
        if (origin)
        {
          double x,y,z;
          origin->QueryDoubleAttribute("x", &x);
          origin->QueryDoubleAttribute("y", &y);
          origin->QueryDoubleAttribute("z", &z);
          currentTransform.setOrigin(tf::Vector3(x, y, z));
        }
        TiXmlElement* rotation = transform->FirstChildElement("rotation");
        if (rotation)
        {
          double roll, pitch, yaw;
          rotation->QueryDoubleAttribute("yaw", &yaw);
          rotation->QueryDoubleAttribute("pitch", &pitch);
          rotation->QueryDoubleAttribute("roll", &roll);
          tf::Quaternion q;
          q.setRPY(roll, pitch, yaw);
          currentTransform.setRotation(q);
        }
      }
      ROS_INFO("Transform loaded.");
    }
    else
    {
      ROS_INFO("Unable to load transform from file. Using standard transform...");
      currentTransform.setOrigin(tf::Vector3(tfXOffset, 0, 0));
      tf::Quaternion q;
      q.setRPY(0, 0, 0);
      currentTransform.setRotation(q);
    }
  }

  void writeTransformToFile()
  {
    TiXmlDocument doc;
    TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
    doc.LinkEndChild(decl);

    TiXmlElement * root = new TiXmlElement("transform");
    doc.LinkEndChild(root);

    TiXmlElement * origin = new TiXmlElement( "origin" );
    origin->SetDoubleAttribute("x", currentTransform.getOrigin().getX());
    origin->SetDoubleAttribute("y", currentTransform.getOrigin().getY());
    origin->SetDoubleAttribute("z", currentTransform.getOrigin().getZ());
    root->LinkEndChild(origin);

    TiXmlElement * rotation = new TiXmlElement( "rotation" );
    tf::Matrix3x3 rotMat(currentTransform.getRotation());
    tfScalar yaw, pitch, roll;
    rotMat.getEulerYPR(yaw, pitch, roll);
    rotation->SetDoubleAttribute("yaw", yaw);
    rotation->SetDoubleAttribute("pitch", pitch);
    rotation->SetDoubleAttribute("roll", roll);
    root->LinkEndChild(rotation);

    ROS_INFO("Writing to file: %s", calibrationFile.c_str());
    if (doc.SaveFile(calibrationFile))
    {
      ROS_INFO("Writing successful.");
    }
    else
    {
      ROS_INFO("Writing failed.");
    }
  }

  void publishTransform ()
  {
    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(currentTransform, ros::Time::now(), parentFrame, childFrame));
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinect_hand_calibration");
  KinectHandCalibration ic;

  ros::Rate loop_rate(20);

  while (ros::ok())
  {
    ic.publishTransform();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
