#include <iostream>
//#include "benchmark.h"

#include "util.h"
#include "AStar2.h"


#include <opencv2/core.hpp>


#include <iostream>
// drawing shapes
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <unistd.h>


#include "ros_navigation/navigation_yck.h" 
#include "geometry_msgs/Point.h"
#include <vector>


#include "ros/ros.h"


using namespace cv;
using namespace std;



struct Coordinates {
    int  x;
    int  y;
};



int draw_path(auto path) {

// Path of the image file
	//Mat image = imread("/home/yck/Desktop/astar-gridmap-2d/test/VectorMapBuilder.png",IMREAD_COLOR);
    Mat image = imread("/home/yck/Desktop/astar-gridmap-2d_KullaN/data/VectorMapBuilder.pgm",IMREAD_UNCHANGED);
    //Mat img = Mat::zeros(600,600,CV_8UC3);
    //Mat image = imread("/home/yck/Desktop/astar-gridmap-2d/data/VectorMapBuilder.pgm",cv::IMREAD_GRAYSCALE);
    //image.convertTo(image, CV_8U, 255 / 255);
    normalize(image, image, 0, 255, NORM_MINMAX);
    //image.convertTo(image, CV_8U,255.0 / 4096.0);
    image.convertTo(image, CV_8U);
    
    cout << "Image Width : " << image.cols << endl;
    cout << "Image Height: " << image.rows << endl;
	
    if (!image.data) {
		std::cout << "Could not open or "
					"find the image";
		return 0;
	}
    
    
	int thickness = 1;
    reverse(path.begin(), path.end()); // Kapatildi
	
    // Line drawn using 8 connected
	// Bresenham algorithm
	for(int i=0;i<path.size()-1;i++) 
	{
	

    int x1=path[i].x;
    int y1=path[i].y;
    int x2=path[i+1].x;
    int y2=path[i+1].y;
    Point Point1(x1, y1);
    Point Point2(x2, y2);
    line(image, Point1, Point2, Scalar(0, 255, 0),thickness);
    
	imshow("Output", image);
   
	// Wait for 100 milliseconds (adjust as desired)
    int key = waitKey(50);

	// Exit the loop if the 'q' key is pressed
        if (key == 'q' || key == 'Q')
            break;
	
	
	
	}

    return 0;
   


}



int main(int argc, char **argv)
{
    
    AStar::PathFinder generator;
    Image image;
    image.readFromPGM("/home/yck/Desktop/astar-gridmap-2d_KullaN/data/VectorMapBuilder.pgm");
    generator.setWorldData(image.width(), image.height(), image.data() );


    AStar::Coord2D startPos (140, 190);
    //AStar::Coord2D targetPos(160, 160);
    AStar::Coord2D targetPos(470,190);

    int bidrectional=0; // 0: sadece tek yon, 1: U turn aktif         
    auto path = generator.findPath(startPos, targetPos, bidrectional);
    
      // ROS objects
  ros::init(argc, argv, "publisher");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<ros_navigation::navigation_yck>("AV_navigation", 1);
  ros::Rate loop_rate(0.5);
  
   // the message to be published
  ros_navigation::navigation_yck msg;
  msg.another_field = 0;
  
    msg.points.clear();
    
   
    for (int i=0; i <path.size(); i++) {
        geometry_msgs::Point points_xy;
        points_xy.x = path[i].x;
        points_xy.y = path[i].y;;
        msg.points.push_back(points_xy);
    }
    
  
 
  int count = 0;
  while (ros::ok())
  {
    
    msg.another_field = count;
    
  

    ROS_INFO( "Counter %d", msg.another_field);

    pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
    
  }
    
    
    
   
    draw_path(path);
   //generator.exportPPM("yck_map_out_large.ppm", &result );
    generator.exportPPM("yck_mapp_black.ppm", &path );
    return 0;
}

