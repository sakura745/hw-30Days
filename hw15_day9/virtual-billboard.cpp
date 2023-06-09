#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

struct userdata{
    Mat im;
    vector<Point2f> points;
};

void mouseHandler(int event, int x, int y, int flags, void* data_ptr);

//计算四个点的质心
Point2f getCentroid(const vector<Point2f>& point_seq) ;

//将质心作为圆心得到相对坐标
Point2f relCoords(const Point2f& raw_point, const Point2f& centroid);

//用于自定义的坐标系在比较函数cmp中比较
int getQuadrant(const Point2f& p);

int main( int argc, char** argv)
{

    // Read in the image.
    //Mat im_src = imread("first-image.jpg");
    Mat im_src = imread("../myPic.jpg");
    Size size = im_src.size();

    // Create a vector of points.
    vector<Point2f>  pts_src { Point2f(0, 0),
                               Point2f(size.width - 1, 0),
                               Point2f(size.width - 1, size.height -1),
                               Point2f(0, size.height - 1) };

    // Destination image
    //Mat im_dst = imread("times-square.jpg");
    Mat im_dst = imread("../ad.jpg");

    // Set data for mouse handler
    Mat im_temp = im_dst.clone();
    userdata data;
    data.im = im_temp;

    //show the image
    imshow("Image", im_temp);

    cout << "Click on four corners of a billboard and then press ENTER" << endl;

    //set the callback function for any mouse event
    setMouseCallback("Image", mouseHandler, &data);
    waitKey(0);

    // ----------  开始你的代码  --------------

    //将四个点按照顺时针的顺序排序
    sort(data.points.begin(), data.points.end(),
         [&](Point2f& p1, Point2f& p2) {
             return ( getQuadrant(relCoords(p1, getCentroid(data.points)))
                   <  getQuadrant(relCoords(p2, getCentroid(data.points))) );
         }
    );

    //求Homographymatrix
    Mat Homo = findHomography(pts_src,data.points);

    //将src_pic转化为tmp_pic的角度
    warpPerspective(im_src, im_temp, Homo, im_temp.size());

    //定义满足fillConvexPoly的参数
    Point pts[4];
    for (int index = 0; index < 4; index++ ) {
        pts[index] = data.points[index];
    }

    //在dst_pic上把pts位置填充
    fillConvexPoly(im_dst, pts, 4, Scalar(0,0,0));

    im_dst = im_dst + im_temp;
    // ----------  结束你的代码  --------------

    // Display image.
    imshow("Image", im_dst);
    waitKey(0);

    return 0;
}

void mouseHandler(int event, int x, int y, int flags, void* data_ptr)
{
    if  ( event == EVENT_LBUTTONDOWN )
    {

        userdata *data = (userdata *) data_ptr;

        circle(data->im, Point(x,y),3,Scalar(0,255,255), 5, CV_AA);
        imshow("Image", data->im);
        if (data->points.size() < 4)
        {
            data->points.push_back(Point2f(x,y));
        }
    }

}

Point2f getCentroid(const vector<Point2f>& point_seq) {
    Point2f centroid(0, 0);

    for (auto& i : point_seq) {
        centroid.x += i.x;
        centroid.y += i.y;
    }

    centroid.x = centroid.x / point_seq.size();
    centroid.y = centroid.y / point_seq.size();

    return centroid;
}


Point2f relCoords(const Point2f& raw_point, const Point2f& centroid){
    Point2f relCoord(0, 0);
    relCoord.x = raw_point.x - centroid.x;
    relCoord.y = raw_point.y - centroid.y;
    return relCoord;
}

int getQuadrant(const Point2f& p){
    double angle = atan2(p.y, p.x);

    if ( p.x > 0 && angle > 0 )
        return 3;
    else if ( p.x < 0 && angle > 0 )
        return 4;
    else if ( p.x < 0 && angle < 0 )
        return 1;
    return 2;
}