

#include <iostream>
#include <vector>
#include "vfc.h"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// 定义ORB特征参数
#define ORB_N_FEATURE				1000	// 需要提取的特征点数目
#define ORB_N_OCTAVE_LAYERS			8		// 8, default value
#define ORB_FAST_THRESHOLD			20		// 20, default value
#define ORB_EDGE_THRESHOLD			31		// 31, default value
#define ORB_PATCH_SIZE				31		// 31, default value
#define ORB_SCALE					1.2		// the default value 1.2 seems not so good, for only features locate on the corners always leads to a weak tracking

int main()
{

    // 读取两张待匹配图片
    Mat image1 = imread( "../image/tsukuba_1.png");
    Mat image2 = imread( "../image/tsukuba_2.png");

    // 提取ORB特征点
    std::vector<KeyPoint> keypoints1, keypoints2;
    Mat descriptors1, descriptors2;

    Ptr<ORB> orb = ORB::create(ORB_N_FEATURE);
    orb->setFastThreshold(ORB_FAST_THRESHOLD);
    orb->setEdgeThreshold(ORB_EDGE_THRESHOLD);
    orb->setPatchSize(ORB_PATCH_SIZE);
    orb->setNLevels(ORB_N_OCTAVE_LAYERS);
    orb->setScaleFactor(ORB_SCALE);
    orb->setMaxFeatures(ORB_N_FEATURE);
    orb->setWTA_K(2);
    orb->setScoreType(ORB::HARRIS_SCORE); // HARRIS_SCORE, also has FAST_SCORE
    orb->detectAndCompute(image1, Mat(), keypoints1, descriptors1);
    orb->detectAndCompute(image2, Mat(), keypoints2, descriptors2);

    //绘制提取到的特征点
    Mat featureImgAll, featureImg1,featureImg2;
    cout << "# orb keypoints number: " << keypoints1.size()  << endl;
    image1.copyTo(featureImg1);
    image2.copyTo(featureImg2);
    drawKeypoints(image1, keypoints1, featureImg1, Scalar::all(-1));
    drawKeypoints(image2, keypoints2, featureImg2, Scalar::all(-1));
    hconcat(featureImg1, featureImg2, featureImgAll);
    putText(featureImgAll, "ORB keypoints", Point(20, 20), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2);
    imshow("ORB keypoints", featureImgAll);



    // ---------------  特征点匹配方法暴力匹配（Brute Force）  --------------------------
    // 暴力匹配法，直接找距离最近的点
    double t = (double)getTickCount();
    BFMatcher matcher_bf(NORM_HAMMING, true); //使用汉明距离度量二进制描述子，允许交叉验证
    vector<DMatch> Matches_bf;
    matcher_bf.match(descriptors1, descriptors2, Matches_bf);
    t = 1000 * ((double)getTickCount() - t) / getTickFrequency();

    assert(Matches_bf.size() > 0);
    cout << "# Brute Force Matches: " << Matches_bf.size() << "/" << keypoints1.size() << ", Times ="<<t<<" ms " << endl;
    Mat BF_img;
    drawMatches(image1, keypoints1, image2, keypoints2, Matches_bf, BF_img);
    resize(BF_img, BF_img, Size(2*image1.cols, image1.rows));
    putText(BF_img, "Brute Force Matches", Point(20, 20), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2);
    imshow("Brute Force Matches", BF_img);



    // ---------------  矢量场一致性(VFC)筛选特征点匹配对 ----------------------------
    // 参考：Robust Point Matching via Vector Field Consensus", IEEE Transactions on Image Processing, 2014

    // 数据格式预处理
    vector<Point2f> X;
    vector<Point2f> Y;
    X.clear();
    Y.clear();

 	// -------------开始代码-------------
    // 将Matches_bf里的匹配点对分别放到X,Y 向量里，约5行代码，参考OpenCV DMatch类
    for (auto& i : Matches_bf) {
        X.push_back(keypoints1[i.queryIdx].pt);
        Y.push_back(keypoints2[i.trainIdx].pt);
    }

    // -------------结束代码-------------

    // 调用VFC主函数
    t = (double)getTickCount();
    VFC myvfc;
    myvfc.setData(X, Y);
    myvfc.optimize();
    vector<int> matchIdx = myvfc.obtainCorrectMatch();
    t = 1000 * ((double)getTickCount() - t) / getTickFrequency();

    // 筛选正确的匹配
    std::vector< DMatch > Matches_VFC;
    for (unsigned int i = 0; i < matchIdx.size(); i++) {
    int idx = matchIdx[i];
        Matches_VFC.push_back(Matches_bf[idx]);
    }

    cout << "# Refine Matches (after VFC): " << Matches_VFC.size() << "/" << Matches_bf.size() << ", Times ="<<t<<" ms " << endl;

    // 绘制筛选结果
    Mat imageVFC;
    drawMatches(image1, keypoints1, image2, keypoints2, Matches_VFC, imageVFC);
    resize(imageVFC, imageVFC, Size(2*image1.cols, image1.rows));
    putText(imageVFC, "VFC Matches", Point(20, 20), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2);
    imshow("VFC Matches", imageVFC);


    waitKey(0);

    destroyAllWindows();
    return 0;
}

