
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void
clusterLidarWithROI(std::vector<BoundingBox>& boundingBoxes, std::vector<LidarPoint>& lidarPoints, float shrinkFactor,
                    cv::Mat& P_rect_xx, cv::Mat& R_rect_xx, cv::Mat& RT) {
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1) {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2) {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt)) {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1) {
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox>& boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait) {
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for (auto it1 = boundingBoxes.begin(); it1 != boundingBoxes.end(); ++it1) {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0, 150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top = 1e8, left = 1e8, bottom = 0.0, right = 0.0;
        float xwmin = 1e8, ywmin = 1e8, ywmax = -1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2) {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin < xw ? xwmin : xw;
            ywmin = ywmin < yw ? ywmin : yw;
            ywmax = ywmax > yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top < y ? top : y;
            left = left < x ? left : x;
            bottom = bottom > y ? bottom : y;
            right = right > x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 0, 0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int) it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left - 250, bottom + 50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax - ywmin);
        putText(topviewImg, str2, cv::Point2f(left - 250, bottom + 125), cv::FONT_ITALIC, 2, currColor);
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i) {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if (bWait) {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox& boundingBox, std::vector<cv::KeyPoint>& kptsPrev,
                              std::vector<cv::KeyPoint>& kptsCurr, std::vector<cv::DMatch>& kptMatches) {
    // ...
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint>& kptsPrev, std::vector<cv::KeyPoint>& kptsCurr,
                      std::vector<cv::DMatch> kptMatches, double frameRate, double& TTC, cv::Mat* visImg) {
    double dT = 1. / frameRate;

}

// calculates the distance between the neighboring points along the x axis
// and returns the median distance
// expects sorted vector based on x coordinate
double _getMedianDistance(std::vector<LidarPoint>& lidarPoints) {
    std::vector<double> distances;
    distances.reserve(lidarPoints.size() - 1);
    for (int i = 0; i < lidarPoints.size() - 1; i++) {
        distances.push_back(lidarPoints[i + 1].x - lidarPoints[i].x);
    }
    // sort distances for median extraction
    std::sort(distances.begin(), distances.end());
    return distances.at(static_cast<int>(distances.size() / 2));
}

void computeTTCLidar(std::vector<LidarPoint>& lidarPointsPrev,
                     std::vector<LidarPoint>& lidarPointsCurr, double frameRate, double& TTC) {
    double dT = 1. / frameRate;
    double laneWidth = 4.0;

    double minXPrev = 1e9, minXCurr = 1e9;
    // find lidar points in ego lane in previous frame
    // calculate median distance between two closest points to make point
    // selection more robust against outliers
    auto sortFunc = [](const LidarPoint& p1, const LidarPoint& p2) {
        return p1.x < p2.x;
    };
    std::sort(lidarPointsPrev.begin(), lidarPointsPrev.end(), sortFunc);
    double medianDistPrev = _getMedianDistance(lidarPointsPrev);
    for (auto it=lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it) {
        // only consider points in ego lane
        if (fabs((*it).y) <= laneWidth / 2) {
            // find closest points
            double dist2nearestPoint = (it + 1)->x - it->x;
            if (it->x < minXPrev && dist2nearestPoint <= medianDistPrev) {
                minXPrev = it->x;
                break;
            }
        }
    }

    std::sort(lidarPointsCurr.begin(), lidarPointsCurr.end(), sortFunc);
    double medianDistCurr = _getMedianDistance(lidarPointsCurr);
    for (auto it=lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it) {
        // only consider points in ego lane
        if (fabs((*it).y) <= laneWidth / 2) {
            // find closest points
            double dist2nearestPoint = (it + 1)->x - it->x;
            if (it->x < minXCurr && dist2nearestPoint <= medianDistPrev) {
                minXCurr = it->x;
                break;
            }
        }
    }

    TTC = minXCurr * dT / (minXPrev - minXCurr);
}

// matches bounding boxes between two frames by associating descriptor matches to bbs
void matchBoundingBoxes(std::vector<cv::DMatch>& matches, std::map<int, int>& bbBestMatches, DataFrame& prevFrame,
                        DataFrame& currFrame) {
    // maps bounding box idx in prev frame to all other bounding boxes and the count of their shared points
    // prevFrameId, <currFrameId, countPoints>
    std::map<int, std::map<int, int>> bbMatches;
    // for each match find if there is a connection between the bounding boxes in the two frames
    for (cv::DMatch& match : matches) {
        cv::Point2f& pointPrevFrame = prevFrame.keypoints[match.queryIdx].pt;
        cv::Point2f& pointCurrFrame = currFrame.keypoints[match.trainIdx].pt;
        for (BoundingBox& bBoxPrev : prevFrame.boundingBoxes) {
            if (bBoxPrev.roi.contains(pointPrevFrame)) {
                for (BoundingBox& bBoxCurr : currFrame.boundingBoxes) {
                    if (bBoxCurr.roi.contains(pointCurrFrame)) {
                        // check if this is the first point for this mapping
                        if (bbMatches.count(bBoxPrev.boxID)) {
                            // check whether the mapping combination exists
                            if (bbMatches[bBoxPrev.boxID].count(bBoxCurr.boxID)) {
                                bbMatches[bBoxPrev.boxID][bBoxCurr.boxID] += 1;
                            } else {
                                bbMatches[bBoxPrev.boxID][bBoxCurr.boxID] = 1;
                            }
                        } else { // create new count for this matching
                            bbMatches.insert({bBoxPrev.boxID, std::map<int, int>{{bBoxCurr.boxID, 1}}});
                        }
                    }
                }
            }
        }
    }
    // find the bbox combinations with the highest number of shared points
    for (auto& bbMap : bbMatches) {
        auto bestCombo = std::max_element(bbMap.second.begin(), bbMap.second.end(),
                                          [](const pair<int, int>& p1, const pair<int, int>& p2) {
                                              return p1.second < p2.second;
                                          }
        );
        bbBestMatches[bbMap.first] = bestCombo->first;
    }

}
