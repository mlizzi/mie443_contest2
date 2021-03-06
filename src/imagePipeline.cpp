#include <imagePipeline.h>

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

#include <iostream>

#include "opencv2/core.hpp"
//#ifdef HAVE_OPENCV_XFEATURES2D 
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

using namespace cv;
using namespace cv::xfeatures2d;


ImagePipeline::ImagePipeline(ros::NodeHandle &n) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    try {
        if (isValid) {
            img.release();
        }
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
        cvtColor(img, img, CV_BGR2GRAY);
        //imshow("greyscale_img",img);
        GaussianBlur(img, img, Size(3, 3), 0);
        isValid = true;
    } catch (cv_bridge::Exception &e) {
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                  << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }
}

int ImagePipeline::getTemplateID(Boxes &boxes) {
    int template_id = -1;
    if (!isValid) {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    } else if (img.empty() || img.rows <= 0 || img.cols <= 0) {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    } else {
        int num_matches[boxes.templates.size()];
        int minHessian = 650;

        // Calculate keypoints for given images and compare descriptors
        for (int i = 0; i < boxes.templates.size(); ++i) {
            cv::resize(boxes.templates[i], boxes.templates[i], cv::Size(500,400));
            Ptr <SURF> detector = SURF::create(minHessian);
            std::vector <KeyPoint> keypoints_object, keypoints_box;
            Mat descriptors_object, descriptors_box;
            Mat img_object = boxes.templates[i];
            detector->detectAndCompute(boxes.templates[i], noArray(), keypoints_object, descriptors_object);
            detector->detectAndCompute(img, noArray(), keypoints_box, descriptors_box);

            //-- Step 2: Matcher estimates which keypoints are most likely matches
            Ptr <DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
            std::vector <std::vector<DMatch>> knn_matches;
            matcher->knnMatch(descriptors_object, descriptors_box, knn_matches, 2);

            //-- Filter matches using the Lowe's ratio test
            const float ratio_thresh = 0.55f;

            //-- Step 3: 1st and 2nd nearest neighbours 
            std::vector <DMatch> good_matches;
            for (size_t i = 0; i < knn_matches.size(); i++) {
                if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
                    good_matches.push_back(knn_matches[i][0]);
                }
            }

            // Add number of good matches to num_matches array
            num_matches[i] = good_matches.size();
            std::cout << num_matches[i] << std::endl;

            // Draw matches and click any button to cycle through images (can uncomment below for testing)
            /*
            Mat img_matches;
            drawMatches(boxes.templates[i], keypoints_object, img, keypoints_box,
                        good_matches, img_matches, Scalar::all(-1),
                        Scalar::all(-1), std::vector<char>(),
                        DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);


            resize(img_matches, img_matches, Size(img_matches.cols/2, img_matches.rows/2));
            imshow("Good Matches & Object detection", img_matches);
            waitKey(0);
            */
        }


        // Find index in num_matches with most matches
        int maxMatches = 0;
        int maxIndex = -1;
        for (int i = 0; i < boxes.templates.size(); ++i) {
            if (num_matches[i] > maxMatches) {
                maxMatches = num_matches[i];
                maxIndex = i;
            }
        }

        int best_match = maxIndex;

        // Set threshold minimum number of matches and return tag_id of found image; returns -1 for no match found.
        int minMatches = 20;
        if (num_matches[best_match] < minMatches) {
            std::cout << "ERROR: Not enough matches!" << std::endl;
            return -1;
        } else {
            std::cout << maxMatches << " matches found!" << std::endl;
            template_id = best_match + 1;
        }
        cv::waitKey(10);
    }

    return template_id;
}