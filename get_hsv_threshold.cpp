#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;

/** Global Variables */
const int max_value_H = 360/2;
const int max_value = 255;
const int max_value_blob = 1000;
const String window_capture_name = "Video Capture";
const String window_detection_name = "color Detection";
const String window_blob_name = "Blob Detection";
int low_H = 0, low_S = 0, low_V = 0, low_convexity = 0, low_circularity = 0, low_inertia = 0;
int high_H = max_value_H, high_S = max_value, high_V = max_value;
int max_convexity = max_value_blob, max_circularity = max_value_blob, max_inertia = max_value_blob;

static void on_low_convexity_thresh_trackbar(int, void *)
{
    low_convexity = min(max_convexity-1, low_convexity);
    setTrackbarPos("Low Convexity", window_blob_name, low_convexity);
}

static void on_high_convexity_thresh_trackbar(int, void *)
{
    max_convexity = max(max_convexity, low_convexity+1);
    setTrackbarPos("High Convexity", window_blob_name, max_convexity);
}

static void on_low_circularity_thresh_trackbar(int, void *)
{
    low_circularity = min(max_circularity-1, low_circularity);
    setTrackbarPos("Low Circularity", window_blob_name, low_circularity);
}

static void on_high_circularity_thresh_trackbar(int, void *)
{
    max_circularity = max(max_circularity, low_circularity+1);
    setTrackbarPos("High Circularity", window_blob_name, max_circularity);
}

static void on_low_inertia_thresh_trackbar(int, void *)
{
    low_inertia = min(max_inertia-1, low_inertia);
    setTrackbarPos("Low Inertia", window_blob_name, low_inertia);
}

static void on_high_inertia_thresh_trackbar(int, void *)
{
    max_inertia = max(max_inertia, low_inertia+1);
    setTrackbarPos("High Inertia", window_blob_name, max_inertia);
}



//! [low]
static void on_low_H_thresh_trackbar(int, void *)
{
    low_H = min(high_H-1, low_H);
    setTrackbarPos("Low H", window_detection_name, low_H);
}
//! [low]

//! [high]
static void on_high_H_thresh_trackbar(int, void *)
{
    high_H = max(high_H, low_H+1);
    setTrackbarPos("High H", window_detection_name, high_H);
}

//! [high]
static void on_low_S_thresh_trackbar(int, void *)
{
    low_S = min(high_S-1, low_S);
    setTrackbarPos("Low S", window_detection_name, low_S);
}

static void on_high_S_thresh_trackbar(int, void *)
{
    high_S = max(high_S, low_S+1);
    setTrackbarPos("High S", window_detection_name, high_S);
}

static void on_low_V_thresh_trackbar(int, void *)
{
    low_V = min(high_V-1, low_V);
    setTrackbarPos("Low V", window_detection_name, low_V);
}

static void on_high_V_thresh_trackbar(int, void *)
{
    high_V = max(high_V, low_V+1);
    setTrackbarPos("High V", window_detection_name, high_V);
}

int main(int argc, char* argv[])
{
    //! [cap]
    VideoCapture cap(argc > 1 ? atoi(argv[1]) : 0);
    cap.set(CAP_PROP_FRAME_WIDTH, 640);  // Set the width
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);
    //! [cap]

    //! [window]
    namedWindow(window_capture_name);
    namedWindow(window_detection_name);
    namedWindow(window_blob_name);
    //! [window]

    //! [trackbar]
    // Trackbars to set thresholds for HSV values
    createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
    createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
    createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
    createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
    createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
    createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);
    //! [trackbar]


    //! [trackbar_blob]
    // Trackbars to set thresholds for blob properties
    createTrackbar("Low Convexity", window_blob_name, &low_convexity, max_convexity, on_low_convexity_thresh_trackbar);
    createTrackbar("High Convexity", window_blob_name, &max_convexity, max_convexity, on_high_convexity_thresh_trackbar);
    createTrackbar("Low Circularity", window_blob_name, &low_circularity, max_circularity, on_low_circularity_thresh_trackbar);
    createTrackbar("High Circularity", window_blob_name, &max_circularity, max_circularity, on_high_circularity_thresh_trackbar);
    createTrackbar("Low Inertia", window_blob_name, &low_inertia, max_inertia, on_low_inertia_thresh_trackbar);
    createTrackbar("High Inertia", window_blob_name, &max_inertia, max_inertia, on_high_inertia_thresh_trackbar);
    //! [trackbar_blob]

    Mat frame, frame_HSV, frame_threshold, flippedFrame;
    while (true) {
        //! [while]
        cap >> frame;
        if(frame.empty())
        {
            break;
        }
        // Flip frame so it's normal
        // Convert from BGR to HSV colorspace
        cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
        // Detect the object based on HSV Range Values
        inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);
        //! [while]

        //! [blob]
        // Setup SimpleBlobDetector parameters.
        SimpleBlobDetector::Params params;
        // Change thresholds
        params.filterByColor = true;
        params.blobColor = 255; // White blobs
        // Filter by Area.
        params.filterByArea = true;
        params.minArea = 1000;
        params.maxArea = 100000;
        // Filter by Convexity
        params.filterByConvexity = true;
        params.minConvexity = low_convexity/1000.0;
        params.maxConvexity = max_convexity/1000.0;
        // Filter by Circularity
        params.filterByCircularity = true;
        params.minCircularity = low_circularity/1000.0;
        params.maxCircularity = max_circularity/1000.0;
        // Filter by Inertia
        params.filterByInertia = true;
        params.minInertiaRatio = low_inertia/1000.0;
        params.maxInertiaRatio = max_inertia/1000.0;
        // Set up the detector with default parameters.
        Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
        // Detect blobs.
        std::vector<KeyPoint> keypoints;
        detector->detect( frame_threshold, keypoints);
        // Draw detected blobs as red circles.
        // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
        Mat im_with_keypoints;
        drawKeypoints( frame, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
        //! [blob]


        //! [show]
        // Show the frames
        imshow(window_capture_name, frame);
        imshow(window_detection_name, frame_threshold);
        imshow(window_blob_name, im_with_keypoints);
        //! [show]

        char key = (char) waitKey(30);
        if (key == 'q' || key == 27)
        {
            break;
        }
    }
    return 0;
}