# srrc_simple_obj_detector

Simplest object detector.
It turns out that the following simple function is already a good detector for srrc samples we have:

  ```C++
  cv::Mat makeObjectMask( cv::Mat const& src_bgr_image
                        , int saturation_threshold = 168
                        , int brightness_threshold = 20)
  {
    cv::Mat hsv_image = src_bgr_image.clone();
    cv::cvtColor(hsv_image, hsv_image, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> hsv_channels;
    cv::split(hsv_image, hsv_channels);
    return (hsv_channels[1] < saturation_threshold) & (hsv_channels[2] >= brightness_threshold);
  }
  ```    



## Examples of results:
every blob that's not exactly black is an object candidate. no farther filtering used.

![](frame0077.jpg "precache partially in shadow")
![](frame0155.jpg "pink rock partially in shadow")

brief usage for processing jpg files:

0. check out this repo into your catkin_ws and `catkin_make`
1. open file `test_simple_detector_node.launch`
2. replace value for parameter `src_image_src_folder` with path to your folder containing jpg files
3. replace value for parameter `output_folder` with the path you want to output the processed images to (it will be created if it does not exist)
4. call `roslaunch srrc_simple_obj_detector test_simple_detector_node.launch` and wait.
