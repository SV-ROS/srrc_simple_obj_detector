#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <cv_bridge/cv_bridge.h>
//#include <image_transport/image_transport.h>
//#include <sensor_msgs/image_encodings.h>

#include "boost/date_time/posix_time/posix_time.hpp" //include all types plus i/o
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

inline cv::Mat makeObjectMask(cv::Mat const& src_bgr_image, int saturation_threshold, int brightness_threshold) {
    cv::Mat hsv_image = src_bgr_image.clone();
    cv::cvtColor(hsv_image, hsv_image, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> hsv_channels;
    cv::split(hsv_image, hsv_channels);
    return (hsv_channels[1] < saturation_threshold) & (hsv_channels[2] >= brightness_threshold);
}

class TestSimpleImageProcessor {
public:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    int saturation_threshold_;
    int brightness_threshold_;

    bool show_images_;
    boost::filesystem::path output_folder_;


    TestSimpleImageProcessor() : private_nh_("~") {
        show_images_ = false;
        updateParam("show_images", show_images_);

        saturation_threshold_ = 168;
        updateParam("saturation_threshold", saturation_threshold_);

        brightness_threshold_ = 30;
        updateParam("brightness_threshold", brightness_threshold_);

        std::string image_trg_folder_name = "";
        updateParam("output_folder", image_trg_folder_name);
        initOutputFolder(image_trg_folder_name);
    }

    void processImagesFromFolder(std::string const& image_folder_path) {
        int num_of_images = countImagesInFolder(image_folder_path);
        ROS_INFO("TRACE(test_simple_image_process): processing folder %s. \n    found %d images", image_folder_path.c_str(), num_of_images);
        boost::filesystem::directory_iterator end_itr;
        boost::filesystem::directory_iterator input_image_itr = findNextImage(boost::filesystem::directory_iterator(image_folder_path));
        int image_index = 0;
        while(input_image_itr != end_itr) {
            processImage(input_image_itr->path().string(), num_of_images, ++image_index);
            input_image_itr = findNextImage(++input_image_itr);
        }
        ROS_INFO("TRACE(test_simple_image_process): Done with processing images.");
    }

    void processImage(std::string const& image_file_name, int num_of_images = 1, int image_index = 1) {
        ROS_INFO("TRACE(test_simple_image_process): processing image [%d of %d] %s", image_index, num_of_images, image_file_name.c_str());
        //boost::posix_time::ptime time0 = boost::posix_time::microsec_clock::local_time();
        if(!boost::filesystem::exists(image_file_name)) {
            ROS_ERROR("ERROR(test_simple_image_process): file %s does not exist.", image_file_name.c_str());
            return;
        }
        cv::Mat src_image = cv::imread(image_file_name, CV_LOAD_IMAGE_COLOR);
//        cv::imshow("original image", src_image_.image);
//        cv::waitKey();
        boost::posix_time::ptime time1 = boost::posix_time::microsec_clock::local_time();
        cv::Mat obj_mask = makeObjectMask(src_image, saturation_threshold_, brightness_threshold_);
        boost::posix_time::ptime time2 = boost::posix_time::microsec_clock::local_time();
        ROS_INFO("TRACE(test_simple_image_process):         done in %d microsec.", (int)(time2 - time1).total_microseconds());
        makeResultImage(image_file_name, src_image, obj_mask);
    }

    template<typename t_Param>
    void updateParam(std::string const& param_name, t_Param& param) {
        t_Param old = param;
        private_nh_.param(param_name, param, old);
    }

private:
    void makeResultImage(std::string const& image_file_name, cv::Mat const& src_image, cv::Mat const& obj_mask) {
        int rows = src_image.size().height;
        int cols = src_image.size().width;
        cv::Mat stitch = cv::Mat(rows, 2 * cols + 2, CV_8UC3, cv::Scalar(0));

        //: left: original image:
        src_image.copyTo(stitch.colRange(0, cols).rowRange(0, rows));

        //: right: highlited object:
        {
          std::vector<cv::Mat> bgr_channels;
          cv::split(src_image, bgr_channels);
          bgr_channels[0] &= obj_mask;
          bgr_channels[1] &= obj_mask;
          bgr_channels[2] &= obj_mask;
          cv::Mat res_image;
          cv::merge(bgr_channels, res_image);
          res_image.copyTo(stitch.colRange(cols + 1, 2 * cols + 1).rowRange(0, rows));
        }

        if(!output_folder_.empty()) {
          boost::filesystem::path image_output_path = output_folder_ / boost::filesystem::path(image_file_name).filename();
          cv::imwrite(image_output_path.string(), stitch);
          ROS_INFO("TRACE(test_simple_image_process):         saved image %s", image_output_path.c_str());
        }
        if(show_images_) {
            cv::imshow("clustered image", stitch);
            cv::waitKey();
        }
    }

private:
    void initOutputFolder(std::string const& root_output_folder) {
        if(root_output_folder.empty())
            return;
        output_folder_ = root_output_folder;
        boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
        std::string generated_folder_name = boost::posix_time::to_iso_extended_string(now);
        std::string chars_to_replace = " ,:";
        BOOST_FOREACH(char& ch, generated_folder_name) {
            if(chars_to_replace.find(ch) != std::string::npos)
                ch = '_';
        }
        output_folder_ /= generated_folder_name;
        boost::filesystem::create_directories(output_folder_);
        ROS_INFO("TRACE(test_simple_image_process): see output images in %s", output_folder_.c_str());
    }

    static boost::filesystem::directory_iterator findNextImage(boost::filesystem::directory_iterator input_image_itr) {
        boost::filesystem::directory_iterator end_itr;
        while(input_image_itr != end_itr
            && !boost::filesystem::is_regular_file(input_image_itr->path())
            && input_image_itr->path().string().size() < 4
            && input_image_itr->path().string().substr(input_image_itr->path().string().size() - 4, 4) != ".jpg"
            ) ++input_image_itr;
        return input_image_itr;
    }

    int countImagesInFolder(std::string const& image_folder_path) {
        boost::filesystem::directory_iterator end_itr;
        boost::filesystem::directory_iterator input_image_itr = findNextImage(boost::filesystem::directory_iterator(image_folder_path));
        int res = 0;
        while(input_image_itr != end_itr) {
            input_image_itr = findNextImage(++input_image_itr);
            ++res;
        }
        return res;
    }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_simple_detector_node");
  TestSimpleImageProcessor node;

  std::string image_file_name = "";
  node.updateParam("src_image_file", image_file_name);
  std::string image_src_folder_name = "";
  node.updateParam("src_image_src_folder", image_src_folder_name);

  if(!image_file_name.empty())
      node.processImage(image_file_name);
  else
      node.processImagesFromFolder(image_src_folder_name);
  return 0;
}
