#include "boost/date_time/posix_time/posix_time.hpp" //include all types plus i/o
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>



struct ObjMaskMaker {
    struct Params {
        int saturation_threshold_;
        int brightness_threshold_;

        Params(int saturation_threshold = 168, int brightness_threshold = 20)
            : saturation_threshold_(saturation_threshold), brightness_threshold_(brightness_threshold) {}
    };

    Params params_;

    cv::Mat makeObjectMask(cv::Mat const& src_bgr_image) const {
        return makeObjectMask(src_bgr_image, params_.saturation_threshold_, params_.brightness_threshold_);
    }

    static cv::Mat makeObjectMask(cv::Mat const& src_bgr_image, int saturation_threshold, int brightness_threshold) {
        cv::Mat hsv_image = src_bgr_image.clone();
        cv::cvtColor(hsv_image, hsv_image, cv::COLOR_BGR2HSV);
        std::vector<cv::Mat> hsv_channels;
        cv::split(hsv_image, hsv_channels);
//        cv::imshow("saturation", hsv_channels[1]);
//        cv::waitKey();
        return (hsv_channels[1] < saturation_threshold) & (hsv_channels[2] >= brightness_threshold);
    }
};

class BlobDetector {
public:
    struct Params {
        int minSize;
        int maxSize;
        int minArea;
        int maxArea;
        int border_margin;

        Params() {
            minSize = 30;
            maxSize = 200;
            minArea = 600;
            maxArea = 40000;
            border_margin = 3;
        }
    };

    typedef std::vector<cv::Point> Contour;
    typedef std::vector<Contour> Contours;

    Params params_;

    Contours findBlobContours(cv::Mat const& binary_image) const {
        Contours contours;
        cv::Mat const tmp_binary_image = binary_image.clone();
        cv::findContours(tmp_binary_image, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
        Contours result;
        for (size_t contourIdx = 0; contourIdx < contours.size(); contourIdx++) {
            if(checkBbox(contours[contourIdx], binary_image.size()) && checkArea(contours[contourIdx]))
                result.push_back(contours[contourIdx]);
        }
        return result;
    }

private:
    bool checkBbox(Contour const& contour, cv::Size const& image_size) const {
        cv::Rect bbox = cv::boundingRect(contour);
        return (checkSize(bbox.size())
                && params_.border_margin < bbox.tl().x
                && params_.border_margin < bbox.tl().y
                && image_size.width > params_.border_margin + bbox.br().x
                && image_size.height > params_.border_margin + bbox.br().y
               );
    }
    bool checkSize(cv::Size const& contour_size) const {
        bool res = (params_.minSize <= std::min(contour_size.height, contour_size.width)
                    && params_.maxSize > std::max(contour_size.height, contour_size.width));
        //ROS_INFO("contour_size: %d, %d. res: %d", contour_size.height, contour_size.width, res ? 1 : 0);
        return res;
    }
    bool checkArea(Contour const& contour) const {
        double area = cv::contourArea(contour);
        //ROS_INFO("contour_area: %f", area);
        return (params_.minArea <= area && params_.maxArea > area);
    }
};


struct ResultImageMaker {
    struct Params {
        bool use_bw_result_;

        Params(bool use_bw_result = false) : use_bw_result_(use_bw_result) {}
    };

    Params params_;

    cv::Mat makeStitchedResultImage(cv::Mat const& src_image, cv::Mat const& obj_mask, BlobDetector::Contours const& blob_contours) const {
        int rows = src_image.size().height;
        int cols = src_image.size().width;
        cv::Mat stitch = cv::Mat(rows, 2 * cols + 2, CV_8UC3, cv::Scalar(0));

        //: left: original image:
        src_image.copyTo(stitch.colRange(0, cols).rowRange(0, rows));

        //: right: highlited object:
        cv::Mat res_image = makeResultImage(src_image, obj_mask, blob_contours);
        res_image.copyTo(stitch.colRange(cols + 1, 2 * cols + 1).rowRange(0, rows));
        return stitch;
    }

    cv::Mat makeResultImage(cv::Mat const& src_image, cv::Mat const& obj_mask, BlobDetector::Contours const& blob_contours) const {
        cv::Mat res_image;
        std::vector<cv::Mat> bgr_channels;
        cv::split(src_image, bgr_channels);
        if(params_.use_bw_result_) {
            bgr_channels[0] = obj_mask;
            bgr_channels[1] = obj_mask;
            bgr_channels[2] = obj_mask;
            cv::merge(bgr_channels, res_image);
        } else {
            bgr_channels[0] &= obj_mask;
            bgr_channels[1] &= obj_mask;
            bgr_channels[2] &= obj_mask;
            cv::merge(bgr_channels, res_image);
            //: enhance brightness:
            cv::Mat hsv_image;
            cv::cvtColor(res_image, hsv_image, cv::COLOR_BGR2HSV);
            std::vector<cv::Mat> hsv_channels;
            cv::split(hsv_image, hsv_channels);
            hsv_channels[2] = (hsv_channels[2] != 0);
            cv::merge(hsv_channels, hsv_image);
            cv::cvtColor(hsv_image, res_image, cv::COLOR_HSV2BGR);
        }

        for(size_t i = 0; i < blob_contours.size(); ++i) {
            cv::Rect bbox = cv::boundingRect(blob_contours[i]);
            cv::rectangle(res_image, bbox, cv::Scalar(0,0,255));
            cv::circle(res_image, cv::Point((bbox.br().x + bbox.tl().x) / 2, (bbox.br().y + bbox.tl().y) / 2), 3, cv::Scalar(0,0,255));
        }
        return res_image;
    }

};


struct Nh {
    ros::NodeHandle private_nh_;
    ros::NodeHandle nh_;

    Nh() : private_nh_("~"), nh_() {}

    template<typename t_Param>
    static void printParam(std::string const& param_name, t_Param& param, std::string const& prefix = " (*) ") {
        std::stringstream strstr;
        strstr << prefix << param_name << ": " << param;
        std::string text = strstr.str();
        ROS_INFO("%s", text.c_str());
    }

    template<typename t_Param>
    static void updateParam(ros::NodeHandle& nh, std::string const& param_name, t_Param& param) {
        t_Param old = param;
        nh.param(param_name, param, old);
        if(param != old)
            printParam(param_name, param, "updated ");
        else
            printParam(param_name, param, "not updated ");
    }

    template<typename t_Param>
    void updateParam(std::string const& param_name, t_Param& param) {
        updateParam(private_nh_, param_name, param);
    }
};


class ResultImageConsumer {
private:
    boost::filesystem::path output_folder_;

    image_transport::ImageTransport it_;
    image_transport::Publisher res_image_pub_;

public:
    struct Params {
        bool show_images_;

        Params(bool show_images = false) : show_images_(show_images) {}
    };

    Params params_;

    ResultImageConsumer(Nh& nh) : it_(nh.nh_) {
        res_image_pub_ = it_.advertise("/result_image", 1);
        std::string output_folder = "";
        nh.updateParam("output_folder", output_folder);
        initOutputFolder(output_folder);
    }

    bool willUseResultImage() const {
        return !output_folder_.empty()
                || params_.show_images_
                || res_image_pub_.getNumSubscribers() > 0;
    }

    void consumeResultImage(cv::Mat const& res_image, std::string const& image_file_name = "") const {
        if(res_image_pub_.getNumSubscribers() > 0) {
            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", res_image).toImageMsg();
            res_image_pub_.publish(img_msg);
        }
        if(!output_folder_.empty() && !image_file_name.empty()) {
            boost::filesystem::path image_output_path = output_folder_ / boost::filesystem::path(image_file_name).filename();
            cv::imwrite(image_output_path.string(), res_image);
            ROS_INFO("TRACE(test_simple_image_process):         saved image %s", image_output_path.c_str());
        }
        if(params_.show_images_) {
            cv::imshow("result image", res_image);
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
};


class TestSimpleImageProcessor {
private:
    Nh& nh_;

    image_transport::ImageTransport it_;
    image_transport::Subscriber mono_image_sub_;
    ros::Publisher res_pub_;

    ObjMaskMaker mask_maker_;
    ResultImageMaker result_image_maker_;
    BlobDetector blob_detector_;
    ResultImageConsumer result_image_consumer_;

public:
    TestSimpleImageProcessor(Nh& nh)
        : nh_(nh)
        , it_(nh.nh_)
        , result_image_consumer_(nh_)
    {
        //: subscription:
        std::string input_image_transfer_hint     = "compressed";
        std::string input_image_topic             = "/turret_stereo/left/image_raw";
        updateParam("input_image_transfer_hint",   input_image_transfer_hint);
        updateParam("input_image_topic",           input_image_topic);
        mono_image_sub_ = it_.subscribe(input_image_topic, 1, &TestSimpleImageProcessor::monoImageCb, this, image_transport::TransportHints(input_image_transfer_hint));

        res_pub_ = nh.nh_.advertise<geometry_msgs::Point>("detector_result_as_point", 1);

        //: object detection params:
        updateParam("saturation_threshold", mask_maker_.params_.saturation_threshold_);
        updateParam("brightness_threshold", mask_maker_.params_.brightness_threshold_);

        //: candidate filtering params:
        updateParam("blob_filter_min_blob_size", blob_detector_.params_.minSize);
        updateParam("blob_filter_max_blob_size", blob_detector_.params_.maxSize);
        updateParam("blob_filter_min_blob_area", blob_detector_.params_.minArea);
        updateParam("blob_filter_max_blob_area", blob_detector_.params_.maxArea);
        updateParam("blob_filter_border_margin", blob_detector_.params_.border_margin);

        //: other params:
        updateParam("use_bw_result", result_image_maker_.params_.use_bw_result_);
        updateParam("show_images", result_image_consumer_.params_.show_images_);
        if(mono_image_sub_.getNumPublishers() > 0)
            result_image_consumer_.params_.show_images_ = false;
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

    void processImage(std::string const& image_file_name, int num_of_images = 1, int image_index = 1) const {
        ROS_INFO("TRACE(test_simple_image_process): processing image [%d of %d] %s", image_index, num_of_images, image_file_name.c_str());
        //boost::posix_time::ptime time0 = boost::posix_time::microsec_clock::local_time();
        if(!boost::filesystem::exists(image_file_name)) {
            ROS_ERROR("ERROR(test_simple_image_process): file %s does not exist.", image_file_name.c_str());
            return;
        }
        cv::Mat src_image = cv::imread(image_file_name, CV_LOAD_IMAGE_COLOR);
        processImage(src_image, image_file_name);
    }

    void processImage(cv::Mat const& src_image, std::string const& image_file_name = "") const {
        boost::posix_time::ptime time1 = boost::posix_time::microsec_clock::local_time();
        cv::Mat obj_mask = mask_maker_.makeObjectMask(src_image);
        BlobDetector::Contours blob_contours = blob_detector_.findBlobContours(obj_mask);
        boost::posix_time::ptime time2 = boost::posix_time::microsec_clock::local_time();

        if(result_image_consumer_.willUseResultImage()) {
            cv::Mat res_image = result_image_maker_.makeStitchedResultImage(src_image, obj_mask, blob_contours);
            result_image_consumer_.consumeResultImage(res_image, image_file_name);
        }
        boost::posix_time::ptime time3 = boost::posix_time::microsec_clock::local_time();
        ROS_INFO("TRACE(test_simple_image_process):         processed in %f milliseconds, %d blobs found, %fms took drawing debug picture."
                 , (time2 - time1).total_microseconds() / 1000.
                 , (int)blob_contours.size()
                 , (time3 - time2).total_microseconds() / 1000.);

        if(blob_contours.size() > 0) {
            //fixme: publish first candidate point for now:
            //fixme: not good message type
            cv::Rect bbox = cv::boundingRect(blob_contours[0]);
            cv::Point res_point((bbox.br().x + bbox.tl().x) / 2, (bbox.br().y + bbox.tl().y) / 2);
            geometry_msgs::Point result;
            result.x = res_point.x;
            result.y = res_point.y;
            result.z = 0;
            //result.hits = 5;
            res_pub_.publish(result);
            ROS_INFO("publish result %d,%d", res_point.x, res_point.y);
        }
    }

    void monoImageCb(const sensor_msgs::ImageConstPtr& image_msg) {
        cv_bridge::CvImagePtr image_ptr;
        try {
          image_ptr = cv_bridge::toCvCopy(*image_msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (...){
          ROS_ERROR("[draw_frames] Failed to convert image");
          return;
        }
        processImage(image_ptr->image);

//        static char prefix = 'a';
//        if(prefix < 'z') {
//            std::string path_to_save = std::string("/home/dd/img2dir_sources/ground/") + prefix++ + ".jpg";
//            cv::imwrite(path_to_save, image_ptr->image);
//        }
    }

    template<typename t_Param>
    void updateParam(std::string const& param_name, t_Param& param) {
        nh_.updateParam(param_name, param);
    }

private:
    static boost::filesystem::directory_iterator findNextImage(boost::filesystem::directory_iterator input_image_itr) {
        boost::filesystem::directory_iterator end_itr;
        while(input_image_itr != end_itr
            && (!boost::filesystem::is_regular_file(input_image_itr->path())
                || input_image_itr->path().string().size() < 4
                || input_image_itr->path().string().substr(input_image_itr->path().string().size() - 4, 4) != ".jpg"
               )
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
  Nh nh;
  TestSimpleImageProcessor node(nh);

  std::string image_file_name = "";
  node.updateParam("src_image_file", image_file_name);
  std::string image_src_folder_name = "";
  node.updateParam("src_image_src_folder", image_src_folder_name);

  if(!image_file_name.empty())
      node.processImage(image_file_name);
  else if(!image_src_folder_name.empty())
      node.processImagesFromFolder(image_src_folder_name);
  else
      ros::spin();
  return 0;
}
