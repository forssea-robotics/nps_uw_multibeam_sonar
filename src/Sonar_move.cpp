#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>

#include <gazebo_msgs/msg/model_states.hpp>
#include <fstream>
#include <iomanip>
#include <sstream>

using std::placeholders::_1;

int getIndex(std::vector<std::string> v, std::string value)
{
    for(int i = 0; i < v.size(); i++)
    {
        if(v[i].compare(value) == 0)
            return i;
    }
    return -1;
}


class PoseImageSubscriber: public rclcpp::Node
{
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_sub_;
    image_transport::Subscriber image_sub_;
    std::ofstream sonar_pose;
    int counter_int;
    public:
    PoseImageSubscriber(): Node("pose_image_subscriber")
    {
        model_sub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>("/model_states", 10, std::bind(&PoseImageSubscriber::model_states_callback, this, _1));
        rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
        image_sub_ = image_transport::create_subscription(this, "/sonar_image", std::bind(&PoseImageSubscriber::imageCallback, this, std::placeholders::_1), "raw", custom_qos);
        counter_int = 0;
    }

    void model_states_callback(const std::shared_ptr<gazebo_msgs::msg::ModelStates> model_states)
    {
        int oculus_m1200d_index = getIndex(model_states->name, "oculus_m1200d");
        std::ostringstream os;
        os << model_states->pose[oculus_m1200d_index].position.x <<" "<<
              model_states->pose[oculus_m1200d_index].position.y <<" "<<
              model_states->pose[oculus_m1200d_index].position.z <<" "<<
              model_states->pose[oculus_m1200d_index].orientation.x <<" "<<
              model_states->pose[oculus_m1200d_index].orientation.y <<" "<<
              model_states->pose[oculus_m1200d_index].orientation.z <<" "<<
              model_states->pose[oculus_m1200d_index].orientation.w <<" "<< std::endl;
        std::string str = os.str();
        sonar_pose.open("/media/jaouadros/My Passport/PhD/Sonar/Sonar-simulator-blender/data/temp_files_blender_simulator/example.txt", std::ios::app);
        sonar_pose << str;
        sonar_pose.close();
    }
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        std::ostringstream counter_string;
        counter_string << "/media/jaouadros/My Passport/PhD/Sonar/Sonar-simulator-blender/data/temp_files_blender_simulator" << std::setw(10) << std::setfill('0') << counter_int++;
        cv::imwrite(counter_string.str(), cv_ptr->image);

    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseImageSubscriber>());
  rclcpp::shutdown();
  return 0;
}
