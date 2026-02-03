#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

class HumanDetector
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Publisher human_area_pub_;
    cv::dnn::Net net_;  // YOLOv8 網路模型

public:
    HumanDetector()
    {
        // 訂閱影像
        image_sub_ = nh_.subscribe("/camera/image_raw", 1, &HumanDetector::imageCallback, this);
        
        // 設定發佈 topic
        human_area_pub_ = nh_.advertise<std_msgs::Float32>("/human_detection_area", 10);
        
        // 載入 YOLOv8 模型 (需下載 yolov8.onnx)
        net_ = cv::dnn::readNet("yolov8m.pt");

        // 使用 OpenCV DNN 加速
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

        ROS_INFO("HumanDetector Node Initialized");
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            // 轉換 ROS 影像到 OpenCV
            cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
            cv::Mat blob = cv::dnn::blobFromImage(image, 1.0 / 255.0, cv::Size(640, 640), cv::Scalar(), true, false);

            // 設定網路輸入
            net_.setInput(blob);

            // 取得偵測結果
            std::vector<cv::Mat> outputs;
            net_.forward(outputs, net_.getUnconnectedOutLayersNames());

            float detected_human_area = 0.0;

            // 解析 YOLOv8 結果
            for (size_t i = 0; i < outputs.size(); i++)
            {
                float* data = (float*)outputs[i].data;
                for (int j = 0; j < outputs[i].rows; j++, data += outputs[i].cols)
                {
                    float confidence = data[4];
                    if (confidence > 0.5)  // 只考慮高信心度的偵測
                    {
                        int class_id = static_cast<int>(data[5]);
                        if (class_id == 0)  // YOLOv8 class 0 代表 "person"
                        {
                            int x = static_cast<int>(data[0] * image.cols);
                            int y = static_cast<int>(data[1] * image.rows);
                            int width = static_cast<int>(data[2] * image.cols);
                            int height = static_cast<int>(data[3] * image.rows);

                            detected_human_area = width * height;

                            // 在影像上畫出框
                            cv::rectangle(image, cv::Rect(x, y, width, height), cv::Scalar(0, 255, 0), 2);
                        }
                    }
                }
            }

            // 發布人員面積
            std_msgs::Float32 msg;
            msg.data = detected_human_area;
            human_area_pub_.publish(msg);

            // 顯示影像（可選）
            cv::imshow("Human Detection", image);
            cv::waitKey(1);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "human_detector");
    HumanDetector hd;
    ros::spin();
    return 0;
}
