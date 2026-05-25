#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "MvCameraControl.h"
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <cstring>

class HikCameraNode : public rclcpp::Node {
public:
    HikCameraNode() : Node("hik_camera_node"), handle_(nullptr) {
        // 声明参数
        this->declare_parameter("camera_selector", "index");
        this->declare_parameter("camera_index", -1);
        this->declare_parameter("camera_ip", "0.0.0.0");
        this->declare_parameter("exposure_time_us", 8000.0f);
        this->declare_parameter("gain", 20.0f);
        this->declare_parameter("frame_id", "hik_camera");
        this->declare_parameter("publish_ros", true);        // 是否发布 ROS 话题
        this->declare_parameter("use_shared_memory", true);  // 是否使用共享内存

        // 无论是否发布 ROS，都创建 publisher，方便动态切换
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);

        if (!init_camera()) {
            RCLCPP_ERROR(this->get_logger(), "相机初始化失败，节点将退出。");
            rclcpp::shutdown();
            return;
        }

        // 初始化共享内存（如果需要）
        if (this->get_parameter("use_shared_memory").as_bool()) {
            setup_shared_memory();
        }

        start_grabbing();
    }

    ~HikCameraNode() {
        stop_grabbing();
        close_camera();
        release_shared_memory();
    }

private:
    bool init_camera() {
        MV_CC_Initialize();

        MV_CC_DEVICE_INFO_LIST device_list;
        memset(&device_list, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        int ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
        if (MV_OK != ret || device_list.nDeviceNum == 0) {
            RCLCPP_ERROR(this->get_logger(), "未找到海康相机设备，错误码: 0x%x", ret);
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "发现 %d 个设备：", device_list.nDeviceNum);
        for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
            print_device_info(device_list.pDeviceInfo[i], i);
        }

        MV_CC_DEVICE_INFO* p_device = nullptr;
        std::string selector = this->get_parameter("camera_selector").as_string();

        if (selector == "index") {
            int idx = this->get_parameter("camera_index").as_int();
            if (idx < 0 || idx >= static_cast<int>(device_list.nDeviceNum)) {
                RCLCPP_ERROR(this->get_logger(),
                    "camera_index (%d) 无效，有效范围: 0 ~ %d", idx, device_list.nDeviceNum - 1);
                return false;
            }
            p_device = device_list.pDeviceInfo[idx];
            RCLCPP_INFO(this->get_logger(), "通过索引 %d 选择设备。", idx);
        } else if (selector == "ip") {
            std::string target_ip = this->get_parameter("camera_ip").as_string();
            p_device = find_device_by_ip(&device_list, target_ip);
            if (!p_device) {
                RCLCPP_ERROR(this->get_logger(), "未找到 IP 为 %s 的设备。", target_ip.c_str());
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "通过 IP %s 选择设备。", target_ip.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "未知的选择器类型: %s", selector.c_str());
            return false;
        }

        ret = MV_CC_CreateHandle(&handle_, p_device);
        if (MV_OK != ret) {
            RCLCPP_ERROR(this->get_logger(), "创建句柄失败，错误码: 0x%x", ret);
            return false;
        }

        ret = MV_CC_OpenDevice(handle_);
        if (MV_OK != ret) {
            RCLCPP_ERROR(this->get_logger(), "打开设备失败，错误码: 0x%x", ret);
            MV_CC_DestroyHandle(handle_);
            handle_ = nullptr;
            return false;
        }

        // 设置连续采集模式
        ret = MV_CC_SetEnumValue(handle_, "TriggerMode", MV_TRIGGER_MODE_OFF);
        if (MV_OK != ret) {
            RCLCPP_WARN(this->get_logger(), "无法设置 TriggerMode=OFF，错误码: 0x%x", ret);
        }

        // 设置分辨率（3072x2048）
        ret = MV_CC_SetIntValue(handle_, "Width", 3072);
        if (MV_OK != ret) RCLCPP_WARN(this->get_logger(), "设置 Width 失败");
        ret = MV_CC_SetIntValue(handle_, "Height", 2048);
        if (MV_OK != ret) RCLCPP_WARN(this->get_logger(), "设置 Height 失败");

        // 设置像素格式为 BayerRG8 以获得 60fps
        ret = MV_CC_SetEnumValue(handle_, "PixelFormat", 0x01080009);
        if (MV_OK != ret) {
            RCLCPP_WARN(this->get_logger(), "设置 PixelFormat 为 BayerRG8 失败，错误码: 0x%x", ret);
        } else {
            RCLCPP_INFO(this->get_logger(), "已设置 BayerRG8 像素格式（60fps）");
        }

        // 关闭帧率控制（如果支持）
        ret = MV_CC_SetBoolValue(handle_, "FrameRateControlEnable", false);
        if (MV_OK != ret) {
            RCLCPP_WARN(this->get_logger(), "关闭帧率控制失败，错误码: 0x%x", ret);
        }

        // 设置曝光和增益
        float exposure = static_cast<float>(this->get_parameter("exposure_time_us").as_double());
        ret = MV_CC_SetFloatValue(handle_, "ExposureTime", exposure);
        if (MV_OK != ret) RCLCPP_WARN(this->get_logger(), "设置曝光时间失败");
        else RCLCPP_INFO(this->get_logger(), "曝光时间: %.1f us", exposure);

        float gain = static_cast<float>(this->get_parameter("gain").as_double());
        ret = MV_CC_SetFloatValue(handle_, "Gain", gain);
        if (MV_OK != ret) RCLCPP_WARN(this->get_logger(), "设置增益失败");
        else RCLCPP_INFO(this->get_logger(), "增益: %.1f dB", gain);

        // 获取实际分辨率
        MVCC_INTVALUE width_val, height_val;
        MV_CC_GetIntValue(handle_, "Width", &width_val);
        MV_CC_GetIntValue(handle_, "Height", &height_val);
        img_width_ = width_val.nCurValue;
        img_height_ = height_val.nCurValue;
        RCLCPP_INFO(this->get_logger(), "相机分辨率: %d x %d", img_width_, img_height_);

        // 获取 PayloadSize
        MVCC_INTVALUE payload;
        ret = MV_CC_GetIntValue(handle_, "PayloadSize", &payload);
        if (MV_OK == ret) {
            n_payload_size_ = payload.nCurValue;
            RCLCPP_INFO(this->get_logger(), "PayloadSize: %u", n_payload_size_);
        }

        return true;
    }

    void start_grabbing() {
        int ret = MV_CC_StartGrabbing(handle_);
        if (ret != MV_OK) {
            RCLCPP_ERROR(this->get_logger(), "开始取流失败，错误码: 0x%x", ret);
            return;
        }
        grab_thread_ = std::thread(&HikCameraNode::grabbing_loop, this);
    }

    void stop_grabbing() {
        if (handle_) {
            MV_CC_StopGrabbing(handle_);
        }
        if (grab_thread_.joinable()) {
            grab_thread_.join();
        }
    }

    void close_camera() {
        if (handle_) {
            MV_CC_CloseDevice(handle_);
            MV_CC_DestroyHandle(handle_);
            handle_ = nullptr;
        }
        MV_CC_Finalize();
    }

    void setup_shared_memory() {
        const char* shm_name = "/hik_camera";  // 最终路径 /dev/shm/hik_camera
        shm_unlink(shm_name);  // 清理可能残留的旧共享内存

        int fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
        if (fd < 0) {
            RCLCPP_ERROR(this->get_logger(), "shm_open 失败: %s", strerror(errno));
            return;
        }

        // 计算总大小：头部 + 图像数据
        header_size_ = sizeof(int32_t) * 2       // width, height
                     + sizeof(uint32_t)           // frame_index
                     + sizeof(int64_t);           // timestamp_ns
        const uint32_t encoding_len = 4;          // "bgr8" 的字符串长度
        const uint32_t encoding_field_size = sizeof(uint32_t) + encoding_len; // 长度 + 字符串
        data_size_ = img_width_ * img_height_ * 3;
        total_size_ = header_size_ + encoding_field_size + data_size_;

        if (ftruncate(fd, total_size_) < 0) {
            RCLCPP_ERROR(this->get_logger(), "ftruncate 失败: %s", strerror(errno));
            close(fd);
            return;
        }

        shm_ptr_ = mmap(nullptr, total_size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        close(fd);  // 映射后可以关闭 fd

        if (shm_ptr_ == MAP_FAILED) {
            RCLCPP_ERROR(this->get_logger(), "mmap 失败: %s", strerror(errno));
            shm_ptr_ = nullptr;
            return;
        }

        // 初始化头部
        auto* header = static_cast<uint8_t*>(shm_ptr_);
        // width
        *reinterpret_cast<int32_t*>(header) = img_width_;
        // height
        *reinterpret_cast<int32_t*>(header + 4) = img_height_;
        // frame_index
        *reinterpret_cast<uint32_t*>(header + 8) = 0;
        // timestamp_ns
        *reinterpret_cast<int64_t*>(header + 12) = 0;
        // encoding 长度
        *reinterpret_cast<uint32_t*>(header + header_size_) = encoding_len;
        // encoding 字符串
        memcpy(header + header_size_ + sizeof(uint32_t), "bgr8", encoding_len);

        shm_ready_ = true;
        RCLCPP_INFO(this->get_logger(),
            "共享内存已创建: %s, 分辨率 %dx%d, 总大小: %zu bytes",
            shm_name, img_width_, img_height_, total_size_);
    }

    void write_shared_memory(const cv::Mat& img, int64_t timestamp_ns) {
        if (!shm_ready_ || shm_ptr_ == nullptr) return;

        auto* header = static_cast<uint8_t*>(shm_ptr_);
        // 写入时间戳
        *reinterpret_cast<int64_t*>(header + 12) = timestamp_ns;
        // 写入图像数据
        uint8_t* data_start = header + header_size_ + sizeof(uint32_t) + 4;
        memcpy(data_start, img.data, data_size_);
        // 递增帧序号（最后操作，作为新帧标志）
        uint32_t new_idx = (*reinterpret_cast<uint32_t*>(header + 8)) + 1;
        *reinterpret_cast<uint32_t*>(header + 8) = new_idx;
    }

    void release_shared_memory() {
        if (shm_ptr_ && shm_ptr_ != MAP_FAILED) {
            munmap(shm_ptr_, total_size_);
            shm_ptr_ = nullptr;
        }
        shm_unlink("/hik_camera");
        shm_ready_ = false;
    }

    void grabbing_loop() {
        MV_FRAME_OUT st_frame;
        memset(&st_frame, 0, sizeof(MV_FRAME_OUT));

        // 预分配转换缓冲区（分辨率固定）
        unsigned int n_rgb_size = img_width_ * img_height_ * 3;
        rgb_buf_.resize(n_rgb_size);

        while (rclcpp::ok()) {
            unsigned int ret = MV_CC_GetImageBuffer(handle_, &st_frame, 1000);
            if (MV_OK != ret) {
                if (ret != MV_E_GC_TIMEOUT) {
                    RCLCPP_WARN(this->get_logger(), "获取图像失败，错误码: 0x%x", ret);
                }
                continue;
            }

            cv::Mat img;
            unsigned int pixel_type = st_frame.stFrameInfo.enPixelType;

            if (pixel_type == PixelType_Gvsp_BayerRG8) {
                MV_CC_PIXEL_CONVERT_PARAM_EX st_convert_param;
                memset(&st_convert_param, 0, sizeof(st_convert_param));
                st_convert_param.nWidth  = st_frame.stFrameInfo.nWidth;
                st_convert_param.nHeight = st_frame.stFrameInfo.nHeight;
                st_convert_param.pSrcData     = st_frame.pBufAddr;
                st_convert_param.nSrcDataLen  = st_frame.stFrameInfo.nFrameLen;
                st_convert_param.enSrcPixelType = PixelType_Gvsp_BayerRG8;
                st_convert_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
                st_convert_param.pDstBuffer     = rgb_buf_.data();
                st_convert_param.nDstBufferSize = n_rgb_size;

                int convert_ret = MV_CC_ConvertPixelTypeEx(handle_, &st_convert_param);
                if (MV_OK != convert_ret) {
                    RCLCPP_WARN(this->get_logger(), "像素格式转换失败，错误码: 0x%x", convert_ret);
                    MV_CC_FreeImageBuffer(handle_, &st_frame);
                    continue;
                }
                // 直接使用 rgb_buf_ 中的数据构造 Mat
                img = cv::Mat(img_height_, img_width_, CV_8UC3, rgb_buf_.data());
            } else if (pixel_type == PixelType_Gvsp_Mono8) {
                img = cv::Mat(st_frame.stFrameInfo.nHeight, st_frame.stFrameInfo.nWidth,
                              CV_8UC1, st_frame.pBufAddr);
            } else if (pixel_type == PixelType_Gvsp_RGB8_Packed) {
                img = cv::Mat(st_frame.stFrameInfo.nHeight, st_frame.stFrameInfo.nWidth,
                              CV_8UC3, st_frame.pBufAddr);
                cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
            } else {
                RCLCPP_WARN(this->get_logger(), "不支持的像素格式 0x%x, 跳过", pixel_type);
                MV_CC_FreeImageBuffer(handle_, &st_frame);
                continue;
            }

            // 获取当前时间戳（纳秒）
            int64_t timestamp_ns = this->now().nanoseconds();

            // 写入共享内存（如果启用）
            if (shm_ready_) {
                write_shared_memory(img, timestamp_ns);
            }

            // 发布 ROS 话题（如果启用）
            if (this->get_parameter("publish_ros").as_bool()) {
                auto msg = cv_bridge::CvImage(
                    std_msgs::msg::Header(),
                    (img.channels() == 1) ? sensor_msgs::image_encodings::MONO8
                                          : sensor_msgs::image_encodings::BGR8,
                    img).toImageMsg();
                msg->header.stamp = rclcpp::Time(timestamp_ns);
                msg->header.frame_id = this->get_parameter("frame_id").as_string();
                publisher_->publish(*msg);
            }

            MV_CC_FreeImageBuffer(handle_, &st_frame);
        }
    }

    void print_device_info(const MV_CC_DEVICE_INFO* info, unsigned int idx) {
        std::string name, serial;
        if (info->nTLayerType == MV_GIGE_DEVICE) {
            name.assign(reinterpret_cast<const char*>(info->SpecialInfo.stGigEInfo.chModelName));
            serial.assign(reinterpret_cast<const char*>(info->SpecialInfo.stGigEInfo.chSerialNumber));
            unsigned int ip = info->SpecialInfo.stGigEInfo.nCurrentIp;
            RCLCPP_INFO(this->get_logger(),
                "  [%u] Model: %s, Serial: %s, IP: %d.%d.%d.%d", idx,
                name.c_str(), serial.c_str(),
                (ip >> 24) & 0xFF, (ip >> 16) & 0xFF, (ip >> 8) & 0xFF, ip & 0xFF);
        } else if (info->nTLayerType == MV_USB_DEVICE) {
            name.assign(reinterpret_cast<const char*>(info->SpecialInfo.stUsb3VInfo.chModelName));
            serial.assign(reinterpret_cast<const char*>(info->SpecialInfo.stUsb3VInfo.chSerialNumber));
            RCLCPP_INFO(this->get_logger(),
                "  [%u] Model: %s, Serial: %s (USB)", idx, name.c_str(), serial.c_str());
        }
    }

    MV_CC_DEVICE_INFO* find_device_by_ip(MV_CC_DEVICE_INFO_LIST* list, const std::string& ip_str) {
        for (unsigned int i = 0; i < list->nDeviceNum; ++i) {
            MV_CC_DEVICE_INFO* info = list->pDeviceInfo[i];
            if (info->nTLayerType == MV_GIGE_DEVICE) {
                unsigned int ip = info->SpecialInfo.stGigEInfo.nCurrentIp;
                char ip_buf[32];
                snprintf(ip_buf, sizeof(ip_buf), "%d.%d.%d.%d",
                         (ip >> 24) & 0xFF, (ip >> 16) & 0xFF, (ip >> 8) & 0xFF, ip & 0xFF);
                if (ip_str == ip_buf) return info;
            }
        }
        return nullptr;
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    void* handle_;
    std::thread grab_thread_;
    int img_width_ = 0;
    int img_height_ = 0;
    unsigned int n_payload_size_ = 0;
    std::vector<unsigned char> rgb_buf_;

    // 共享内存相关
    void* shm_ptr_ = nullptr;
    bool shm_ready_ = false;
    size_t total_size_ = 0;
    size_t header_size_ = 0;   // 固定头部大小 (width+height+frame_index+timestamp)
    size_t data_size_ = 0;     // 图像数据大小
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HikCameraNode>();
    rclcpp::spin(node);
    return 0;
}