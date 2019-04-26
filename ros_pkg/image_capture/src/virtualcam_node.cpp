


/**

    V4L2 - Need to install apt-get install libv4l-dev v4l2loopback-utils v4l2loopback-dkms

    Create virtual device (can create multiple devices):
    sudo modprobe v4l2loopback video_nr=2,3 card_label="VirtualCamHead","VirtualCamNav"


**/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <string>
#include <fstream>
#include <fcntl.h>
#include <stdio.h>
#include <errno.h>
#include <linux/videodev2.h>
#include <libv4l2.h>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <assert.h>

# define FRAME_FORMAT V4L2_PIX_FMT_YUYV

namespace enc = sensor_msgs::image_encodings;

#define ROUND_UP_2(num)  (((num)+1)&~1)
#define ROUND_UP_4(num)  (((num)+3)&~3)
#define ROUND_UP_8(num)  (((num)+7)&~7)
#define ROUND_UP_16(num) (((num)+15)&~15)
#define ROUND_UP_32(num) (((num)+31)&~31)
#define ROUND_UP_64(num) (((num)+63)&~63)

int format_properties(const unsigned int format,
                      const unsigned int width,
                      const unsigned int height,
                      size_t*linewidth,
                      size_t*framewidth)
{
    size_t lw, fw;
    switch(format)
    {
    case V4L2_PIX_FMT_YUV420: case V4L2_PIX_FMT_YVU420:
        lw = width; /* ??? */
        fw = ROUND_UP_4 (width) * ROUND_UP_2 (height);
        fw += 2 * ((ROUND_UP_8 (width) / 2) * (ROUND_UP_2 (height) / 2));
        break;

    case V4L2_PIX_FMT_UYVY: case V4L2_PIX_FMT_Y41P: case V4L2_PIX_FMT_YUYV: case V4L2_PIX_FMT_YVYU:
        lw = (ROUND_UP_2 (width) * 2);
        fw = lw * height;
        break;
    default:
        return 0;
    }

    if(linewidth)*linewidth=lw;
    if(framewidth)*framewidth=fw;

    return 1;
}

void print_format(struct v4l2_format*vid_format)
{
    ROS_INFO("vid_format->type                =%u\n",	vid_format->type );
    ROS_INFO("vid_format->fmt.pix.width       =%u\n",	vid_format->fmt.pix.width );
    ROS_INFO("vid_format->fmt.pix.height      =%u\n",	vid_format->fmt.pix.height );
    ROS_INFO("vid_format->fmt.pix.pixelformat =%u\n",	vid_format->fmt.pix.pixelformat);
    ROS_INFO("vid_format->fmt.pix.sizeimage   =%u\n",	vid_format->fmt.pix.sizeimage );
    ROS_INFO("vid_format->fmt.pix.field       =%u\n",	vid_format->fmt.pix.field );
    ROS_INFO("vid_format->fmt.pix.bytesperline=%u\n",	vid_format->fmt.pix.bytesperline );
    ROS_INFO("vid_format->fmt.pix.colorspace  =%u\n",	vid_format->fmt.pix.colorspace );
}

#define LOOPBACK_DEVICE "v4l2loopback"
#define x_ioctl v4l2_ioctl
#define x_open v4l2_open
#define x_close v4l2_close
#define x_write v4l2_write



/**
 * @brief The ImageConverter class
 * https://github.com/czw90130/virtual_camera/blob/master/src/streamputer.cpp
 */
class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    int fdwr;
    size_t imageSize;
    size_t lineSize;
    __u8 *buffer;
    cv::Mat yuv_img;
    cv::Size yuv_img_sz;

public:
    ImageConverter(int fd,
        size_t imgsz,
        size_t linesz,
        const std::string &topic,
        int image_width,
        int image_height)
        : it_(nh_)
    {
        fdwr = fd;
        imageSize = imgsz;
        lineSize = linesz;

        yuv_img_sz = cv::Size(image_width, image_height);
        yuv_img = cv::Mat(yuv_img_sz, CV_8U);

        //YUV buffer
        buffer = (__u8*)malloc(sizeof(__u8)*imageSize);
        memset(buffer, 0, imageSize);
        write(fdwr, buffer, imageSize);

        image_sub_=it_.subscribe(topic, 1, &ImageConverter::imageCb, this);
    }

    ~ImageConverter()
    {
        free(buffer);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImageConstPtr cv_ptr;

        try
        {
            cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            assert(0);
        }

        if (cv_ptr->image.rows != yuv_img_sz.height || cv_ptr->image.cols != yuv_img_sz.width)
        {
            try
            {
                cv::resize(cv_ptr->image, yuv_img, yuv_img_sz);
            }
            catch(cv::Exception& e)
            {
                ROS_ERROR("cvResize error");
                assert(0);
            }
            writeImage(yuv_img);
        }    
        else
        {
            writeImage(cv_ptr->image);
        }
    }

    void writeImage(const cv::Mat& image)
    {
        size_t nx;
        size_t ny;
        __u8 r0,g0,b0,r1,g1,b1;
        __u8 y0,y1,u,v;

        int ncol;
        const uchar* pcol;
        __u8* bufferCell = buffer;

        ncol = image.cols*image.channels();
        for(ny=0; ny<image.rows; ny++)
        {
            pcol = image.ptr<const uchar>(ny);
            for(nx=0; nx < ncol; nx=nx+6)
            {
                r0 = (__u8)*(pcol++);
                g0 = (__u8)*(pcol++);
                b0 = (__u8)*(pcol++);
                r1 = (__u8)*(pcol++);
                g1 = (__u8)*(pcol++);
                b1 = (__u8)*(pcol++);

                y0 = (__u8)((306*(int)r0 + 601*(int)g0 + 117*(int)b0) >> 10);
                y1 = (__u8)((306*(int)r1 + 601*(int)g1 + 117*(int)b1) >> 10);
                u  = (__u8)(((-172*(int)r0 - 339*(int)g0 + 512*(int)b0) >> 10) + 128);
                v  = (__u8)(((512*(int)r0 - 428*(int)g0 - 83*(int)b0) >> 10) + 128);

                *(bufferCell++) = y0;
                *(bufferCell++) = v;
                *(bufferCell++) = y1;
                *(bufferCell++) = u;
            }            
        }
        if(bufferCell - buffer > imageSize)
        {
            ROS_ERROR("The size of image exceed!");
            return;
        }
        write(fdwr, buffer, imageSize);
    }

};

class VirtualCamera
{
public:

    VirtualCamera(ros::NodeHandle &nh,
        const std::string &topic,
        const std::string &virtual_dev,
        int image_width,
        int image_height)
        : m_fd(-1), m_converter(NULL)
    {

        //Open loopback device, will always be of fixed image size.
        m_fd = open_loopback_device2(virtual_dev, image_width, image_height);

        //Will convert and write image to the loopback device.
        //Will subscribe to the topic
        size_t linewidth = 0;
        size_t framewidth = 0;

        //Will get linewidth and framewidth according to specified output format
        format_properties(FRAME_FORMAT, image_width, image_height, &linewidth, &framewidth);

        //Create converter
        m_converter = new ImageConverter(m_fd,framewidth, linewidth, topic, image_width, image_height);
    }

    ~VirtualCamera()
    {
        if (m_converter)
        {
            delete m_converter;
            m_converter = NULL;
        }

    }


protected:


    int open_loopback_device2(const std::string &device_name, unsigned int width, unsigned int height)
    {

        //Config camera
        struct v4l2_capability vid_caps;
        struct v4l2_format vid_format;

        size_t framesize;
        size_t linewidth;

        int fdwr = open(device_name.c_str(), O_RDWR);
        assert(fdwr >= 0);

        int ret_code = ioctl(fdwr, VIDIOC_QUERYCAP, &vid_caps);
        assert(ret_code != -1);

        memset(&vid_format, 0, sizeof(vid_format));

        ret_code = ioctl(fdwr, VIDIOC_G_FMT, &vid_format);
        //print_format(&vid_format);

        //format
        vid_format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        vid_format.fmt.pix.width = width;
        vid_format.fmt.pix.height = height;
        vid_format.fmt.pix.pixelformat = FRAME_FORMAT;
        vid_format.fmt.pix.field = V4L2_FIELD_NONE;
        vid_format.fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;


        if(!format_properties(vid_format.fmt.pix.pixelformat,
                              vid_format.fmt.pix.width, vid_format.fmt.pix.height,
                              &linewidth,
                              &framesize))
        {
            ROS_ERROR("unable to guess correct settings for format '%i'\n", FRAME_FORMAT);
            close(fdwr);
            return -1;
        }

        vid_format.fmt.pix.sizeimage = framesize;
        vid_format.fmt.pix.bytesperline = linewidth;

        //print_format(&vid_format);
        ret_code = ioctl(fdwr, VIDIOC_S_FMT, &vid_format);
        assert(ret_code != -1);


        print_format(&vid_format);
        return fdwr;
    }

    //File descriptor for loopback device
    int m_fd;

    //Image converter and scaler from BRG8 to UYVY 4:2:2
    ImageConverter *m_converter;
};

int main(int argc, char* argv[])
{
    //ROS INIT
    ros::init(argc, argv, "webrtc_gui_node");
    ros::NodeHandle nh(""), nh_param("~");

    std::string image_topic;
    std::string virtual_dev;
    int image_width;
    int image_height;
    nh_param.param<std::string>("image_topic", image_topic, "/test_camera_node/image_raw");    
    nh_param.param<std::string>("virtual_dev", virtual_dev, "/dev/video3");
    nh_param.param("image_width", image_width, 640);
    nh_param.param("image_height", image_height, 480);


    //Virtual Camera
    VirtualCamera cam(nh, image_topic, virtual_dev, image_width, image_height);

    /**
    * ros::spin() will enter a loop, pumping callbacks.  With this version, all
    * callbacks will be called from within this thread (the main one).  ros::spin()
    * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
    */
    ros::spin();


    return 0;
}
