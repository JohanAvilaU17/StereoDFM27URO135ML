#include "utility.h"

class StereoDFM
{
public:
    ParamServer param_server;

    ros::NodeHandle nh;
    ros::Publisher pub_img_cam0;
    ros::Publisher pub_img_cam1;

    GstElement *pipeline0;
    GstElement *cam0;
    GstElement *capsfilter0;
    GstElement *appsink0;

    GstElement *pipeline1;
    GstElement *cam1;
    GstElement *capsfilter1;
    GstElement *appsink1;

    StereoDFM()
    {
        string pip_des0;
        pip_des0.append("tcambin name=camera serial=");
        pip_des0.append(this->param_server.serial_cam0);

        string pip_des1;
        pip_des1.append("tcambin name=camera serial=");
        pip_des1.append(this->param_server.serial_cam1);

        string pip_des_c;
        pip_des_c.append(" use-dutils=true ");
        pip_des_c.append("! capsfilter name=filter ");
        pip_des_c.append("! queue max-size-buffers=2 leaky=downstream ");
        pip_des_c.append("! appsink name=sink");

        pip_des0.append(pip_des_c);
        pip_des1.append(pip_des_c);

        const char *pipeline_description0 = pip_des0.c_str();
        const char *pipeline_description1 = pip_des1.c_str();

        GError *err0 = NULL;
        this->pipeline0 = gst_parse_launch(pipeline_description0, &err0);
        this->cam0 = gst_bin_get_by_name(GST_BIN(this->pipeline0), "camera");
        this->capsfilter0 = gst_bin_get_by_name(GST_BIN(this->pipeline0), "filter");
        this->appsink0 = gst_bin_get_by_name(GST_BIN(this->pipeline0), "sink");

        GError *err1 = NULL;
        this->pipeline1 = gst_parse_launch(pipeline_description1, &err1);
        this->cam1 = gst_bin_get_by_name(GST_BIN(this->pipeline1), "camera");
        this->capsfilter1 = gst_bin_get_by_name(GST_BIN(this->pipeline1), "filter");
        this->appsink1 = gst_bin_get_by_name(GST_BIN(this->pipeline1), "sink");

        if (this->pipeline0 == NULL)
        {
            ROS_INFO_STREAM("Could not create pipeline0. Cause:" << err0->message);
            exit(1);
        }
        if (this->cam0 == NULL)
        {
            ROS_INFO_STREAM("Could not retrieve cam0 from pipeline0.");
            exit(1);
        }
        if (this->capsfilter0 == NULL)
        {
            ROS_INFO_STREAM("Could not retrieve capsfilter0 from pipeline0.");
            exit(1);
        }
        if (this->appsink0 == NULL)
        {
            ROS_INFO_STREAM("Could not retrieve appsink0 from pipeline0.");
            exit(1);
        }

        if (this->pipeline1 == NULL)
        {
            ROS_INFO_STREAM("Could not create pipeline1. Cause:" << err1->message);
            exit(1);
        }
        if (this->cam1 == NULL)
        {
            ROS_INFO_STREAM("Could not retrieve cam1 from pipeline1.");
            exit(1);
        }
        if (this->capsfilter1 == NULL)
        {
            ROS_INFO_STREAM("Could not retrieve capsfilter1 from pipeline1.");
            exit(1);
        }
        if (this->appsink1 == NULL)
        {
            ROS_INFO_STREAM("Could not retrieve appsink1 from pipeline1.");
            exit(1);
        }
    }

    void setting_publisher()
    {
        this->pub_img_cam0 = this->nh.advertise<sensor_msgs::Image>(
            "stereo_dfm27uro135ml/cam0/image_raw", 1);
        this->pub_img_cam1 = this->nh.advertise<sensor_msgs::Image>(
            "stereo_dfm27uro135ml/cam1/image_raw", 1);
    }

    void setting_camera_before_PLAYING()
    {
        this->param_server.set_property(this->cam0, "Brightness", "Brightness", "int");
        this->param_server.set_property(this->cam0, "GPIn", "GPIn", "int");
        this->param_server.set_property(this->cam0, "GPOut", "GPOut", "int");
        this->param_server.set_property(this->cam0, "GainDB100", "Gain (dB/100)", "int");
        this->param_server.set_property(this->cam0, "OffsetAutoCenter", "Offset Auto Center", "boolean");
        this->param_server.set_property(this->cam0, "OffsetX", "Offset X", "int");
        this->param_server.set_property(this->cam0, "OffsetY", "Offset Y", "int");
        this->param_server.set_property(this->cam0, "OverrideScanningMode", "Override Scanning Mode", "int");
        this->param_server.set_property(this->cam0, "StrobeEnable", "Strobe Enable", "boolean");
        this->param_server.set_property(this->cam0, "StrobeExposure", "Strobe Exposure", "boolean");
        this->param_server.set_property(this->cam0, "StrobePolarity", "Strobe Polarity", "boolean");
        this->param_server.set_property(this->cam0, "TriggerDelayUs", "Trigger Delay (us)", "int");
        this->param_server.set_property(this->cam0, "TriggerGlobalResetRelease", "Trigger Global Reset Release", "boolean");
        this->param_server.set_property(this->cam0, "TriggerMode", "Trigger Mode", "boolean");
        this->param_server.set_property(this->cam0, "cameraWhitebalance", "camera-whitebalance", "boolean");
        this->param_server.set_property(this->cam0, "whitebalanceAuto", "whitebalance-auto", "boolean");
        this->param_server.set_property(this->cam0, "whitebalanceBlue", "whitebalance-blue", "int");
        this->param_server.set_property(this->cam0, "whitebalanceGreen", "whitebalance-green", "int");
        this->param_server.set_property(this->cam0, "whitebalanceModuleEnabled", "whitebalance-module-enabled", "boolean");
        this->param_server.set_property(this->cam0, "whitebalanceRed", "whitebalance-red", "int");

        this->param_server.set_property(this->cam1, "Brightness", "Brightness", "int");
        this->param_server.set_property(this->cam1, "GPIn", "GPIn", "int");
        this->param_server.set_property(this->cam1, "GPOut", "GPOut", "int");
        this->param_server.set_property(this->cam1, "GainDB100", "Gain (dB/100)", "int");
        this->param_server.set_property(this->cam1, "OffsetAutoCenter", "Offset Auto Center", "boolean");
        this->param_server.set_property(this->cam1, "OffsetX", "Offset X", "int");
        this->param_server.set_property(this->cam1, "OffsetY", "Offset Y", "int");
        this->param_server.set_property(this->cam1, "OverrideScanningMode", "Override Scanning Mode", "int");
        this->param_server.set_property(this->cam1, "StrobeEnable", "Strobe Enable", "boolean");
        this->param_server.set_property(this->cam1, "StrobeExposure", "Strobe Exposure", "boolean");
        this->param_server.set_property(this->cam1, "StrobePolarity", "Strobe Polarity", "boolean");
        this->param_server.set_property(this->cam1, "TriggerDelayUs", "Trigger Delay (us)", "int");
        this->param_server.set_property(this->cam1, "TriggerGlobalResetRelease", "Trigger Global Reset Release", "boolean");
        this->param_server.set_property(this->cam1, "TriggerMode", "Trigger Mode", "boolean");
        this->param_server.set_property(this->cam1, "cameraWhitebalance", "camera-whitebalance", "boolean");
        this->param_server.set_property(this->cam1, "whitebalanceAuto", "whitebalance-auto", "boolean");
        this->param_server.set_property(this->cam1, "whitebalanceBlue", "whitebalance-blue", "int");
        this->param_server.set_property(this->cam1, "whitebalanceGreen", "whitebalance-green", "int");
        this->param_server.set_property(this->cam1, "whitebalanceModuleEnabled", "whitebalance-module-enabled", "boolean");
        this->param_server.set_property(this->cam1, "whitebalanceRed", "whitebalance-red", "int");
    }

    void setting_camera_during_PLAYING()
    {
        this->param_server.set_property(this->cam0, "BrightnessReference", "Brightness Reference", "int");
        this->param_server.set_property(this->cam0, "ExposureAuto", "Exposure Auto", "boolean");
        this->param_server.set_property(this->cam0, "ExposureMin", "Exposure Min", "int");
        this->param_server.set_property(this->cam0, "ExposureMax", "Exposure Max", "int");
        this->param_server.set_property(this->cam0, "ExposureTimeUs", "Exposure Time (us)", "int");
        this->param_server.set_property(this->cam0, "GainAuto", "Gain Auto", "boolean");
        this->param_server.set_property(this->cam0, "GainMin", "Gain Min", "double");
        this->param_server.set_property(this->cam0, "GainMax", "Gain Max", "double");
        this->param_server.set_property(this->cam0, "Gain", "Gain", "int");
        this->param_server.set_property(this->cam0, "ExposureROILeft", "Exposure ROI Left", "int");
        this->param_server.set_property(this->cam0, "ExposureROIWidth", "Exposure ROI Width", "int");
        this->param_server.set_property(this->cam0, "ExposureROITop", "Exposure ROI Top", "int");
        this->param_server.set_property(this->cam0, "ExposureROIHeight", "Exposure ROI Height", "int");

        this->param_server.set_property(this->cam1, "BrightnessReference", "Brightness Reference", "int");
        this->param_server.set_property(this->cam1, "ExposureAuto", "Exposure Auto", "boolean");
        this->param_server.set_property(this->cam1, "ExposureMin", "Exposure Min", "int");
        this->param_server.set_property(this->cam1, "ExposureMax", "Exposure Max", "int");
        this->param_server.set_property(this->cam1, "ExposureTimeUs", "Exposure Time (us)", "int");
        this->param_server.set_property(this->cam1, "GainAuto", "Gain Auto", "boolean");
        this->param_server.set_property(this->cam1, "GainMin", "Gain Min", "double");
        this->param_server.set_property(this->cam1, "GainMax", "Gain Max", "double");
        this->param_server.set_property(this->cam1, "Gain", "Gain", "int");
        this->param_server.set_property(this->cam1, "ExposureROILeft", "Exposure ROI Left", "int");
        this->param_server.set_property(this->cam1, "ExposureROIWidth", "Exposure ROI Width", "int");
        this->param_server.set_property(this->cam1, "ExposureROITop", "Exposure ROI Top", "int");
        this->param_server.set_property(this->cam1, "ExposureROIHeight", "Exposure ROI Height", "int");

        gst_object_unref(this->cam0);
        gst_object_unref(this->cam1);
    }

    void setting_capsfilter()
    {
        this->param_server.set_format(this->capsfilter0);
        this->param_server.set_format(this->capsfilter1);

        gst_object_unref(this->capsfilter0);
        gst_object_unref(this->capsfilter1);
    }

    void setting_appsink()
    {
        g_object_set(G_OBJECT(this->appsink0), "sync", FALSE, NULL);
        g_object_set(G_OBJECT(this->appsink0), "emit-signals", TRUE, NULL);
        g_object_set(G_OBJECT(this->appsink0), "drop", TRUE, NULL);
        g_object_set(G_OBJECT(this->appsink0), "max-buffers", 4, NULL);

        g_object_set(G_OBJECT(this->appsink1), "sync", FALSE, NULL);
        g_object_set(G_OBJECT(this->appsink1), "emit-signals", TRUE, NULL);
        g_object_set(G_OBJECT(this->appsink1), "drop", TRUE, NULL);
        g_object_set(G_OBJECT(this->appsink1), "max-buffers", 4, NULL);
    }
};

gboolean block_until_playing(GstElement *pipeline)
{
    while (TRUE)
    {
        GstState state;
        GstState pending;

        // wait 0.1 seconds for something to happen
        GstStateChangeReturn ret = gst_element_get_state(pipeline, &state, &pending, 100000000);

        if (ret == GST_STATE_CHANGE_SUCCESS)
        {
            return TRUE;
        }
        else if (ret == GST_STATE_CHANGE_FAILURE)
        {
            printf("Failed to change state %s %s %s\n",
                   gst_element_state_change_return_get_name(ret),
                   gst_element_state_get_name(state),
                   gst_element_state_get_name(pending));

            return FALSE;
        }
    }
}

static GstFlowReturn publish_img0(GstElement *appsink, StereoDFM *stereo_DFM)
{
    GstSample *sample = NULL;
    g_signal_emit_by_name(appsink, "pull-sample", &sample, NULL);
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstMapInfo info;
    gst_buffer_map(buffer, &info, GST_MAP_READ);
    sensor_msgs::Image img;
    img.header.frame_id = "cam0";
    img.height = stereo_DFM->param_server.height;
    img.width = stereo_DFM->param_server.width;
    img.encoding = stereo_DFM->param_server.ros_encoding_type;
    img.data.assign(info.data, info.data + info.size);
    img.step = stereo_DFM->param_server.width;
    img.header.stamp = ros::Time::now();
    stereo_DFM->pub_img_cam0.publish(img);
    gst_buffer_unmap(buffer, &info);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
};

static GstFlowReturn publish_img1(GstElement *appsink, StereoDFM *stereo_DFM)
{
    GstSample *sample = NULL;
    g_signal_emit_by_name(appsink, "pull-sample", &sample, NULL);
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstMapInfo info;
    gst_buffer_map(buffer, &info, GST_MAP_READ);
    sensor_msgs::Image img;
    img.header.frame_id = "cam1";
    img.height = stereo_DFM->param_server.height;
    img.width = stereo_DFM->param_server.width;
    img.encoding = stereo_DFM->param_server.ros_encoding_type;
    img.data.assign(info.data, info.data + info.size);
    img.step = stereo_DFM->param_server.width;
    img.header.stamp = ros::Time::now();
    stereo_DFM->pub_img_cam1.publish(img);
    gst_buffer_unmap(buffer, &info);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "stereo_dfm27uro135ml");

    gst_debug_set_default_threshold(GST_LEVEL_WARNING);
    gst_init(&argc, &argv);

    StereoDFM stereo_DFM;
    stereo_DFM.setting_publisher();
    stereo_DFM.setting_camera_before_PLAYING();
    stereo_DFM.setting_capsfilter();
    stereo_DFM.setting_appsink();

    g_signal_connect(stereo_DFM.appsink0, "new-sample", G_CALLBACK(publish_img0), &stereo_DFM);
    g_signal_connect(stereo_DFM.appsink1, "new-sample", G_CALLBACK(publish_img1), &stereo_DFM);

    gst_object_unref(stereo_DFM.appsink0);
    gst_object_unref(stereo_DFM.appsink1);

    gst_element_set_state(stereo_DFM.pipeline0, GST_STATE_PLAYING);
    gst_element_set_state(stereo_DFM.pipeline1, GST_STATE_PLAYING);

    if (!block_until_playing(stereo_DFM.pipeline0))
    {
        ROS_INFO_STREAM("Unable to start pipeline1.");
    }

    if (!block_until_playing(stereo_DFM.pipeline1))
    {
        ROS_INFO_STREAM("Unable to start pipeline1.");
    }

    stereo_DFM.setting_camera_during_PLAYING();

    ROS_INFO_STREAM("\033[1;32m-> StereoImages.\033[0m");
    ROS_INFO_STREAM("Press Ctrl-C to stop.");

    while (ros::ok())
    {
    }

    gst_element_set_state(stereo_DFM.pipeline0, GST_STATE_NULL);
    gst_element_set_state(stereo_DFM.pipeline1, GST_STATE_NULL);

    gst_object_unref(stereo_DFM.pipeline0);
    gst_object_unref(stereo_DFM.pipeline1);
    return 0;
}
