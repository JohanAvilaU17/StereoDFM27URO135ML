#include "utility.h"

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

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "stereo_dfm27uro135ml");

    gst_debug_set_default_threshold(GST_LEVEL_WARNING);
    gst_init(&argc, &argv);

    ParamServer param_server;

    string pip_des;
    pip_des.append("tcambin name=camera serial=");
    pip_des.append(param_server.serial_cam1);
    pip_des.append(" use-dutils=true ");
    pip_des.append("! capsfilter name=filter ");
    pip_des.append("! queue max-size-buffers=2 leaky=downstream ");
    pip_des.append("! videoconvert ");
    pip_des.append("! xvimagesink double-buffer=true sync=false draw-borders=false");

    GError *err = NULL;
    const char *pipeline_description = pip_des.c_str();
    GstElement *pipeline = gst_parse_launch(pipeline_description, &err);
    GstElement *cam1 = gst_bin_get_by_name(GST_BIN(pipeline), "camera");
    GstElement *capsfilter = gst_bin_get_by_name(GST_BIN(pipeline), "filter");

    if (pipeline == NULL)
    {
        printf("Could not create pipeline. Cause: %s\n", err->message);
        return 1;
    }

    if (cam1 == NULL)
    {
        printf("Could not retrieve cam1 from pipeline.");
        return 1;
    }

    if (capsfilter == NULL)
    {
        printf("Could not retrieve capsfilter from pipeline.");
        return 1;
    }

    param_server.set_property(cam1, "Brightness", "Brightness", "int");
    param_server.set_property(cam1, "GPIn", "GPIn", "int");
    param_server.set_property(cam1, "GPOut", "GPOut", "int");
    param_server.set_property(cam1, "GainDB100", "Gain (dB/100)", "int");
    param_server.set_property(cam1, "OffsetAutoCenter", "Offset Auto Center", "boolean");
    param_server.set_property(cam1, "OffsetX", "Offset X", "int");
    param_server.set_property(cam1, "OffsetY", "Offset Y", "int");
    param_server.set_property(cam1, "OverrideScanningMode", "Override Scanning Mode", "int");
    param_server.set_property(cam1, "StrobeEnable", "Strobe Enable", "boolean");
    param_server.set_property(cam1, "StrobeExposure", "Strobe Exposure", "boolean");
    param_server.set_property(cam1, "StrobePolarity", "Strobe Polarity", "boolean");
    param_server.set_property(cam1, "TriggerDelayUs", "Trigger Delay (us)", "int");
    param_server.set_property(cam1, "TriggerGlobalResetRelease", "Trigger Global Reset Release", "boolean");
    param_server.set_property(cam1, "TriggerMode", "Trigger Mode", "boolean");
    param_server.set_property(cam1, "cameraWhitebalance", "camera-whitebalance", "boolean");
    param_server.set_property(cam1, "whitebalanceAuto", "whitebalance-auto", "boolean");
    param_server.set_property(cam1, "whitebalanceBlue", "whitebalance-blue", "int");
    param_server.set_property(cam1, "whitebalanceGreen", "whitebalance-green", "int");
    param_server.set_property(cam1, "whitebalanceModuleEnabled", "whitebalance-module-enabled", "boolean");
    param_server.set_property(cam1, "whitebalanceRed", "whitebalance-red", "int");

    param_server.set_format(capsfilter);
    gst_object_unref(capsfilter);

    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    if (!block_until_playing(pipeline))
    {
        cout << "\n"
             << "Unable to start pipeline."
             << "\n"
             << endl;
    }

    param_server.set_property(cam1, "BrightnessReference", "Brightness Reference", "int");
    param_server.set_property(cam1, "ExposureAuto", "Exposure Auto", "boolean");
    param_server.set_property(cam1, "ExposureMin", "Exposure Min", "int");
    param_server.set_property(cam1, "ExposureMax", "Exposure Max", "int");
    param_server.set_property(cam1, "ExposureTimeUs", "Exposure Time (us)", "int");
    param_server.set_property(cam1, "GainAuto", "Gain Auto", "boolean");
    param_server.set_property(cam1, "GainMin", "Gain Min", "double");
    param_server.set_property(cam1, "GainMax", "Gain Max", "double");
    param_server.set_property(cam1, "Gain", "Gain", "int");
    param_server.set_property(cam1, "ExposureROILeft", "Exposure ROI Left", "int");
    param_server.set_property(cam1, "ExposureROIWidth", "Exposure ROI Width", "int");
    param_server.set_property(cam1, "ExposureROITop", "Exposure ROI Top", "int");
    param_server.set_property(cam1, "ExposureROIHeight", "Exposure ROI Height", "int");

    gst_object_unref(cam1);

    ROS_INFO_STREAM("\033[1;32m-> LiveStreamCam1.\033[0m");
    ROS_INFO_STREAM("Press Ctrl-C to stop the stream.");
    while (ros::ok())
    {
    }

    gst_element_set_state(pipeline, GST_STATE_NULL);

    gst_object_unref(pipeline);
    return 0;
}
