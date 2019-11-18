#include "RealSenseStreamer.hpp"
#include "FAST/Data/Image.hpp"
#include "FAST/Data/Mesh.hpp"
#include "librealsense2/rsutil.h"
#include <atomic>

namespace fast {

RealSenseStreamer::RealSenseStreamer() {
    createOutputPort<Image>(0); // RGB
    createOutputPort<Image>(1); // Depth image
    createOutputPort<Mesh>(2); // Point cloud
    mNrOfFrames = 0;
    mIsModified = true;
}

void RealSenseStreamer::execute() {
    startStream();
    waitForFirstFrame();
}


MeshVertex RealSenseStreamer::getPoint(int x, int y) {
    if(mDepthImage && mColorImage) {
        float upoint[3];
        float upixel[2] = {(float) x, (float) y};
        auto depthAccess = mDepthImage->getImageAccess(ACCESS_READ);
        float depth = depthAccess->getScalar(Vector2i(x, y)) / 1000.0f; // Get depth of current frame in meters
        rs2_deproject_pixel_to_point(upoint, intrinsics, upixel, depth);
        MeshVertex vertex(Vector3f(upoint[0] * 1000, upoint[1] * 1000, upoint[2] * 1000)); // Convert to mm
        auto colorAccess = mColorImage->getImageAccess(ACCESS_READ);
        Vector4f color = colorAccess->getVector(Vector2i(x, y));
        vertex.setColor(Color(color.x()/255.0f, color.y()/255.0f, color.z()/255.0f));  // get color of current frame
        return vertex;
    } else {
        throw Exception("Can't call getPoint before any color or depth data has arrived.");
    }
}

static float get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw Exception("Device does not have a depth sensor");
}

std::array<uint8_t, 3> get_texcolor(const rs2::video_frame& texture, float u, float v)
{
    const int w = texture.get_width(), h = texture.get_height();
    int x = std::min(std::max(int(u*w + .5f), 0), w - 1);
    int y = std::min(std::max(int(v*h + .5f), 0), h - 1);
    int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
    const auto texture_data = reinterpret_cast<const uint8_t*>(texture.get_data());
    return { texture_data[idx], texture_data[idx + 1], texture_data[idx + 2] };
}

void RealSenseStreamer::generateStream() {
    reportInfo() << "Trying to set up real sense stream..." << reportEnd();

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline pipeline;

    rs2::config config;
    // Use a configuration object to request only depth from the pipeline
    config.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);
    config.enable_stream(RS2_STREAM_COLOR, 848, 480, RS2_FORMAT_RGB8, 30);

    // Configure and start the pipeline
    rs2::pipeline_profile profile;
    try {
        profile = pipeline.start(config);
    } catch(rs2::error &e) {
        throw Exception("Error could not start real sense streaming pipeline: " + std::string(e.what()));
    }

    const float depth_scale = get_depth_scale(profile.get_device());

    auto stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto tmp = stream.get_intrinsics();
    intrinsics = &tmp;

    //
    rs2::pointcloud pc;
    rs2::points pc_points;

    // Create a rs2::align object.
    // rs2::align allows us to perform alignment of depth frames to others frames
    //The "align_to" is the stream type to which we plan to align depth frames.
    rs2::align align(RS2_STREAM_COLOR);

    // Declare filters
    rs2::decimation_filter decimation_filter;  // Decimation - reduces depth frame density
    rs2::spatial_filter spat_filter;    // Spatial    - edge-preserving spatial smoothing
    rs2::temporal_filter temp_filter;   // Temporal   - reduces temporal noise

    // Declare disparity transform from depth to disparity and vice versa
    rs2::disparity_transform depth_to_disparity(true);
    rs2::disparity_transform disparity_to_depth(false);

    mFilters.emplace_back(RealSenseFilter::FilterType::DECIMATE, decimation_filter);
    mFilters.emplace_back(RealSenseFilter::FilterType::DEPTH_TO_DISPARITY, depth_to_disparity);
    mFilters.emplace_back(RealSenseFilter::FilterType::SPATIAL, spat_filter);
    mFilters.emplace_back(RealSenseFilter::FilterType::TEMPORAL, temp_filter);
    mFilters.emplace_back(RealSenseFilter::FilterType::DISPARITY_TO_DEPTH, disparity_to_depth);

    // Declaring two concurrent queues that will be used to push and pop frames from different threads
    rs2::frame_queue color_data_queue;
    rs2::frame_queue depth_data_queue;

    // Atomic boolean to allow thread safe way to stop the thread
    std::atomic_bool stopped(false);

    // Create a thread for getting frames from the device and process them
    // to prevent UI thread from blocking due to long computations.
    std::thread processing_thread([&]() {
        while (!stopped) //While application is running
        {
            rs2::frameset frames = pipeline.wait_for_frames(); // Wait for next set of frames from the camera
            //Get processed aligned frame
            rs2::frameset processed = align.process(frames);

            // Trying to get both other and aligned depth frames
            rs2::video_frame color_frame = processed.get_color_frame();
            rs2::depth_frame depth_frame = processed.get_depth_frame(); // Aligned depth frame

            //If one of them is unavailable, continue iteration
            if(!depth_frame || !color_frame)
                continue;

            rs2::frame filtered = depth_frame; // Does not copy the frame, only adds a reference

            /* Apply filters.
            The implemented flow of the filters pipeline is in the following order:
            1. (apply decimation filter)
            2. transform the scence into disparity domain
            3. apply spatial filter
            4. apply temporal filter
            5. revert the results back (if step Disparity filter was applied
            to depth domain (each post processing block is optional and can be applied independantly).
            */

            for (auto&& filter : mFilters)
            {
                if (filter.isEnabled())
                    filtered = filter.process(filtered);
            }

            //filtered = decimation_filter.process(filtered);
            //filtered = depth_to_disparity.process(filtered);
            //filtered = spat_filter.process(filtered);
            //filtered = temp_filter.process(filtered);
            //filtered = disparity_to_depth.process(filtered);

            // Push filtered & original data to their respective queues
            // Note, pushing to two different queues might cause the application to display
            //  original and filtered pointclouds from different depth frames
            //  To make sure they are synchronized you need to push them together or add some
            //  synchronization mechanisms
            depth_data_queue.enqueue(filtered);
            color_data_queue.enqueue(color_frame);
        }
    });


    while(true) {
        {
            // Check if stop signal is sent
            std::unique_lock<std::mutex> lock(m_stopMutex);
            if(m_stop) {
                m_streamIsStarted = false;
                m_firstFrameIsInserted = false;
                break;
            }
        }

        // Block program until frames arrive
        rs2::frame depth_frame;
        depth_data_queue.poll_for_frame(&depth_frame);

        rs2::frame color_frame;
        color_data_queue.poll_for_frame(&color_frame);

        if(!color_frame || !depth_frame)
            continue;

        pc.map_to(color_frame);
        pc_points = pc.calculate(depth_frame);

        auto vertices = pc_points.get_vertices();
        auto texcoords = pc_points.get_texture_coordinates();

        std::vector<MeshVertex> points;
        for(size_t i = 0; i < pc_points.size(); ++i)
        {
            if (!vertices[i].z || vertices[i].z*1000 > mMaxRange || vertices[i].z*1000 < mMinRange)
                continue;

            if(vertices[i].x*1000 > mMaxWidth || vertices[i].x*1000 < mMinWidth)
                continue;

            if(vertices[i].y*1000 > mMaxHeight || vertices[i].y*1000 < mMinHeight)
                continue;

            MeshVertex vertex(Vector3f(vertices[i].x*1000, vertices[i].y*1000, vertices[i].z*1000)); // Convert to mm
            auto rgb = get_texcolor(color_frame, texcoords[i].u, texcoords[i].v);
            vertex.setColor(Color(rgb[0]/255.0f, rgb[1]/255.0f, rgb[2]/255.0f));
            points.push_back(vertex);
        }

        // Create depth image
        int width = static_cast<rs2::video_frame>(depth_frame).get_width();
        int height = static_cast<rs2::video_frame>(depth_frame).get_height();

        std::unique_ptr<uint16_t[]> depthData = std::make_unique<uint16_t[]>(width*height);
        std::memcpy(depthData.get(), depth_frame.get_data(), width*height*2);

        Image::pointer depthImage = Image::New();
        depthImage->create(width, height, TYPE_UINT16, 1, std::move(depthData));
        depthImage->setCreationTimestamp(depth_frame.get_frame_metadata(RS2_FRAME_METADATA_BACKEND_TIMESTAMP));
        mDepthImage = depthImage;

        // Create mesh
        Mesh::pointer cloud = Mesh::New();
        cloud->create(points);
        cloud->setCreationTimestamp(depth_frame.get_frame_metadata(RS2_FRAME_METADATA_BACKEND_TIMESTAMP));

        // Create RGB camera image
        width = static_cast<rs2::video_frame>(color_frame).get_width();
        height = static_cast<rs2::video_frame>(color_frame).get_height();
        std::unique_ptr<uint8_t[]> colorData = std::make_unique<uint8_t[]>(width*height*3);
        std::memcpy(colorData.get(), color_frame.get_data(), width*height*sizeof(uint8_t)*3);
        Image::pointer colorImage = Image::New();
        colorImage->create(width, height, TYPE_UINT8, 3, std::move(colorData));
        colorImage->setCreationTimestamp(color_frame.get_frame_metadata(RS2_FRAME_METADATA_BACKEND_TIMESTAMP));
        mColorImage = colorImage;

        try {
            addOutputData(0, colorImage);
            addOutputData(1, depthImage);
            addOutputData(2, cloud);
        } catch(ThreadStopped &e) {
            break;
        }

        frameAdded();
        mNrOfFrames++;
    }

    stopped = true;
    processing_thread.join();

    reportInfo() << "Real sense streamer stopped" << Reporter::end();
}

void RealSenseStreamer::setFilterValue(RealSenseFilter::FilterType filterType, rs2_option option, float value)
{
    for(auto filter: mFilters)
    {
        if(filter.getFilterType() == filterType)
        {
            filter.setOption(option, value);
        }
    }
}

uint RealSenseStreamer::getNrOfFrames() const {
    return mNrOfFrames;
}

RealSenseStreamer::~RealSenseStreamer() {
    stop();
}

void RealSenseStreamer::setMaxRange(float range) {
    if(range < 0)
        throw Exception("Range has to be >= 0");
    mMaxRange = range;
}

void RealSenseStreamer::setMinRange(float range) {
    if(range < 0)
        throw Exception("Range has to be >= 0");
    mMinRange = range;
}

void RealSenseStreamer::setMaxWidth(float range) {
    mMaxWidth = range;
}

void RealSenseStreamer::setMinWidth(float range) {
    mMinWidth = range;
}

void RealSenseStreamer::setMaxHeight(float range) {
    mMaxHeight = range;
}

void RealSenseStreamer::setMinHeight(float range) {
    mMinHeight = range;
}


RealSenseFilter::RealSenseFilter(FilterType type, rs2::filter &filter):
    mFilterType(type),
    mFilter(filter),
    mEnabled(true)
{
    mOptions = filter.get_supported_options();
}

void RealSenseFilter::setOption(rs2_option option, float value)
{
    mFilter.set_option(option, value);
}

rs2::frame RealSenseFilter::process(rs2::frame frame)
{
    return mFilter.process(frame);
}

}