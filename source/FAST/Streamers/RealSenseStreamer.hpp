#ifndef FAST_KINECT_STREAMER_HPP_
#define FAST_KINECT_STREAMER_HPP_

#include "FAST/ProcessObject.hpp"
#include "Streamer.hpp"
#include <thread>
#include <stack>
#include <librealsense2/rs.hpp>

struct rs2_intrinsics;

namespace fast {

class Image;
class MeshVertex;

class FAST_EXPORT RealSenseFilter{
    public:
        enum FilterType{DECIMATE, SPATIAL, TEMPORAL, DEPTH_TO_DISPARITY, DISPARITY_TO_DEPTH};

        RealSenseFilter(FilterType type, rs2::filter& filter);
        void setOption(rs2_option option, float value);
        std::vector<rs2_option> getFilterOptions(){ return mOptions;};
        bool isEnabled(){ return mEnabled;};
        rs2::frame process(rs2::frame frame);
        FilterType getFilterType(){ return mFilterType;};

    private:
        FilterType mFilterType;
        rs2::filter& mFilter;
        std::vector<rs2_option> mOptions;
        bool mEnabled = false;
};


/**
 * \brief Streams data RGB and depth data from a kinect device.
 *
 * The RGB camera and depth stream are registered so that a color value for each point in the
 * point cloud is established.
 *
 * Output port 0: Registered RGB image
 * Output port 1: Registered depth image
 * Output port 2: Registered point cloud
 */

class FAST_EXPORT RealSenseStreamer : public Streamer {
    FAST_OBJECT(RealSenseStreamer);

    public:
        void setMaxRange(float range);
        void setMinRange(float range);
        void setMaxWidth(float range);
        void setMinWidth(float range);
        void setMaxHeight(float range);
        void setMinHeight(float range);

        uint getNrOfFrames() const;
        MeshVertex getPoint(int x, int y);

        void setFilterValue(RealSenseFilter::FilterType filter, rs2_option option, float value);
        std::vector<MeshVertex> rsPointsToVertices(rs2::points points, rs2::frame color_frame);

        ~RealSenseStreamer();
    private:
        RealSenseStreamer();

        void execute();
        void generateStream() override;

        float mMaxRange = std::numeric_limits<float>::max();
        float mMinRange = 0;
        float mMaxWidth = std::numeric_limits<float>::max();
        float mMinWidth = -std::numeric_limits<float>::max();
        float mMaxHeight = std::numeric_limits<float>::max();
        float mMinHeight = -std::numeric_limits<float>::max();

        uint mNrOfFrames;

        std::vector<RealSenseFilter> mFilters;
        rs2_intrinsics* intrinsics;
        SharedPointer<Image> mDepthImage;
        SharedPointer<Image> mColorImage;
};

}

#endif
