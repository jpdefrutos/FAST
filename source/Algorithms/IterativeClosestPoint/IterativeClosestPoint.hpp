#ifndef ITERATIVE_CLOSEST_POINT_HPP
#define ITERATIVE_CLOSEST_POINT_HPP

#include "ProcessObject.hpp"
#include "LinearTransformation.hpp"
#include "PointSet.hpp"

namespace fast {

class IterativeClosestPoint : public ProcessObject {
    FAST_OBJECT(IterativeClosestPoint)
    public:
        typedef enum { RIGID, TRANSLATION } TransformationType;
        void setFixedPointSetPort(ProcessObjectPort port);
        void setFixedPointSet(PointSet::pointer data);
        void setMovingPointSetPort(ProcessObjectPort port);
        void setMovingPointSet(PointSet::pointer data);
        void setTransformationType(const IterativeClosestPoint::TransformationType type);
        LinearTransformation getOutputTransformation();
        float getError() const;
    private:
        IterativeClosestPoint();
        void execute();

        float mMinErrorChange;
        uint mMaxIterations;
        float mError;
        LinearTransformation mTransformation;
        IterativeClosestPoint::TransformationType mTransformationType;
};

} // end namespace fast

#endif
