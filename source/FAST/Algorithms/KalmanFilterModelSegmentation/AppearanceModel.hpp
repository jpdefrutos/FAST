#ifndef APPEARANCE_MODEL_HPP
#define APPEARANCE_MODEL_HPP

#include "FAST/Data/DataTypes.hpp"
#include "Shape.hpp"

namespace fast {

class Image;

class FAST_EXPORT  Measurement {
	public:
		float displacement;
		float uncertainty;
};

/**
 * This is a base class for appearance models.
 * These classes model of an object appears in an image.
 * They are used by the Kalman filter to collect measurements.
 */
class FAST_EXPORT  AppearanceModel : public Object {
	public:
		typedef std::shared_ptr<AppearanceModel> pointer;
		virtual std::vector<Measurement> getMeasurements(std::shared_ptr<Image> image, std::shared_ptr<Shape> shape, ExecutionDevice::pointer device) = 0;


};

}

#endif
