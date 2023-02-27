#pragma once

#include <vector>
#include <FAST/Object.hpp>
#include <FAST/Data/DataTypes.hpp>

namespace fast {

/**
 * @brief Object representing the shape of a Tensor
 *
 * @ingroup neural-network
 * @sa Tensor
 */
class FAST_EXPORT TensorShape {
    public:
#ifndef SWIG
        /**
         * Construct tensor shape
         * @param dimensions
         */
        TensorShape(std::initializer_list<int> dimensions);
        /**
         * Construct tensor shape
         * @param dimensions
         */
        explicit TensorShape(VectorXi dimensions);

        /**
         * Copy constructor
         * @param other
         */
        TensorShape(const TensorShape& other);
        /**
         * Assignment constructor
         * @param other
         * @return
         */
        TensorShape& operator=(const TensorShape& other);
#endif
        /**
        * Construct tensor shape
        * @param dimensions
        */
        explicit TensorShape(std::vector<int> dimensions = std::vector<int>());
        /**
         * If shape is empty or not
         */
        bool empty() const;
        /**
         * Total size of tensor, excluding any unknown dimensions
         * @return
         */
        int getTotalSize() const;
        /**
         * Get nr of dimensions
         * @return
         */
        int getDimensions() const;
        /**
         * Get nr of known dimensions (dimension not -1)
         * @return
         */
        int getKnownDimensions() const;
        /**
         * Get nr of unknown dimensions (dimensions that are -1)
         */
        int getUnknownDimensions() const;
        /**
         * Get all dimensions as vector
         * @return
         */
        std::vector<int> getAll() const;
        int& operator[](int i);
        const int& operator[](int i) const;
        /**
         * Set dimension i to value
         * @param i
         * @param value
         */
        void setDimension(int i, int value);
        /**
         * Add a dimension to the end of the shape with value
         * @param value
         */
        void addDimension(int value);
		/**
		 * Insert a new dimension at a given position with the given value
		 * @param position
		 * @param value
		 */
		void insertDimension(int position, int value);
        /**
         * Convert shape into string
         */
        std::string toString() const;

        /**
         * Delete a dimension
         * @param index
         */
        void deleteDimension(int index);

        /**
         * Delete a range of dimensions
         * @param startIndex
         * @param endIndex
         */
        void deleteDimensions(int startIndex, int endIndex);
    private:
        std::vector<int> m_data;
};

}