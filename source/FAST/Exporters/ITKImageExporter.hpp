#pragma once

#include "FAST/ProcessObject.hpp"
#include "FAST/Data/Image.hpp"

#include <itkImageSource.h>
#include <itkImageRegionIterator.h>

namespace fast {

/**
 * @brief Export a FAST Image to an ITK image
 *
 * This can be used to connect a FAST pipeline to an ITK pipeline.
 * @warning This class is not included in the release builds.
 *
 * <h3>Input ports</h3>
 * 0: Image
 *
 * @ingroup exporters
 * @tparam TImage
 */
template<class TImage>
class FAST_EXPORT  ITKImageExporter: public itk::ImageSource<TImage>, public ProcessObject {
    public:
        /** Standard class typedefs. */
        typedef ITKImageExporter Self;
        typedef itk::ImageSource<TImage> Superclass;
        typedef itk::SmartPointer<Self> Pointer;

        /** Method for creation through the object factory. */
        itkNewMacro(Self);

        /** Run-time type information (and related methods). */
        itkTypeMacro(MyImageSource, ImageSource);

        std::string getNameOfClass() const { return "ITKImageExporter"; };
    private:
        ITKImageExporter();
        void execute() {};

        // Is called by ITK
        void GenerateData();

        template <class T>
        void transferDataToITKImage(Image::pointer input);

};

} // end namespace fast

template<class TImage>
fast::ITKImageExporter<TImage>::ITKImageExporter() {
    createInputPort<Image>(0);
}

template <class TImage>
template <class T>
void fast::ITKImageExporter<TImage>::transferDataToITKImage(Image::pointer input) {

    typename TImage::Pointer output = this->GetOutput();
    typename TImage::RegionType region;
    typename TImage::IndexType start;
    start[0] = 0;
    start[1] = 0;
    if(input->getDimensions() == 3)
        start[2] = 0;

    typename TImage::SizeType size;
    size[0] = input->getWidth();
    size[1] = input->getHeight();
    if(input->getDimensions() == 3)
        size[2] = input->getDepth();

    region.SetSize(size);
    region.SetIndex(start);

    output->SetRegions(region);
    output->Allocate();

    ImageAccess::pointer access = input->getImageAccess(ACCESS_READ);
    T * fastPixelData = (T*)access->get();

    itk::ImageRegionIterator<TImage> imageIterator(output,
            output->GetLargestPossibleRegion());
    unsigned int width = input->getWidth();
    if(input->getDimensions() == 2) {
        while (!imageIterator.IsAtEnd()) {
            unsigned int x = imageIterator.GetIndex()[0];
            unsigned int y = imageIterator.GetIndex()[1];
            imageIterator.Set(fastPixelData[x + y*width]);

            ++imageIterator;
        }
    } else {
        unsigned int height = input->getHeight();

        while (!imageIterator.IsAtEnd()) {
            unsigned int x = imageIterator.GetIndex()[0];
            unsigned int y = imageIterator.GetIndex()[1];
            unsigned int z = imageIterator.GetIndex()[2];
            imageIterator.Set(fastPixelData[x + y*width + z*width*height]);

            ++imageIterator;
        }
    }
}

template<class TImage>
void fast::ITKImageExporter<TImage>::GenerateData() {

    update(); // Update FAST pipeline

    Image::pointer input = getInputData<Image>();
    if(input->getDimensions() != TImage::ImageDimension)
        throw Exception("The dimension of the input and output images of the ITKImageExporter are unequal.");

    if(input->getNrOfChannels() != 1)
        throw Exception("The ITKImageExporter currently doesn't support images with multiple components");


    // Transfer data from mInput to vtk image
    switch(input->getDataType()) {
        fastSwitchTypeMacro(transferDataToITKImage<FAST_TYPE>(input))
    }


}
