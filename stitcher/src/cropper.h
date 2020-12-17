#ifndef CROPPER_H
#define CROPPER_H

#include <vector>

#include <opencv2/core.hpp>

/**
 * @brief The Cropper class
 *  Provides `cropNullEdges` function which accepts an input image and crops them off.
 * The crop function works by eroding
 */
class Cropper
{
public:
    /**
     * @brief cropNullRegions
     *  Search for null (black) regions along the edge of the image.
     * Return an image cropped (reduced in dimensions without stretching) with null
     * regions reduced or removed.
     * @param original
     * @param maximumRatio
     *  Ratio representing the most permissible cropping amount. If the resulting image
     * area has a smaller ratio than the maximum ratio would permit then the original
     * image is returned. In that case the null regions are significant enough they should
     * represent images missing in the panorama.
     * @return
     *  Check if image.empty() -- if true, then crop succeeded but would result in
     * violating maxCropRatio
     * @throws cv::Exception -- the Cropper isn't perfect and OpenCV is pedantic.
     * @throws std::invalid_argument -- image is too small
     * @throws std::logic_error -- the cropping logic failed and needs to be adjusted
     */
    cv::Mat cropNullEdges(const cv::Mat &original, double maximumRatio = 99. / 100);

protected:
    using Contour = std::vector<cv::Point>;
    using Contours = std::vector<Contour>;
    using Areas = std::vector<double>;

    /**
     * @brief CropMask
     *  Rect describing the inner crop region, and a Mat with that region filled white.
     */
    using CropMask = std::pair<cv::Mat, cv::Rect>;

    /**
     * @brief validateMinimumSize
     * Throws if the image shouldn't be processed.
     * @param img
     * @throws std::invalid_argument
     *  image is too small
     */
    void validateMinimumSize(const cv::Mat &img);

    /**
     * @brief buildPanoramaMask
     * Return a mask where black represents null regions and white represents image data.
     * @param original
     * @return cv::Mat containing black-and-white mask; white represents panorama data and
     * black represents null regions
     */
    cv::Mat buildPanoramaMask(const cv::Mat &original);

    /**
     * @brief bestContourRect
     *  Searches for contours and returns the "best" representation of the image excluding
     * null regions.
     * @param img
     * @return
     */
    cv::Rect bestContourRect(const cv::Mat &img);

    /**
     * @brief Cropper::maskContour
     * maskContour gets the minimum bounding rectangle of the image's contour (excluding
     * the null edge). It fills the bounding rectangle with white, returning a new mask.
     * The returned cropping rectangle may intersect null regions and image data along the
     * boundary.
     * @param img
     * @return
     */
    CropMask maskContour(const cv::Mat &img);

    /**
     * @brief Cropper::erodeMask
     * This accepts an input black-and-white mask.
     * It will clone the mask to two temporaries;
     *  * the first will contain the erosion result
     *  * the second will be used to count the number of pixels remaining to be eroded
     * (which are white) In a loop, it will erode the second, subtract the input, and save
     * to the first. It repeats while any white pixels remain outside of the erosion
     * boundary.
     *
     * The returned image is strictly a tighter view of the input mask.
     * The erosion reduces noise (null region intersections) along the edge by tightening
     * the view.
     *
     * The OpenCV tutorial on erosion describes the effect with some visual aid:
     * https://docs.opencv.org/4.2.0/db/df6/tutorial_erosion_dilatation.html
     * @param input_black_and_white_mask
     * @return
     */
    cv::Mat erodeMask(const cv::Mat &input_black_and_white_mask);

    /**
     * @brief Cropper::findErosionBoundary
     *  Search for the "best" boundary along a mask, erode it to reduce noise, and return
     * a bounding rectangle for it.
     * @param uneroded_mask
     * @param original
     * @return
     */
    cv::Rect findErosionBoundary(const cv::Mat &uneroded_mask, const cv::Mat &original);

    /**
     * @brief Cropper::borderPixels
     *  A border will be added during the cropping function.
     *  The size of the border affects algorithmic performance.
     *  Too small will result in the crop function failing.
     *  Too large will result in the crop function performing slowly.
     */
    int borderPixels_ = 10;

    // 16 is a super small number, power of two, easy to reason about.
    // I like powers of two. I wanted it bigger than `borderPixels` because I wanted to
    // still have room after subtracting it from the image size. I didn't want to have a
    // large number (say, > 1000) and risk rejecting a valid-but-small panorama. I didn't
    // put a lot of thought put into it beyond that.
    const int minwidth_ = 16;
    const int minheight_ = 16;

    // Cache to avoid reallocations
    Contours contours_;
    Areas areas_;
};

#endif // CROPPER_H
