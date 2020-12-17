#pragma once
#include <opencv2/opencv.hpp>

class CubeMap
{
public:
    // Define our six cube faces.
    // 0 - 3 are side faces, clockwise order starting front
    // 4 and 5 are top and bottom, respectively
    enum class Face {
        Front,
        Right,
        Back,
        Left,
        Top,
        Bottom,

        NumFaces
    };

    enum class FlipCode { X, Y, Both, None };
    using Paths = std::map<Face, std::string>;

    /**
     * @brief write creates and writes all cube faces out to the given paths
     * @param in - the equirectangular panorama to break down into faces
     * @param paths - face->path map
     */
    static void write(const cv::Mat &in, const Paths &paths);

    static void writeFace(const cv::Mat &in, Face faceId, const std::string &out, int width,
                          int height, FlipCode flipCode);
    static void createFace(const cv::Mat &in, cv::Mat &face, Face faceId, int width,
                           int height);
};
