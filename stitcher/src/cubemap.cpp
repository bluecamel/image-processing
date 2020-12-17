#include "cubemap.h"
#include <cmath>

void CubeMap::write(const cv::Mat &input, const Paths &paths)
{
    size_t face_dimension = input.cols / 4;

    writeFace(input, Face::Front, paths.at(Face::Front), face_dimension, face_dimension,
              FlipCode::None);
    writeFace(input, Face::Right, paths.at(Face::Right), face_dimension, face_dimension,
              FlipCode::None);
    writeFace(input, Face::Back, paths.at(Face::Back), face_dimension, face_dimension,
              FlipCode::None);
    writeFace(input, Face::Left, paths.at(Face::Left), face_dimension, face_dimension,
              FlipCode::None);
    writeFace(input, Face::Top, paths.at(Face::Top), face_dimension, face_dimension,
              FlipCode::Y);
    writeFace(input, Face::Bottom, paths.at(Face::Bottom), face_dimension, face_dimension,
              FlipCode::X);
}

void CubeMap::writeFace(const cv::Mat &in, Face faceId, const std::string &pathOut,
                        int width, int height, FlipCode flipCode)
{
    cv::Mat out;
    createFace(in, out, faceId, width, height);
    if (flipCode != FlipCode::None) {
        cv::transpose(out, out);
        cv::flip(out, out, static_cast<int>(flipCode));
    }
    cv::imwrite(pathOut, out);
}

/*
 * Code found:
 * https://stackoverflow.com/questions/29678510/convert-21-equirectangular-panorama-to-cube-map/34720686#34720686
 * and after https://stackoverflow.com/help/licensing, presumed licensed under the
 * CC BY-SA 3.0 (see https://creativecommons.org/licenses/by-sa/3.0/)
 */
float faceTransform[6][2] = { { 0, 0 },         { M_PI / 2, 0 },  { M_PI, 0 },
                              { -M_PI / 2, 0 }, { 0, -M_PI / 2 }, { 0, M_PI / 2 } };

void CubeMap::createFace(const cv::Mat &in, cv::Mat &face, Face faceId, int width,
                         int height)
{
    const float inWidth = in.cols;
    const float inHeight = in.rows;
    assert(in.size().height == in.size().width / 2);

    // Allocate map
    cv::Mat mapx(height, width, CV_32F);
    cv::Mat mapy(height, width, CV_32F);

    // Calculate adjacent (ak) and opposite (an) of the
    // triangle that is spanned from the sphere center
    // to our cube face.
    const float an = sin(M_PI / 4);
    const float ak = cos(M_PI / 4);

    const float ftu = faceTransform[static_cast<int>(faceId)][0];
    const float ftv = faceTransform[static_cast<int>(faceId)][1];

    // For each point in the target image,
    // calculate the corresponding source coordinates.
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {

            // Map face pixel coordinates to [-1, 1] on plane
            float nx = (float)y / (float)height - 0.5f;
            float ny = (float)x / (float)width - 0.5f;

            nx *= 2;
            ny *= 2;

            // Map [-1, 1] plane coords to [-an, an]
            // thats the coordinates in respect to a unit sphere
            // that contains our box.
            nx *= an;
            ny *= an;

            float u, v;

            // Project from plane to sphere surface.
            if (ftv == 0) {
                // Center faces
                u = atan2(nx, ak);
                v = atan2(ny * cos(u), ak);
                u += ftu;
            } else if (ftv > 0) {
                // Bottom face
                float d = sqrt(nx * nx + ny * ny);
                v = M_PI / 2 - atan2(d, ak);
                u = atan2(ny, nx);
            } else {
                // Top face
                float d = sqrt(nx * nx + ny * ny);
                v = -M_PI / 2 + atan2(d, ak);
                u = atan2(-ny, nx);
            }

            // Map from angular coordinates to [-1, 1], respectively.
            u = u / (M_PI);
            v = v / (M_PI / 2);

            // Warp around, if our coordinates are out of bounds.
            while (v < -1) {
                v += 2;
                u += 1;
            }
            while (v > 1) {
                v -= 2;
                u += 1;
            }

            while (u < -1) {
                u += 2;
            }
            while (u > 1) {
                u -= 2;
            }

            // Map from [-1, 1] to in texture space
            u = u / 2.0f + 0.5f;
            v = v / 2.0f + 0.5f;

            u = u * (inWidth - 1);
            v = v * (inHeight - 1);

            // Save the result for this pixel in map
            mapx.at<float>(x, y) = u;
            mapy.at<float>(x, y) = v;
        }
    }

    // Recreate output image if it has wrong size or type.
    if (face.cols != width || face.rows != height || face.type() != in.type()) {
        face = cv::Mat(width, height, in.type());
    }

    // Do actual resampling using OpenCV's remap
    cv::remap(in, face, mapx, mapy, cv::INTER_CUBIC, cv::BORDER_CONSTANT,
              cv::Scalar(0, 0, 0));
}
