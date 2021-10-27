# Panorama Stitcher
This project takes a set of images and stitches them into a panorama.  It started life as a fork of the OpenCV [detail stitching example](https://docs.opencv.org/4.2.0/d9/dd8/samples_2cpp_2stitching_detailed_8cpp-example.html), but has been refactored and extended with additional features, including:
- cubemap generation
- detailed debugging
- memory budgeting
- elapsed and estimated time
- camera-specific undistortion for pinhole and omnidirectional cameras
- rotation correction for 360 panoramas that are generated upside down

Currently, only 360 (spherical) panoramas are supported, but care has been taken to make sure that adding [other types of panoramas](#additional-panorama-types) will be a relatively straightforward task.


# Basic Usage
A basic stitch can be performed by simply providing the images:
```
./airmap_stitcher --input_path /path/to/images/*.jpg
```

By default, OpenCL will be used, if available.  Currently, stitches fail on some hardware.  In this case, OpenCL can be disabled:
```
./airmap_stitcher --input_path /path/to/images/*.jpg --disable_opencl
```

# Debugging
Detailed debugging output can be generated:
```
./airmap_stitcher --input_path /path/to/images/*.jpg --debug --debug_path debug_a
```

This will create a directory named `debug_a` and store artifacts from many of the operations, including:
- detected features
- matched features for each image pair
- a graphviz graph of matched images
- warped images

# Full Usage
```
./airmap_stitcher [OPTIONS] image1.jpg image2.jpg ... imageN.jpg
stitches the given images into an equirectangular 360 panorama:
  --help                         show help
  --input arg                    input images
  --input_path                   input images will be sought for in this folder
  --output arg (=./panorama.jpg) path to the resulting equirectangular 
                                 stitching
  --cubemap                      if set, also generates cubemap in 
                                 <output>.<face>.jpg
  --ram_budget arg (=28835)      RAM buget (in MB) the stitcher can assume it 
                                 can use
  --retries arg (=6)             The stitching process is non-deterministic, 
                                 this increases chances to succeed
  --debug                        If set, debug artifacts (e.g. detected 
                                 features, matches, warping, etc.) will be 
                                 created in <debug_output>.
  --debug_path arg (=debug)      Path to the debug output.
  --disable_opencl               If set, OpenCL will be disabled.
  --elapsed_time                 Track elapsed times of the main stitch 
                                 operations.  Always enabled if 
                                 elapsed_time_log is.
  --elapsed_time_log             Log elapsed times of stitch operations.
  --estimate                     Estimate time remaining during stitch.  Always
                                 enabled if elapsed_time or estimate_log are.
  --estimate_log                 Log estimates of remaining time.  Always 
                                 enabled if elapsed_time_log is.
```

# Camera Calibration and Distortion Models
For best results, calibration should be performed and a [camera model](src/camera_models.cpp) defined for each camera used.  The model consists of an [intrinsic matrix](https://learnopencv.com/tag/intrinsic-matrix) and an optional [distortion model](https://learnopencv.com/understanding-lens-distortion/).

When the images are opened, EXIF data is read.  If the camera model in the EXIF data matches one of the defined cameras models, and the camera model has a distortion model, then the images will be undistorted before any other processing.

## Brown-Conrady (Pinhole Distortion)
**NOTE**: Poor semantics were used when naming the `PinholeDistortionModel`.  It *corrects for* deviations from a pinhole model.  It probably should be renamed to `BrownConradyDistortionModel`.

A pinhole camera is a camera without a lens and therefore has no distortion.  Here, it is used to describe an ideal camera, and distortion is defined as deviation from that ideal.  Likewise, undistortion is a process of reversing the distortion so that the image (ideally) appears to have been captured with a pinhole camera.

For many cameras/lenses, this distortion is (mostly) [radially symmetric](https://en.wikipedia.org/wiki/Distortion_(optics)#Radial_distortion), due to the symmetry of the lens, and can be categorized as either barrel or pincushion distortion (and generally a combination of both).  Additionally, there may be some tangential distortion caused by the lens and sensor plane not being perfectly parallel.

For these cameras, we can simply use OpenCV's [calibration example](https://github.com/opencv/opencv/blob/bda89a6469aa79ecd8713967916bd754bff1d931/samples/cpp/calibration.cpp) and [undistort](https://docs.opencv.org/4.2.0/d9/d0c/group__calib3d.html#ga69f2545a8b62a6b0fc2ee060dc30559d), which is an implementation of the Brown-Conrady distortion model.

Build OpenCV with the option to build the examples.  For example:
```
cmake -DCMAKE_BUILD_TYPE=Debug -DWITH_OPENGL=ON -DFORCE_VTK=ON -DWITH_TBB=ON -DWITH_GDAL=ON -DWITH_XINE=ON -DENABLE_PRECOMPILED_HEADERS=OFF -DBUILD_EXAMPLES=ON -DOPENCV_ENABLE_NONFREE:BOOL=ON -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

Create in image list:
```
./bin/example_cpp_imagelist_creator camera_name_images.yml *.jpg
```

Perform calibration:
```
./bin/example_cpp_calibration -w=8 -h=6 -s=0.025 -o=camera_name.yml -op -oe camera_name_images.yml
```

The resulting `camera_name.yml` should contain the intrinsic matrix and distortion coefficients:
```
camera_matrix: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 2.9803291905661031e+03, 0., 2.3574166587045511e+03, 0.,
       2.9704815213723482e+03, 1.6709805782521339e+03, 0., 0., 1. ]
distortion_coefficients: !!opencv-matrix
   rows: 5
   cols: 1
   dt: d
   data: [ 2.8391010208309218e-02, -2.7239202041003809e-02,
       -2.4700935014356916e-03, 6.1345950301455029e-03, 0. ]
```


## Fisheye/Omnidirectional
Omnidirectional cameras are more complicated.  OpenCV has a fisheye model, but we didn't have good results with it, particularly for the Vantage Vesper.  Instead, this project includes an implementation of a model from [OCamCalib:  Omnidirectional Camera Calibration Toolbox for Matlab](https://sites.google.com/site/scarabotix/ocamcalib-omnidirectional-camera-calibration-toolbox-for-matlab?authuser=0).

OCamCalib is written for a much older version of OpenCV, so this project also includes a set of patches and a script to download the source, apply patches for a modern version of OpenCV (tested with 4.2.0 and 4.5.3), and compile the corner finding binary:
```
cd calibration/ocam_calib
./setup.sh
```

Copy calibration images to the `Scaramuzza_OCamCalib_v3.0_win` directory:
```
cp /path/to/calibration/images/*.jpg Scaramuzza_OCamCalib_v3.0_win/
```

Then, matlab can be run from that directory:
```
cd Scaramuzza_OCamCalib_v3.0_win
matlab 
```

On Ubuntu 20.04, matlab may need to be forced to use the system `libstdc++.so.6`:
```
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6
matlab
```

Within matlab, OCamCalib can now be run:
```
ocam_calib
```

### Tips
- Detailed instructions are available on the [project site](https://sites.google.com/site/scarabotix/ocamcalib-omnidirectional-camera-calibration-toolbox-for-matlab?authuser=0).
- Be careful to review the detected corners, and either recapture images or manually refine the corners.
- Once a calibration is done, corners can be further refined in OCamCalib, and calibration run again to improve results.

## Capturing Calibration Images
Download a [calibration pattern](https://markhedleyjones.com/projects/calibration-checkerboard-collection) (e.g. [A4 - 25mm squares - 8x6 verticies, 9x7 squares](https://raw.githubusercontent.com/MarkHedleyJones/markhedleyjones.github.io/master/media/calibration-checkerboard-collection/Checkerboard-A4-25mm-8x6.pdf)) and print it.  Make sure that the image is not scaled when printing (print at 100% scale).

The pattern needs to be as flat as possible, so printing on heavier paper or cardstock might help.  The pattern can also be taped to a flat piece of plywood or something else that is flat and rigid.

Keeping the camera in a fixed location, capture images of the calibration pattern at different angles and different positions in the frame.  Capture at least 30 images, but more images and angles will usually give better results.

### Tips:
* Account for as much of the image frame as possible.
* Hold the pattern at a variety of orientations (e.g. parallel to the camera sensor plane, angled forward and backwards in each cardinal direction, and repeated in many positions in the frame).
* Don’t move the camera between images.
* Don’t change focus (disable autofocus, if possible).
* In each image, the checkerboard should cover at least 20% of the frame.
* Make sure that the entire checkerboard pattern is in view (e.g. not cut off at an edge or at too extreme of an angle).
* Don’t scale or crop the images after capture.

## Extracting frames from video.
This method should be avoided, if possible, and still images used instead.  However, if all you have is video, you can used the included script to extract frames where corners are detected:
```
cd calibration
./extract_video_frames.py --input_path /path/to/video.mp4 --fps 5 --board_width 8 --board_height 6 --output /path/to/frames
```

Note that the video resolution will likely be different from still image resolution.  The resulting intrinsic matrix and distortion coefficients will need to be adjusted to match the scale of the images to be stitched.

# Main Operations
- Undistort Images
  - If input images match (via EXIF) a defined camera model which has a distortion model, then the before anything else.
- Scale Images
  - An attempt is made to scale images based on input image size and available memory.
- Find Features
  - Find features in all input images.  By default, [OpenCV's ORB features](https://docs.opencv.org/4.2.0/db/d95/classcv_1_1ORB.html) finder is used.
  - `Akaze`, `Sift`, and `Surf` are also available, but are not fully implemented.
- Match Features
  - Find image pairs with matching features.  By default, [OpenCV's BestOf2NearestMatcher](https://docs.opencv.org/4.2.0/d4/d26/classcv_1_1detail_1_1BestOf2NearestMatcher.html) is used.
- Filter Images
  - Filter images with no matches or below a [match confidence threshold](https://github.com/airmap/image-processing/blob/912bca5ef286a6d04aa0e63b38dac67aa0cabede/stitcher/src/stitcher_configuration.cpp#L25).
- Estimate Camera Parameters
  - Perform initial motion estimation.  By default, [OpenCV's HomographyBasedEstimator](https://docs.opencv.org/4.2.0/db/d3e/classcv_1_1detail_1_1HomographyBasedEstimator.html) is used.
- Adjust Camera Parameters
  - Perform bundle adjustment to refine the camera parameters.  By default [OpenCV's BundleAdjusterRay](https://docs.opencv.org/4.2.0/da/d7c/classcv_1_1detail_1_1BundleAdjusterRay.html) is used.
- Warp Images
  - Warp images for the final stitch.  In the case of 360 panoramas, the images are projected onto the inside of a sphere, using [OpenCV's SphericalWarper](https://docs.opencv.org/4.2.0/d6/dd0/classcv_1_1detail_1_1SphericalWarper.html).
- Prepare Exposure Compensation
  - Calculate exposure compensation before finding seams.  This attempts to create consistent exposure for all images, despite variance at different angles.  By default, [OpenCV's BlocksGainCompensator](https://docs.opencv.org/4.2.0/d7/d81/classcv_1_1detail_1_1BlocksGainCompensator.html) is used.
- Find Seams
  - Find seams between images and create masks in preparation for the final composition.  By default, [OpenCV's GraphCutSeamFinder](https://docs.opencv.org/4.2.0/db/dda/classcv_1_1detail_1_1GraphCutSeamFinder.html) is used.
- Compose Panorama
  - Scale, warp, mask, and blend the images into the final panorama.  By default, [OpenCV's MultiBandBlender](https://docs.opencv.org/4.2.0/d5/d4b/classcv_1_1detail_1_1MultiBandBlender.html) is used.

# Potential Enhancements and Additions

## General
- Upgrade to latest OpenCV.
- Expose more configuration options as CLI parameters.

## Camera
- Recalibrate Vesper with still images instead of video frames.
- Create generic camera model (currently defaults to Parrot Anafi).

## Feature Matching
Use EXIF geolocation to optimize feature matching by skipping image pairs that are not adjacent.  The [FeaturesMatcher](https://github.com/airmap/image-processing/blob/main/stitcher/src/stitcher.cpp#L736) can accept an [optional mask](https://docs.opencv.org/4.2.0/da/d87/classcv_1_1detail_1_1FeaturesMatcher.html#a2df19558a646700d9543841cafae4bc2)

## Bundle Adjustment
Replace the OpenCV bundle adjuster with a [ceres-solver](https://github.com/ceres-solver/ceres-solver) or [openMVG](https://github.com/openMVG/openMVG) implementation.  The included bundle adjuster is relatively weak compared to state of the art and will often either fail to converge or run indefinitely.

## Additional Panorama Types
Currently, only 360 (spherical) panoramas are supported, but care has been taken to make sure that adding other types of panoramas will be a relatively straightforward task.

### Rectilinear
Images are captured with the camera maintaining orientation such that the images share a common plane, which is parallel to the sensor plane (e.g. [nadir](https://en.wikipedia.org/wiki/Nadir) images).

### Cylindrical
Images are captured with the camera rotating about the vertical axis of its center of perspective.  The resulting stitch is projected onto the inside of a cylinder and then unrolled to a flat image.

# Caveat Emptor
**This README, and much of the project, was written by a software engineer who is very interested in computer vision and photogrammetry, but is very far from an expert and has likely used poor terminology and made plenty of mistakes.**

# Credit
1. Scaramuzza, D., Martinelli, A. and Siegwart, R.. "A Flexible Technique for Accurate Omnidirectional Camera Calibration and Structure from Motion", Proceedings of IEEE International Conference of Vision Systems (ICVS'06), New York, January 5-7, 2006.  [PDF](http://www.google.com/url?q=http%3A%2F%2Frpg.ifi.uzh.ch%2Fdocs%2FICVS06_scaramuzza.pdf&sa=D&sntz=1&usg=AFQjCNEkDu6MS50jEsx5Goe6A24h7SWNLQ)
2. Scaramuzza, D., Martinelli, A. and Siegwart, R.. "A Toolbox for Easy Calibrating Omnidirectional Cameras", Proceedings to IEEE International Conference on Intelligent Robots and Systems (IROS 2006), Beijing China, October 7-15, 2006.  [PDF](http://www.google.com/url?q=http%3A%2F%2Frpg.ifi.uzh.ch%2Fdocs%2FIROS06_scaramuzza.pdf&sa=D&sntz=1&usg=AFQjCNE5uuggibI56NephnOcrX4kYFp8Jw)
3. Rufli, M., Scaramuzza, D., and Siegwart, R., Automatic Detection of Checkerboards on Blurred and Distorted Images,Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2008), Nice, France, September 2008.  [PDF](http://www.google.com/url?q=http%3A%2F%2Frpg.ifi.uzh.ch%2Fdocs%2FIROS08_scaramuzza_b.pdf&sa=D&sntz=1&usg=AFQjCNGJ8OLmN9O7WgpxMV-rnYdwvUpOkA)
