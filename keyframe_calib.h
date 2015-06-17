//
//  keyframe_calib.h
//  keyframeCalib
//
//  Created by jimmy on 6/15/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __keyframeCalib__keyframe_calib__
#define __keyframeCalib__keyframe_calib__

#include <vil/vil_image_view.h>
#include <vgl/vgl_point_2d.h>
#include <vcl_vector.h>
#include <vpgl/vpgl_perspective_camera.h>
#include <bapl/bapl_lowe_keypoint.h>
#include <bapl/bapl_bbf_tree.h>
#include <bapl/bapl_keypoint_sptr.h>
#include <bapl/bapl_keypoint_set.h>

class keyframeCalib
{
public:
    // sift feature
    static void getSIFT(const vil_image_view<vxl_byte> &image, vcl_vector<bapl_keypoint_sptr> & features);
    static bool writeSIFT(const char *file, const vcl_vector< bapl_keypoint_sptr > & keypoints);
    static bool readSIFT(const char *file, vcl_vector< bapl_keypoint_sptr > & keypoints);
    
    // calibration using key frames.
    // equation (10)
    static bool calibFromKeyframeCachedSIFT(const vil_image_view<vxl_byte> & topviewImage,
                                            const vpgl_perspective_camera<double> &keyframeCamera,
                                            vcl_vector<bapl_keypoint_sptr> * keyframeFeatures,
                                            const vil_image_view<vxl_byte> & queryImage,
                                            vcl_vector<bapl_keypoint_sptr> * queryImageFeatures,
                                            vpgl_perspective_camera<double> &queryCamera,
                                            int featureMatchingNumThreshold = 20);
    
};

#endif /* defined(__keyframeCalib__keyframe_calib__) */
