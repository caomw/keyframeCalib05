//
//  keyframe_calib.cpp
//  keyframeCalib
//
//  Created by jimmy on 6/15/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "keyframe_calib.h"
#include <bapl/bapl_keypoint_extractor.h>
#include <bapl/bapl_keypoint_sptr.h>
#include <bapl/bapl_dense_sift_sptr.h>
#include <bapl/bapl_lowe_keypoint_sptr.h>
#include <vil/vil_convert.h>
#include <vil/vil_new.h>
#include <vnl/vnl_inverse.h>
#include <vil/vil_load.h>

#include "basketballCourt.h"
#include "vxl_vrel_plus.h"
#include "util.h"

void keyframeCalib::getSIFT(const vil_image_view<vxl_byte> &image, vcl_vector<bapl_keypoint_sptr> & features)
{
    assert(image.nplanes() == 1 || image.nplanes() == 3);
    assert(features.size() == 0);
    
    vil_image_view<vxl_byte> grey_img;
    if (image.nplanes() == 3) {
        vil_convert_planes_to_grey(image, grey_img);
    }
    else
    {
        grey_img.deep_copy(image);
    }
    
    bapl_keypoint_extractor(vil_new_image_resource_of_view(grey_img), features);
}

bool keyframeCalib::writeSIFT(const char *file, const vcl_vector< bapl_keypoint_sptr > & keypoints)
{
    FILE *pf = fopen(file, "w");
    if (!pf) {
        printf("Error: canot open %s\n", file);
        return false;
    }
    
    int n = (int)keypoints.size();
    int len = 128;
    fprintf(pf, "%d\t %d\n", n, len);
    for (int i = 0; i<keypoints.size(); i++) {
        bapl_lowe_keypoint_sptr sift = dynamic_cast<bapl_lowe_keypoint*>(keypoints[i].as_pointer());
        double loc_x = sift->location_i();
        double loc_y = sift->location_j();
        double scale = sift->scale();
        double orientation = sift->orientation();
        fprintf(pf, "%lf\t %lf\t %lf\t %lf\n", loc_x, loc_y, scale, orientation);
        
        vnl_vector_fixed<double,128> feat = sift->descriptor();
        for (int j = 0; j<feat.size(); j++) {
            fprintf(pf, "%lf  ", feat[j]);
        }
        fprintf(pf, "\n");
    }
    fclose(pf);
    
    return true;
}

bool keyframeCalib::readSIFT(const char *file, vcl_vector< bapl_keypoint_sptr > & keypoints)
{
    FILE *pf = fopen(file, "r");
    if (!pf) {
        printf("Error: canot open %s\n", file);
        return false;
    }
    int n = 0;
    int len = 0;
    int ret = fscanf(pf, "%d %d ", &n, &len);
    assert(ret == 2);
    assert(len == 128);
    
    for (int i = 0; i < n; i++) {
        double loc_x = 0;
        double loc_y = 0;
        double scale = 0;
        double orientation = 0;
        ret = fscanf(pf, "%lf %lf %lf %lf ", &loc_x, &loc_y, &scale, &orientation);
        assert(ret == 4);
        
        vnl_vector_fixed<double, 128> desc;
        for (int j = 0; j < len; j++) {
            double val = 0;
            ret = fscanf(pf, "%lf", &val);
            desc[j] = val;
        }
        bapl_lowe_pyramid_set_sptr py;
        bapl_lowe_keypoint_sptr kp = new bapl_lowe_keypoint(py, loc_x, loc_y, scale, orientation, desc);
        keypoints.push_back(kp);
    }
    fclose(pf);
    printf("find %d keypoints.\n", n);
    return true;
}



bool keyframeCalib::calibFromKeyframeCachedSIFT(const vil_image_view<vxl_byte> & topviewImage,
                                                const vpgl_perspective_camera<double> &keyframeCamera,
                                                vcl_vector<bapl_keypoint_sptr> * keyframeFeatures,
                                                const vil_image_view<vxl_byte> & queryImage,
                                                vcl_vector<bapl_keypoint_sptr> * queryImageFeatures,
                                                vpgl_perspective_camera<double> &queryCamera,
                                                int featureMatchingNumThreshold)
{
   
    assert(queryImage.nplanes()  == 3);   
    assert(keyframeFeatures);
    assert(queryImageFeatures);
    
    int destWidth  = queryImage.ni();
    int destHeight = queryImage.nj();
    
    vcl_vector<bapl_keypoint_sptr> keyframeSift   = *keyframeFeatures;
    vcl_vector<bapl_keypoint_sptr> queryImageSift = *queryImageFeatures;
    
    
    bapl_bbf_tree tree(queryImageSift, 16);
    
    // match from keyframe to image
    // use two threshold 0.6 and 0.7 to avoid too few matching situation
    // if 0.6 get enough matches, 0.7 and later will be discarded
    vcl_vector<double> ratios;
    ratios.push_back(0.6);
    ratios.push_back(0.7);
    
    vcl_vector< vcl_vector<bapl_key_match> > matchesVec(2);
    for (unsigned int i = 0; i<keyframeSift.size(); i++) {
        bapl_keypoint_sptr      query = keyframeSift[i];
        vcl_vector<bapl_keypoint_sptr> match;
        
        tree.n_nearest(query, match, 2, -1);
        double ssd0 = vnl_vector_ssd(query->descriptor(), match[0]->descriptor());
        double ssd1 = vnl_vector_ssd(query->descriptor(), match[1]->descriptor());
        bapl_key_match k_p(query, match[0]);
        for (int j = 0; j<ratios.size(); j++) {
            if (ssd0 < ssd1 * ratios[j]) {
                matchesVec[j].push_back(k_p);
            }
        }
    }
    
    // decide which threshold 0.6 or 0.7
    // 100 is an experimental number for robust estimation
    vcl_vector<bapl_key_match> matches;
    if (matchesVec[0].size() >= 100) {
        matches = matchesVec[0];
    }
    else
    {
        matches = matchesVec[1];
    }
    if (matches.size() <= featureMatchingNumThreshold) {
        vcl_cerr<<"match number too few "<<matches.size()<<vcl_endl;
        return false;
    }
    
    vcl_vector<vgl_point_2d<double> > pts_keyframe;
    vcl_vector<vgl_point_2d<double> > pts_image;
    for (int i = 0; i<matches.size(); i++) {
        bapl_lowe_keypoint_sptr sift1 = dynamic_cast<bapl_lowe_keypoint*>(matches[i].first.as_pointer());
        bapl_lowe_keypoint_sptr sift2 = dynamic_cast<bapl_lowe_keypoint*>(matches[i].second.as_pointer());
        vgl_point_2d<double> p1(sift1->location_i(), sift1->location_j());
        vgl_point_2d<double> p2(sift2->location_i(), sift2->location_j());
        
        pts_keyframe.push_back(p1);
        pts_image.push_back(p2);
    }
    assert(pts_keyframe.size() == pts_image.size());
    vcl_cout<<"initial matched number is "<<pts_keyframe.size()<<vcl_endl;
    
    BasketballCourt court;
    //ransac 10 times, to get the best one with minimal projection error
    vgl_h_matrix_2d<double> Hk;
    bool isOk = Util::cameraToHomography(keyframeCamera, Hk);
    assert(isOk);
    
    vcl_vector<vpgl_perspective_camera<double> > optimizedCameras;
    for (int i = 0; i<10; i++) {
        vcl_vector<bool> inlier;  //inlier not used here, only H_keyframeToQuery is used
        vgl_h_matrix_2d<double> H;
        bool isOk = VrelPlus::homography_RANSAC(pts_keyframe, pts_image, inlier, H, 2.0);    //Result vary from time to time
        if (!isOk) {
            continue;
        }
        assert(inlier.size() == pts_keyframe.size());
        
        vgl_h_matrix_2d<double> HHk = H * Hk;  // equation 10
        
        vcl_vector<vgl_point_2d<double> > courtPoints = BasketballCourt::getCalibPoints();
        vcl_vector<vgl_point_2d<double> > sampledCourtPoints_world;
        vcl_vector<vgl_point_2d<double> > sampledCourtPoints;   // query image sapce
        for (int  j = 0; j<courtPoints.size(); j++) {
            vgl_point_2d<double> p_world   = courtPoints[j];
            vgl_point_2d<double> proj_p = (vgl_point_2d<double>)HHk(vgl_homg_point_2d<double>(p_world));
            
            if(Util::inside_image(proj_p, destWidth, destHeight, 20))
            {
                sampledCourtPoints_world.push_back(p_world);  // world coordinate in meter
                sampledCourtPoints.push_back(proj_p);         // pixel position in query image
            }
        }       
        
        assert(sampledCourtPoints.size() == sampledCourtPoints_world.size());
        if (sampledCourtPoints.size() < 4) {
            continue;
        }
        
        vgl_point_2d<double> principlePoint(destWidth/2, destHeight/2);
        vpgl_perspective_camera<double> initCamera;
        bool isCalib = Util::init_calib(sampledCourtPoints_world, sampledCourtPoints, principlePoint, initCamera);
        
        if (!isCalib) {
            vcl_cerr<<"init calib failed."<<vcl_endl;
            continue;
        }
        
        if (sampledCourtPoints_world.size() < 5) {
            vcl_cerr<<"have less than 5 correspondences."<<vcl_endl;
            continue;
        }
        
        vpgl_perspective_camera<double> opt_camera;
        bool isOpted = Util::optimize_perspective_camera(sampledCourtPoints_world, sampledCourtPoints, initCamera, opt_camera);
        if (!isOpted) {
            vcl_cerr<<"optimize camera failed."<<vcl_endl;
            continue;
        }
        
        vcl_cout<<"optimized focal length is "<<opt_camera.get_calibration().focal_length()<<vcl_endl;
        optimizedCameras.push_back(opt_camera);
    }
    
    if (optimizedCameras.size() == 0) {
        vcl_cerr<<"No camera found under projection error threshold.\n";
        return false;
    }
    
    printf("found %lu camera candidates.\n", optimizedCameras.size());
    
    vcl_vector<double> ssds;
    vcl_vector<double> sads;
    court.alignmentQuality(topviewImage, queryImage, optimizedCameras, ssds, sads);
    int idx_min = (int)(vcl_min_element(sads.begin(), sads.end()) - sads.begin());
    queryCamera = optimizedCameras[idx_min];
    return true;
}

