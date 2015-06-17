//
//  basketballCourt.h
//  version: sample code
//
//  Created by jimmy on 11/15/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __VpglPtzOpt__basketballCourt__
#define __VpglPtzOpt__basketballCourt__


#include <vxl_config.h>
#include <vcl_vector.h>
#include <vgl/vgl_line_segment_2d.h>
#include <vgl/vgl_line_segment_3d.h>
#include <vil/vil_image_view.h>
#include <vgl/vgl_line_segment_2d.h>
#include <vpgl/vpgl_perspective_camera.h>
#include "util.h"


class BasketballCourt
{
public:
    BasketballCourt();
    ~BasketballCourt();
    
    // weight image: pixels near court lines have higher weight
    void getWeightImage(const vpgl_perspective_camera<double> & camera, int width, int height, int lineWidth, double gauss_sigma, vil_image_view<double> &wt);
    
    // visualization purpose
    void overlayAllLines(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> &image, const vcl_vector<vxl_byte> & color);
    
    // project topview image by a camera
    bool projectTopviewImage(const vil_image_view<vxl_byte> &topview, const vpgl_perspective_camera<double> & camera, int width, int height,
                             vil_image_view<vxl_byte> & outImage, int threshold);
    
    // correspondences of: world points, overview image points and image points
    static void projectCourtPoints(const vpgl_perspective_camera<double> &camera, int width, int height,
                                   vcl_vector<vgl_point_2d<double> > & points_world,     //meter
                                   vcl_vector<vgl_point_2d<double> > & points_court_image, //pixel, overview image
                                   vcl_vector<vgl_point_2d<double>>  & points_camera_image, // pixel
                                   int threshold); // threshold away from the image boundary    
   
    
    // estimate alignment quality by SSD from topview to image
    // SSD: sum of squared difference
    // SAD: sum of absolute difference
    void alignmentQuality(const vil_image_view<vxl_byte> &topview, const vil_image_view<vxl_byte> &image,
                          const vcl_vector<vpgl_perspective_camera<double> > & cameras,
                          vcl_vector<double> & SSDs, vcl_vector<double> & SADs);
    
    // topview image (pixels) to world (meters)
    vgl_transform_2d< double > imageToWorld();
    static vcl_vector<vgl_point_2d<double> > getCalibPoints();     // points used in generate 3D/2D correspondence
private:
    static vcl_vector<vgl_point_2d<double> > getCourtPoints();    
    static vcl_vector< vgl_line_segment_2d< double > > getLineSegments();
    vcl_vector< vgl_line_segment_2d< double > > getAllLineSegments();
    vcl_vector< vgl_line_segment_2d< double > > getDivisionLine();
    
};



#endif /* defined(__VpglPtzOpt__basketballCourt__) */
