//
//  basketballCourt.cpp
//  VpglPtzOpt
//
//  Created by jimmy on 11/15/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#include "basketballCourt.h"
#include <vnl/vnl_math.h>
#include <vil/algo/vil_gauss_filter.h>
#include <vil/algo/vil_structuring_element.h>
#include <vil/algo/vil_binary_dilate.h>
#include <vil/vil_save.h>
#include <vil/vil_load.h>
#include <vnl/vnl_inverse.h>
#include <vil/vil_convert.h>
#include "util.h"


/*
    ----------------------------------------------- BasketballCourt ------------------------------------------------
 */
BasketballCourt::BasketballCourt()
{
   
}

BasketballCourt::~BasketballCourt()
{
    
   
    
}

void BasketballCourt::getWeightImage(const vpgl_perspective_camera<double> & camera, int width, int height, int lineWidth, double gauss_sigma, vil_image_view<double> &wt)
{
    vil_image_view<vxl_byte> wt_black_white = vil_image_view<vxl_byte>(width, height, 1);
    wt_black_white.fill(0);
    
    vcl_vector< vgl_line_segment_2d< double > > markings = getLineSegments();
    
    vcl_vector< vxl_byte > color_white;
    color_white.push_back(255);
    
    for ( unsigned int i = 0; i < markings.size(); ++i )
    {
        vgl_homg_point_3d< double > p1( markings[i].point1().x(), markings[i].point1().y(), 0, 1.0 );
        vgl_homg_point_3d< double > p2( markings[i].point2().x(), markings[i].point2().y(), 0, 1.0 );
        
        if (camera.is_behind_camera(p1) || camera.is_behind_camera(p2)) {
            continue;
        }
        
        vgl_point_2d< double > start = vgl_point_2d< double >(camera.project(p1));
        vgl_point_2d< double > stop = vgl_point_2d< double >( camera.project(p2));
    
        Util::overlay_line_segment(wt_black_white, vgl_line_segment_2d< double >( start, stop ), color_white, lineWidth);
    }
    
    wt = vil_image_view<double>(width, height, 1);
    vil_gauss_filter_5tap_params params(gauss_sigma);
    vil_gauss_filter_5tap(wt_black_white, wt, params);
    
    for (int y = 0; y<wt.nj(); y++) {
        for (int x = 0; x<wt.ni(); x++) {
            wt(x, y, 0) /= 255.0;
        }
    }
}


void BasketballCourt::overlayAllLines(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> &image, const vcl_vector<vxl_byte> & color)
{
    assert(image.nplanes() == 3);
    assert(color.size() == 3);
    
    vcl_vector< vgl_line_segment_2d< double > > markings = getAllLineSegments();
    
    for ( unsigned int i = 0; i < markings.size(); ++i )
    {
        vgl_homg_point_3d< double > p1( markings[i].point1().x(), markings[i].point1().y(), 0, 1.0 );
        vgl_homg_point_3d< double > p2( markings[i].point2().x(), markings[i].point2().y(), 0, 1.0 );
        
        if (camera.is_behind_camera(p1) || camera.is_behind_camera(p2)) {
            continue;
        }
        
        vgl_point_2d< double > start = vgl_point_2d< double >(camera.project(p1));
        vgl_point_2d< double > stop = vgl_point_2d< double >( camera.project(p2));
        
        
        Util::overlay_line_segment(image, vgl_line_segment_2d< double >( start, stop ), color);
    }
    
    vcl_vector< vgl_line_segment_2d< double > > divisionLines = getDivisionLine();
    for (int i = 0; i<divisionLines.size(); i++) {
        vgl_point_2d< double > start = vgl_point_2d< double >( camera.project (vgl_homg_point_3d< double >( divisionLines[i].point1().x(), divisionLines[i].point1().y(), 0 ) ) );
        vgl_point_2d< double > stop = vgl_point_2d< double >( camera.project( vgl_homg_point_3d< double >( divisionLines[i].point2().x(), divisionLines[i].point2().y(), 0 ) ) );
        
        Util::overlay_line_segment(image, vgl_line_segment_2d< double >( start, stop ), color);
    }
}



bool BasketballCourt::projectTopviewImage(const vil_image_view<vxl_byte> &topview, const vpgl_perspective_camera<double> & camera,
                                                     int width, int height, vil_image_view<vxl_byte> & outImage, int threshold)
{
    assert(topview.nplanes() == 3);
    
    //get H from court image to camera image
    vcl_vector<vgl_point_2d<double> > points_src;
    vcl_vector<vgl_point_2d<double> > points_dst;
    vcl_vector<vgl_point_2d<double> > points_world_dump;
    
    this->projectCourtPoints(camera, width, height, points_world_dump, points_src, points_dst, threshold);
    
    if (points_src.size() < 4) {
        return false;
    }
    assert(points_src.size() >= 4);
    assert(points_src.size() == points_dst.size());
    
    vcl_vector<vgl_homg_point_2d<double> > src;
    vcl_vector<vgl_homg_point_2d<double> > dst;
    for (int i = 0; i<points_src.size(); i++) {
        src.push_back(vgl_homg_point_2d<double>(points_src[i]));
        dst.push_back(vgl_homg_point_2d<double>(points_dst[i]));
    }
    
    
    vgl_h_matrix_2d<double> H(src, dst);
    
    outImage = vil_image_view<vxl_byte>(width, height, 3);
    outImage.fill(0);
    
    vil_image_view<vxl_byte> temp = vil_image_view<vxl_byte>(width, height, 3);
    temp.fill(0);
    
    Util::homography_warp_fill(topview, H, temp, outImage);
    return true;
}


void BasketballCourt::projectCourtPoints(const vpgl_perspective_camera<double> &camera, int width, int height,
                                                    vcl_vector<vgl_point_2d<double> > & points_world,
                                                    vcl_vector<vgl_point_2d<double> > & points_court_image,
                                                    vcl_vector<vgl_point_2d<double> > & points_camera_image,
                                                    int threshold)
{
   
    assert(points_world.size() == 0);
    assert(points_court_image.size() == 0);
    assert(points_camera_image.size() == 0);
    
    vcl_vector<vgl_point_2d<double> > courtPts = BasketballCourt::getCourtPoints();
    
    for (int i = 0; i<courtPts.size(); i++) {
        vgl_homg_point_3d<double> p(courtPts[i].x(), courtPts[i].y(), 0, 1.0);
        if (camera.is_behind_camera(p)) {
            continue;
        }

        vgl_point_2d<double> q = vgl_point_2d< double >(camera.project(p));
       
        if (Util::inside_image(q, width, height, threshold))
         {
             points_world.push_back(vgl_point_2d<double>(p.x(), p.y()));
             points_camera_image.push_back(q);
         }
    }
    
    BasketballCourt court;
    
    // from world to image coordinate
    vgl_transform_2d<double> imageToWorld = court.imageToWorld();
    vgl_transform_2d<double> worldToImage = imageToWorld.inverse();
    for (int i = 0; i<points_world.size(); i++) {
        vgl_point_2d<double> p = points_world[i];
        vgl_point_2d<double> q = worldToImage(p);
        points_court_image.push_back(q);
    }
    
    assert(points_world.size() == points_court_image.size());
    assert(points_world.size() == points_camera_image.size());
}

void BasketballCourt::alignmentQuality(const vil_image_view<vxl_byte> &topview, const vil_image_view<vxl_byte> &image,
                                                  const vcl_vector<vpgl_perspective_camera<double> > & cameras,
                                                  vcl_vector<double> & SSDs, vcl_vector<double> & SADs)
{
    assert(cameras.size() >= 1);
    assert(topview.nplanes() == 3);
    assert(image.nplanes() == 3);
    assert(SSDs.size() == 0);
    assert(SADs.size() == 0);
    
    int destWidth  = image.ni();
    int destHeight = image.nj();
    
    for (int i = 0; i<cameras.size(); i++) {
        vil_image_view<vxl_byte> warpedTopview;
        bool isWarpOk = this->projectTopviewImage(topview, cameras[i], destWidth, destHeight, warpedTopview, 20);
        if (!isWarpOk) {
            SSDs.push_back(INT_MAX);
            SADs.push_back(INT_MAX);
            continue;
        }
        vil_image_view<double> weightMap;
        this->getWeightImage(cameras[i], destWidth, destHeight, 4, 100, weightMap);
        
        double ssd = 0;
        double sad = 0;
        for (int y = 0; y<destHeight; y++) {
            for (int x = 0; x<destWidth; x++) {
                if (weightMap(x, y) != 0.0) {
                    double dif_r = abs(warpedTopview(x, y, 0) - image(x, y, 0));
                    double dif_g = abs(warpedTopview(x, y, 1) - image(x, y, 1));
                    double dif_b = abs(warpedTopview(x, y, 2) - image(x, y, 2));
                    sad += dif_r + dif_g + dif_b;
                    ssd += dif_r * dif_r + dif_g * dif_g + dif_b * dif_b;
                }
            }
        }
        SSDs.push_back(ssd);
        SADs.push_back(sad);
    }
}




vgl_transform_2d< double > BasketballCourt::imageToWorld()
{
    // @ to be implemenated
    // this is the transformation from overimage image (pixel) to world coordinate (meter)
    assert(0);
    vgl_transform_2d< double > model;
    return model;
}

vcl_vector<vgl_point_2d<double> > BasketballCourt::getCourtPoints()
{
    // @ to be implemenated
    assert(0);
    vcl_vector< vgl_point_2d<double> > markings; 
    
    return markings;
}

vcl_vector<vgl_point_2d<double> > BasketballCourt::getCalibPoints()
{
    // @ to be implemenated
    assert(0);
    vcl_vector<vgl_point_2d<double> > pts;
    return pts;
}


vcl_vector< vgl_line_segment_2d< double > > BasketballCourt::getLineSegments()
{
    // @ to be implemenated
    assert(0);
    vcl_vector< vgl_line_segment_2d< double > > markings;
    return markings;
}

vcl_vector< vgl_line_segment_2d< double > > BasketballCourt::getAllLineSegments()
{
    // @ to be implemenated
    assert(0);
    vcl_vector< vgl_line_segment_2d< double > > markings;
    
    return markings;
}


vcl_vector< vgl_line_segment_2d< double > > BasketballCourt::getDivisionLine()
{
    // @ to be implemenated
    assert(0);
    vcl_vector< vgl_line_segment_2d< double > > markings;    
    return markings;
}





















