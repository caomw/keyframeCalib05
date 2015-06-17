//
//  vxl_vrel_plus.h
//  version: sample code
//
//  Created by Jimmy Chen LOCAL on 8/9/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __CameraPlaning__vxl_vrel_plus__
#define __CameraPlaning__vxl_vrel_plus__

#include <vgl/algo/vgl_h_matrix_2d.h>
#include <vcl_vector.h>
#include <vgl/vgl_point_2d.h>
#include <vil/vil_image_view.h>


class VrelPlus
{
public:
   
    static bool homography_RANSAC(vcl_vector< vgl_point_2d< double > > const& first,
                                  vcl_vector< vgl_point_2d< double > > const& second,
                                  vcl_vector< bool > & inlier, vgl_h_matrix_2d< double > & H,
                                  double error_threshold = 1.0);
};



#endif /* defined(__CameraPlaning__vxl_vrel_plus__) */
