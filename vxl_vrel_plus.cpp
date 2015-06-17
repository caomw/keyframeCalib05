//
//  vxl_vrel_plus.cpp
//  CameraPlaning
//
//  Created by Jimmy Chen LOCAL on 8/9/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#include <vcl_map.h>
#include "vxl_vrel_plus.h"


bool VrelPlus::homography_RANSAC(vcl_vector< vgl_point_2d< double > > const& first,
                                                      vcl_vector< vgl_point_2d< double > > const& second,
                                                      vcl_vector< bool > & inlier, vgl_h_matrix_2d< double > & H,
                                                      double error_threshold)
{
    // @ to be implemenated
    assert(first.size() >= 4);
    assert(first.size() == second.size());
    assert(inlier.size() == 0);
    assert(0);
    
    
        
   
    return true;
}






