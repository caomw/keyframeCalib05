//
//  util.h
//  keyframeCalib
//
//  Created by jimmy on 6/15/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __keyframeCalib__util__
#define __keyframeCalib__util__

#include <vcl_string.h>
#include <vpgl/vpgl_perspective_camera.h>
#include <vcl_vector.h>
#include <vgl/vgl_point_2d.h>
#include <vnl/vnl_inverse.h>
#include <vil/vil_image_view.h>

class Util
{
public:
    // write/read camera to .txt file
    static bool writeCamera(const char *fileName, const char *imageName, const vpgl_perspective_camera<double> & camera);
    static bool readCamera(const char *fileName, vcl_string & imageName, vpgl_perspective_camera<double> & camera);
    
    // p is inside image ?
    static inline bool inside_image(const vgl_point_2d<double> &p, int width, int height, int threshold)
    {
        // assert(threshold > 0);
        return p.x() >= (0 - threshold) && p.x() < (width + threshold) && p.y() >= (0 - threshold) && p.y() < (height + threshold);
    }
    
    // get pixel position in a line segment
    static void overlay_line_segment(vil_image_view< vxl_byte >& image,
                                     vgl_line_segment_2d< double > const& segment,
                                     vcl_vector< vxl_byte > const& colour);
    // get pixel position in a thick line segment
    static void overlay_line_segment(vil_image_view< vxl_byte >& image,
                                     vgl_line_segment_2d< double > const& segment,
                                     vcl_vector< vxl_byte > const& colour,
                                     int thickness);
    
    // calibration from 2D point correspondeces
    static bool init_calib(const vcl_vector<vgl_point_2d<double> > &wldPts, const vcl_vector<vgl_point_2d<double> > &imgPts,
                           const vgl_point_2d<double> &principlePoint, vpgl_perspective_camera<double> &camera);
    // optimize camera parameter by minimizing re-projection error
    static bool optimize_perspective_camera(const vcl_vector<vgl_point_2d<double> > & wldPts,
                                            const vcl_vector<vgl_point_2d<double> > & imgPts,
                                            const vpgl_perspective_camera<double> & initCamera,
                                            vpgl_perspective_camera<double> & finalCamera);
    // warp srcImage to destImage by homography mapping H
    static bool homography_warp_fill(const vil_image_view<vxl_byte>& srcImage,
                                           const vgl_h_matrix_2d<double> &H,
                                           const vil_image_view<vxl_byte> &destImage,
                                           vil_image_view<vxl_byte> &outImage);
    
    // extract homography mapping H from camera parameter
    static bool cameraToHomography(const vpgl_perspective_camera<double> &camera, vgl_h_matrix_2d<double> &H);
    
private:
    static bool draw_line(const vgl_point_2d<double> & p0, const vgl_point_2d<double> & p1, vcl_vector<vgl_point_2d<double> > & linePts);
    static bool draw_line(const vgl_point_2d<double> & p0, const vgl_point_2d<double> & p1, double width, vcl_vector<vgl_point_2d<double> > & linePts);
};

// 3x3 matrix describing 2D transformation
template< typename T >
class vgl_transform_2d: public vnl_matrix_fixed<T, 3, 3>
{
public:

    vgl_transform_2d()
    {
        this->set_identity();
    }
    
    vgl_transform_2d( vnl_matrix_fixed< T, 3, 3 > const& matrix ) :
    vnl_matrix_fixed< T, 3, 3 >( matrix )
    {
        
    }
    
    vgl_point_2d< T > operator()( vgl_point_2d< T > const& point ) const
    {
        vnl_vector_fixed<T, 3> temp(point.x(), point.y(), 1.0);
        temp = (*this) * temp;
        return vgl_point_2d<T>(temp[0]/temp[2], temp[1]/temp[2]);
    }
    
    ~vgl_transform_2d()
    {
    }
    
    vgl_transform_2d<T> inverse()
    {
        return vgl_transform_2d<T>(vnl_inverse(*this));
    }
    
    
};



#endif /* defined(__keyframeCalib__util__) */
