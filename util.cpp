//
//  util.cpp
//  keyframeCalib
//
//  Created by jimmy on 6/15/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "util.h"

#include <vpgl/algo/vpgl_calibration_matrix_compute.h>
#include <vpgl/algo/vpgl_camera_compute.h>
#include <vnl/vnl_least_squares_function.h>
#include <vnl/algo/vnl_levenberg_marquardt.h>
//#include <vil/vil_interp.h>
#include <vil/vil_warp.h>
#include <vil/vil_bilin_interp.h>

bool Util::writeCamera(const char *fileName, const char *imageName, const vpgl_perspective_camera<double> & camera)
{
    assert(fileName);
    assert(imageName);
    
    FILE *pf = fopen(fileName, "w");
    if (!pf) {
        printf("can not create file %s\n", fileName);
        return false;
    }
    fprintf(pf, "%s\n", imageName);
    fprintf(pf, "ppx\t ppy\t focal length\t Rx\t Ry\t Rz\t Cx\t Cy\t Cz\n");
    double ppx = camera.get_calibration().principal_point().x();
    double ppy = camera.get_calibration().principal_point().y();
    double fl = camera.get_calibration().get_matrix()[0][0];
    double Rx = camera.get_rotation().as_rodrigues()[0];
    double Ry = camera.get_rotation().as_rodrigues()[1];
    double Rz = camera.get_rotation().as_rodrigues()[2];
    double Cx = camera.get_camera_center().x();
    double Cy = camera.get_camera_center().y();
    double Cz = camera.get_camera_center().z();
    fprintf(pf, "%f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\n", ppx, ppy, fl, Rx, Ry, Rz, Cx, Cy, Cz);
    fclose(pf);
    return true;
}

bool Util::readCamera(const char *fileName, vcl_string & imageName, vpgl_perspective_camera<double> & camera)
{
    assert(fileName);
    FILE *pf = fopen(fileName, "r");
    if (!pf) {
        printf("can not open file %s\n", fileName);
        return false;
    }
    char buf[1024] = {NULL};
    int num = fscanf(pf, "%s\n", buf);
    assert(num == 1);
    imageName = vcl_string(buf);
    for (int i = 0; i<1; i++) {
        char lineBuf[BUFSIZ] = {NULL};
        fgets(lineBuf, sizeof(lineBuf), pf);
        vcl_cout<<lineBuf;
    }
    double ppx, ppy, fl, rx, ry, rz, cx, cy, cz;
    int ret = fscanf(pf, "%lf %lf %lf %lf %lf %lf %lf %lf %lf", &ppx, &ppy, &fl, &rx, &ry, &rz, &cx, &cy, &cz);
    assert(ret == 9);
    
    vpgl_calibration_matrix<double> K(fl, vgl_point_2d<double>(ppx, ppy));
    vnl_vector_fixed<double, 3> rod(rx, ry, rz);
    vgl_rotation_3d<double> R(rod);
    vgl_point_3d<double> cc(cx, cy, cz);
    
    camera.set_calibration(K);
    camera.set_rotation(R);
    camera.set_camera_center(cc);
    
    return true;
}

void Util::overlay_line_segment(vil_image_view< vxl_byte >& image,
                                vgl_line_segment_2d< double > const& segment,
                                vcl_vector< vxl_byte > const& colour)
{
    assert(image.nplanes() == 3);
    assert(colour.size() == 3);
    
    const int w = image.ni();
    const int h = image.nj();
    
    vcl_vector<vgl_point_2d<double> > linePts;
    Util::draw_line(segment.point1(), segment.point2(), linePts);
    
    for (int i = 0; i<linePts.size(); i++) {
        if (Util::inside_image(linePts[i], w, h, 0)) {
            for (int j = 0; j<3; j++) {
                int x = linePts[i].x();
                int y = linePts[i].y();
                image(x, y, j) = colour[j];
            }
        }
    }
}

void Util::overlay_line_segment(vil_image_view< vxl_byte >& image,
                                vgl_line_segment_2d< double > const& segment,
                                vcl_vector< vxl_byte > const& colour,
                                int thickness)
{
    assert(image.nplanes() == colour.size());
    
    
    const int w = image.ni();
    const int h = image.nj();
    
    vcl_vector<vgl_point_2d<double> > linePts;
    Util::draw_line(segment.point1(), segment.point2(), thickness, linePts);
    
    for (int i = 0; i<linePts.size(); i++) {
        if (Util::inside_image(linePts[i], w, h, 0)) {
            for (int j = 0; j<image.nplanes(); j++) {
                int x = linePts[i].x();
                int y = linePts[i].y();
                image(x, y, j) = colour[j];
            }
        }
    }
}


bool Util::init_calib(const vcl_vector<vgl_point_2d<double> > &wldPts, const vcl_vector<vgl_point_2d<double> > &imgPts,
                          const vgl_point_2d<double> &principlePoint, vpgl_perspective_camera<double> &camera)
{
    if (wldPts.size() < 4 && imgPts.size() < 4) {
        return false;
    }
    if (wldPts.size() != imgPts.size()) {
        return false;
    }
    assert(wldPts.size() >= 4 && imgPts.size() >= 4);
    assert(wldPts.size() == imgPts.size());
    
    vpgl_calibration_matrix<double> K;
    if (vpgl_calibration_matrix_compute::natural(imgPts, wldPts, principlePoint, K) == false) {
        vcl_cerr<<"Failed to compute K"<<vcl_endl;
        vcl_cerr<<"Default principle point: "<<principlePoint<<vcl_endl;
        return false;
    }
    
    camera.set_calibration(K);
    
    // vpgl_perspective_camera_compute_positiveZ
    if (vpgl_perspective_camera_compute::compute(imgPts, wldPts, camera) == false) {
        vcl_cerr<<"Failed to computer R, C"<<vcl_endl;
        return false;
    }
    return true;
}

class optimize_perspective_camera_residual:public vnl_least_squares_function
{
protected:
    const vcl_vector<vgl_point_2d<double> > wldPts_;
    const vcl_vector<vgl_point_2d<double> > imgPts_;
    const vgl_point_2d<double> principlePoint_;
    
public:
    optimize_perspective_camera_residual(const vcl_vector<vgl_point_2d<double> > & wldPts, const vcl_vector<vgl_point_2d<double> > & imgPts,
                                         const vgl_point_2d<double> & pp):
    vnl_least_squares_function(7, (unsigned int)(wldPts.size()) * 2, no_gradient),
    wldPts_(wldPts),
    imgPts_(imgPts),
    principlePoint_(pp)
    {
        assert(wldPts.size() == imgPts.size());
        assert(wldPts.size() >= 5);
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        //focal length, Rxyz, Camera_center_xyz
        vpgl_calibration_matrix<double> K(x[0], principlePoint_);
        
        vnl_vector_fixed<double, 3> rod(x[1], x[2], x[3]);
        vgl_rotation_3d<double>  R(rod);
        vgl_point_3d<double> cc(x[4], x[5], x[6]);  //camera center
        
        vpgl_perspective_camera<double> camera;
        camera.set_calibration(K);
        camera.set_rotation(R);
        camera.set_camera_center(cc);
        
        //loop all points
        int idx = 0;
        for (int i = 0; i<wldPts_.size(); i++) {
            vgl_point_3d<double> p(wldPts_[i].x(), wldPts_[i].y(), 0);
            vgl_point_2d<double> proj_p = (vgl_point_2d<double>)camera.project(p);
            
            fx[idx] = imgPts_[i].x() - proj_p.x();
            idx++;
            fx[idx] = imgPts_[i].y() - proj_p.y();
            idx++;
        }
    }
    
    void getCamera(vnl_vector<double> const &x, vpgl_perspective_camera<double> &camera)
    {
        
        vpgl_calibration_matrix<double> K(x[0], principlePoint_);
        
        vnl_vector_fixed<double, 3> rod(x[1], x[2], x[3]);
        vgl_rotation_3d<double>  R(rod);
        vgl_point_3d<double> camera_center(x[4], x[5], x[6]);
        
        camera.set_calibration(K);
        camera.set_rotation(R);
        camera.set_camera_center(camera_center);
    }
    
};


bool Util::optimize_perspective_camera(const vcl_vector<vgl_point_2d<double> > & wldPts,
                                           const vcl_vector<vgl_point_2d<double> > & imgPts,
                                           const vpgl_perspective_camera<double> &initCamera,
                                           vpgl_perspective_camera<double> & finalCamera)
{
    assert(wldPts.size() == imgPts.size());
    assert(wldPts.size() >= 5);
    
    optimize_perspective_camera_residual residual(wldPts, imgPts, initCamera.get_calibration().principal_point());
    
    vnl_vector<double> x(7, 0);
    x[0] = initCamera.get_calibration().get_matrix()[0][0];
    x[1] = initCamera.get_rotation().as_rodrigues()[0];
    x[2] = initCamera.get_rotation().as_rodrigues()[1];
    x[3] = initCamera.get_rotation().as_rodrigues()[2];
    x[4] = initCamera.camera_center().x();
    x[5] = initCamera.camera_center().y();
    x[6] = initCamera.camera_center().z();
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinimied = lmq.minimize(x);
    if (!isMinimied) {
        vcl_cerr<<"Error: perspective camera optimize not converge.\n";
        lmq.diagnose_outcome();
        return false;
    }
    lmq.diagnose_outcome();
    
    //    lmq.diagnose_outcome();
    residual.getCamera(x, finalCamera);
    return true;
}

//inverse H mapping from court (world) to image space
class VxlPlusInvHwarp
{
public:
    VxlPlusInvHwarp(const vgl_h_matrix_2d<double> &invH)
    {
        invH_ = invH;
    }
    void operator()(const double ox, const double oy, double &ix, double &iy)
    {
        vgl_homg_point_2d<double> p = invH_((vgl_homg_point_2d<double>(ox, oy)));
        ix = p.x()/p.w();
        iy = p.y()/p.w();
    }
private:
    vgl_h_matrix_2d<double> invH_;
    
};

//bilinear interperation
static vxl_byte InterpolatorFunc( vil_image_view< vxl_byte > const& view, double x, double y, unsigned p )
{
	return vil_bilin_interp_safe( view, x, y, p);
}


bool Util::homography_warp_fill(const vil_image_view<vxl_byte>& srcImage,
                                   const vgl_h_matrix_2d<double> &H,
                                   const vil_image_view<vxl_byte> &destImage,
                                   vil_image_view<vxl_byte> &outImage)
{
    assert(srcImage.nplanes() == destImage.nplanes());
    assert(srcImage.nplanes() == outImage.nplanes());
    
    vgl_h_matrix_2d<double> invH = H.get_inverse();
    
    vil_warp(srcImage, outImage, VxlPlusInvHwarp(invH), InterpolatorFunc);
    
    for (int h = 0; h<outImage.nj(); h++) {
        for (int w =0; w<outImage.ni(); w++) {
            vgl_homg_point_2d<double> p = invH(vgl_homg_point_2d<double>(w, h, 1.0));
            double px = p.x()/p.w();
            double py = p.y()/p.w();
            
            //out of image
            if ((px < 0 || px >= srcImage.ni()) || (py < 0 || py >= srcImage.nj())) {
                for (int i = 0; i<destImage.nplanes(); i++) {
                    outImage(w, h, i) = destImage(w, h, i);
                }
            }
        }
    }
    
    return true;
}

bool Util::cameraToHomography(const vpgl_perspective_camera<double> &camera, vgl_h_matrix_2d<double> &H)
{
    vnl_matrix_fixed<double, 3, 4> P = camera.get_matrix();
    vnl_matrix_fixed<double, 3, 3> H_data;
    H_data(0, 0) = P(0, 0); H_data(0, 1) = P(0, 1); H_data(0, 2) = P(0, 3);
    H_data(1, 0) = P(1, 0); H_data(1, 1) = P(1, 1); H_data(1, 2) = P(1, 3);
    H_data(2, 0) = P(2, 0); H_data(2, 1) = P(2, 1); H_data(2, 2) = P(2, 3);
    H = vgl_h_matrix_2d<double>(H_data);
    return true;
}




// http://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
// Bresenham's line algorithm
bool Util::draw_line(const vgl_point_2d<double> & p0, const vgl_point_2d<double> & p1, vcl_vector<vgl_point_2d<double> > & linePts)
{    
    double x0 = p0.x();
    double y0 = p0.y();
    double x1 = p1.x();
    double y1 = p1.y();
    if (p0.x() > p1.x()) {
        vcl_swap(x0, x1);
        vcl_swap(y0, y1);
    }
    
    double deltaX = x1 - x0;
    double deltaY = y1 - y0;
    double error = 0;
    if (fabs(deltaX) < 0.5) {
        // vertical line
        if (y0 > y1) {
            vcl_swap(y0, y1);
        }
        int x = (x0 + x1)/2.0;
        for (int y = y0; y <= y1; y++) {
            linePts.push_back(vgl_point_2d<double>(x, y));
        }
    }
    else if(fabs(deltaY) < 0.5)
    {
        // horizontal line
        int y = (y0 + y1)/2.0;
        for (int x = x0; x <= x1; x++) {
            linePts.push_back(vgl_point_2d<double>(x, y));
        }
    }
    else
    {
        double deltaErr = fabs(deltaY/deltaX);
        int y = (int)y0;
        int sign = y1 > y0 ? 1:-1;
        for (int x = x0; x <= x1; x++) {
            linePts.push_back(vgl_point_2d<double>(x, y));
            error += deltaErr;
            while (error >= 0.5) {
                linePts.push_back(vgl_point_2d<double>(x, y));  // may have duplicated (x, y)
                y += sign;
                error -= 1.0;
            }
        }
    }
    return true;
}

// http://members.chello.at/~easyfilter/bresenham.html
bool Util::draw_line(const vgl_point_2d<double> & p0, const vgl_point_2d<double> & p1, double width, vcl_vector<vgl_point_2d<double> > & linePts)
{
    assert(width >= 0.5);
    
    int x0 = p0.x();
    int y0 = p0.y();
    int x1 = p1.x();
    int y1 = p1.y();
    float wd = (float)width;
    
    // plot an anti-aliased line of width wd
    int dx = abs(x1-x0), sx = x0 < x1 ? 1 : -1;
    int dy = abs(y1-y0), sy = y0 < y1 ? 1 : -1;
    int err = dx-dy;
    int e2 = 0;
    int x2 = 0;
    int y2 = 0;                           // error value e_xy
    float ed = dx+dy == 0 ? 1 : sqrt((float)dx*dx+(float)dy*dy);
    
    for (wd = (wd+1)/2; ; )
    {                                    // pixel loop
        linePts.push_back(vgl_point_2d<double>(x0, y0));
        e2 = err; x2 = x0;
        if (2*e2 >= -dx)
        {                                            // x step
            for (e2 += dy, y2 = y0; e2 < ed*wd && (y1 != y2 || dx > dy); e2 += dx)
            {
                //   setPixelColor(x0, y2 += sy, max(0,255*(abs(e2)/ed-wd+1)));
                linePts.push_back(vgl_point_2d<double>(x0, y2 += sy));
            }
            if (x0 == x1)
            {
                break;
            }
            e2 = err;
            err -= dy;
            x0 += sx;
        }
        if (2*e2 <= dy)
        {                                             // y step
            for (e2 = dx-e2; e2 < ed*wd && (x1 != x2 || dx < dy); e2 += dy)
            {
                linePts.push_back(vgl_point_2d<double>(x2 += sx, y0));
            }
            
            if (y0 == y1)
            {
                break;
            }
            err += dx;
            y0 += sy;
        }
    }
    return true;
}
