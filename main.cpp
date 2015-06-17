//
//  main.cpp
//  keyframeCalib
//
//  Created by jimmy on 6/15/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include <vil/vil_image_view.h>
#include <vcl_vector.h>
#include <vil/vil_load.h>
#include <vil/vil_save.h>
#include <vcl_algorithm.h>
#include "util.h"
#include "keyframe_calib.h"

#include "basketballCourt.h"

int calibration_from_keyframes_example(const vcl_vector<vcl_string> & keyframeCameraFiles,
                                       const vcl_string & queryImageName,
                                       const vcl_string & topviewImageName)
{
    const int mini_focal_length = 1000; // focal length will not be very samll for particular camera, unit pixel
    
    assert(keyframeCameraFiles.size() > 0);
    
    // read key frame image and key frame camera
    const int sz = (int)keyframeCameraFiles.size();
    vcl_vector<vpgl_perspective_camera<double> > keyframeCameras(sz);
    vcl_vector< vcl_vector<bapl_keypoint_sptr> > keyframeFeaturesVec(sz);
    vcl_vector<vil_image_view<vxl_byte> > keyframeImages;
    for (int i = 0; i<keyframeCameraFiles.size(); i++) {
        vcl_string imageName;
        vpgl_perspective_camera<double> camera;
        bool isRead = Util::readCamera(keyframeCameraFiles[i].c_str(), imageName, camera);
        assert(isRead);
        keyframeCameras[i] = camera;
        printf("image name is %s, make sure there are in the right place of your computer.\n", imageName.c_str());
        
        vil_image_view<vxl_byte> image = vil_load(imageName.c_str());
        keyframeCalib::getSIFT(image, keyframeFeaturesVec[i]);
        keyframeImages.push_back(image);
    }
    
    assert(keyframeImages.size() > 0);
    
    // read query image which is to be calibrated
    // this the test image
    
    printf("test image name is %s, make sure there are in the right place of your computer.\n", queryImageName.c_str());
    vil_image_view<vxl_byte> queryImage = vil_load(queryImageName.c_str());
    vcl_vector<bapl_keypoint_sptr> querySIFT;
    keyframeCalib::getSIFT(queryImage, querySIFT);
    assert(queryImage.ni() == keyframeImages[0].ni());
    assert(queryImage.nj() == keyframeImages[0].nj());    
    
    vil_image_view<vxl_byte> topviewImage = vil_load(topviewImageName.c_str());
    vcl_vector<vpgl_perspective_camera<double> > candidateCameras;
    // loop all keyframes
    for (int j = 0; j<keyframeCameras.size(); j++){
        vpgl_perspective_camera<double> queryCamera;
        // this is main calibration function
        bool isCalib = keyframeCalib::calibFromKeyframeCachedSIFT(topviewImage, keyframeCameras[j], &keyframeFeaturesVec[j], queryImage, &querySIFT, queryCamera);
        if (isCalib && queryCamera.get_calibration().get_matrix()[0][0] >= mini_focal_length)
        {
            candidateCameras.push_back(queryCamera);
        }
    }
    if (candidateCameras.size() == 0) {
        printf("calibration failed.\n");
        return -1;
    }
    
    printf("topview image name is %s, make sure there are in the right place of your computer.\n", topviewImageName.c_str());
    //get the best camera
    BasketballCourt court;
    vcl_vector<double> dump;
    vcl_vector<double> SADs;
    court.alignmentQuality(topviewImage, queryImage, candidateCameras, dump, SADs);
    
    //save the one with minimum SAD error
    int sadIdx_min = (int)(vcl_min_element(SADs.begin(), SADs.end()) - SADs.begin());
    
    vpgl_perspective_camera<double> camera = candidateCameras[sadIdx_min];
    
    // overlay lines to the image
    vcl_vector<vxl_byte> colour;
    colour.push_back(0);
    colour.push_back(255);
    colour.push_back(0);
    court.overlayAllLines(camera, queryImage, colour);
    
    vil_save(queryImage, "overlay_court_line.jpg");
    
    printf("save to: overlay_court_line.jpg\n");
    
    return 0;
}


int main(int argc, const char * argv[])
{
    // example of camera file
    // start
    // image_name.jpg
    // ppx	 ppy	 focal length	 Rx	 Ry	 Rz	 Cx	 Cy	 Cz
    // 640.000000	 360.000000	 2023.539112	 1.718020	 -0.432401	 0.372700	 13.371400	 -14.490342	 6.067632
    // end
    //load key frame camera files. It contains image directory and camera parameters
    
    vcl_vector<vcl_string> keyframeCameraFiles;
    keyframeCameraFiles.push_back(vcl_string("keyframe/00000001.txt"));
    keyframeCameraFiles.push_back(vcl_string("keyframe/00000002.txt"));
    keyframeCameraFiles.push_back(vcl_string("keyframe/00000003.txt"));
    keyframeCameraFiles.push_back(vcl_string("keyframe/00000004.txt"));
    keyframeCameraFiles.push_back(vcl_string("keyframe/00000005.txt"));
    keyframeCameraFiles.push_back(vcl_string("keyframe/00000006.txt"));

    vcl_string queryImageName("test_image.jpg");
    vcl_string topviewImageName = vcl_string("topview.png");
    
    printf("you have to change the key frame camera, and topview image as yours first.");
    assert(0);
    
    calibration_from_keyframes_example(keyframeCameraFiles, queryImageName, topviewImageName);

   
    return 0;
}

