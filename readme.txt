It is the re-implementation calibration part of paper
@inproceedings{chen2015mimicking,
  title={Mimicking Human Camera Operators},
  author={Chen, Jianhui and Carr, Peter},
  booktitle={Applications of Computer Vision (WACV), 2015 IEEE Winter Conference on},
  pages={215--222},
  year={2015},
  organization={IEEE}
}

One thing is that: there is a typo in the original paper. The equation 10 should be: m_i_k = H * H_k * M_i. The inverse of H_k is not necessary.

It is designed to make similar calibration easier. It only provides the framework fo the algorithm described in the above paper.
It lacks following code/data. It is because they are the property of the organization I worked for.

1. an overview image. It is the image that warpping to the basketball court to the canoical view.
2. basektball court size. They can be get from: http://www.nba.com/media/dleague/1314-nba-rule-book.pdf
3. some basic functions:
   bool homography_RANSAC(vcl_vector< vgl_point_2d< double > > const& first,
                          vcl_vector< vgl_point_2d< double > > const& second,
                          vcl_vector< bool > & inlier,
                          vgl_h_matrix_2d< double > & H,
                          double error_threshold = 1.0);
  It estimate the homography matrix from points in "first" to "second" by RANSAC method. The "inlier" indicates which pairs are inlier or outlier. 
  The "error_threshold" is the threshold of re-projeciton error.
  
  In basketballCourt.cpp, these functions:

    vgl_transform_2d< double > imageToWorld();
    static vcl_vector<vgl_point_2d<double> > getCalibPoints();
    static vcl_vector<vgl_point_2d<double> > getCourtPoints();    
    static vcl_vector< vgl_line_segment_2d< double > > getLineSegments();
    vcl_vector< vgl_line_segment_2d< double > > getAllLineSegments();
    vcl_vector< vgl_line_segment_2d< double > > getDivisionLine();
 
 are not implemented on purpose as they contain basketball court data. 
 
 June 17, 2015.
 Please contact me (Jimmy) jhchen14@cs.ubc.ca if you have any questions.