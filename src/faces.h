
#ifndef _FACES_H_
#define _FACES_H_

#include <cv.h>
#include <ml.h>
#include <opencv2/core/cuda.hpp>

using namespace cv;
using namespace cuda;
std::vector<Rect> faceDetectAndDisplay(std::string windowName, Mat frame ) ;
#endif /* _FACES_H_ */
