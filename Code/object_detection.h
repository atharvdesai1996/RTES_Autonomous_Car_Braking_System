#ifndef _OBJECT_DETECTION_H

#define _OBJECT_DETECTION_H

#include <stdio.h>
#include <stdio.h>
#include "/usr/local/include/opencv2/highgui.hpp"
#include "/usr/local/include/opencv2/objdetect.hpp"
#include "/usr/local/include/opencv2/imgproc.hpp"
#include "/usr/local/include/opencv2/core/mat.hpp"
#include "/usr/local/include/opencv2/highgui.hpp"
#include "/usr/local/include/opencv2/imgcodecs.hpp"
#include "motor.h"
#include <semaphore.h>
#include <iostream>
#include <syslog.h>



int objdetect_action(Mat& img, CascadeClassifier& cascade, double scale);
void *objdetect_thread(void *args);


#endif
