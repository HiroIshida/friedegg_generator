# note that edgetpu_object_detector's input is /kinect_head/rgb/image_rect_color
rosbag record /kinect_head/depth_registered/half/points \
    /kinect_head/rgb/image_rect_color \
    /edgetpu_object_detector/output/class \
    /edgetpu_object_detector/output/image \
    /edgetpu_object_detector/output/rects \
    /tf \
    -O ./pan_detect.bag
