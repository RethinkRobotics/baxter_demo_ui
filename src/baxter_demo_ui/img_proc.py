import cv
import cv_bridge
import PIL

'''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# PIL/cv interaction
# The functions in this secion handle interactions between
#   PIL and cvmat formats.
# **gen_msg() converts an RGB PIL image to a BGR cvmat image
# **rgb_to_bgr() converts an RGB PIL image to a BGR PIL image
# **PIL_to_cv() converts a BGR PIL image to a cvmat image
# **cv_to_msg() converts a cvmat image to a rosmsg format
# **msg_to_cv() converts a rosmsg image to a cvmat image
# **overlay() takes an original image, the size of that image,
#    a rectangle defined in that original image, and a new
#    new image and returns the original image with the new
#    image overlayed in the defined rectangle.
#    For the moment, this is mainly used for the Camera 
#       Display demo, overlaying the selected camera's image 
#       onto the UI display
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''

def gen_cv(img):
    img = rgb_to_bgr(img)
    return PIL_to_cv(img)

def rgb_to_bgr(img):
    r, g, b = img.split()
    return PIL.Image.merge('RGB', (b, g, r))

def PIL_to_cv(img):
    ci = cv.CreateImage((1024, 600), cv.IPL_DEPTH_8U, 3)
    cv.SetData(ci, img.tostring(), 3072)
    return ci

def cv_to_msg(img):
    return cv_bridge.CvBridge().cv_to_imgmsg(img, encoding='bgr8')

def msg_to_cv(img):
    return cv_bridge.CvBridge().imgmsg_to_cv(img, desired_encoding='bgr8')

def overlay(old_img, new_img, original_size, new_rect):
    tmp = cv.CreateImage(original_size, cv.IPL_DEPTH_8U, 3)
    cv.Copy(old_img, tmp)
    sub = cv.GetSubRect(tmp, new_rect)
    cv_img = msg_to_cv(new_img)
    cv.Copy(cv_img, sub)
    return cv_to_msg(tmp)
