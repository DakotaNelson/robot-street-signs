# robot-street-signs
ROS Package which enables a Neato to recognize and obey simple street signs.

## System
![alt text][system-overview]

[system-overview]: images/vision-nav-system-overview.png "Three stages of the vision and navigation system: 1) waypoint navigation 2) sign recognition, and 3) sign obeyance via changing the next waypoint"

Navigation and mapping is handled by the built-in ROS package ```neato_2dnav``` .  Mapping the location of the robot in the environment was handled by [```gmapping```](http://wiki.ros.org/gmapping), a package that provides laser-based SLAM (simultaneous localization and mapping).  Navigation was handled by the [```move_base```](http://wiki.ros.org/move_base) package;   our program published waypoints to the ```/move_base_simple/goal``` topic while the internals of path planning and obstacle avoidance were abstracted away.

The significant chunk of the work was spent developing a sign detection node which published to a topic, ```/predicted_sign``` once it was confident about recognizing the sign in front of it.

#### Detecting Signs in Scene Images
Reliable detection of traffic signs and creating accurate bounding box crops is important preprocessing for further parts of the pipeline. 

1. colorspace conversion to hue, saturation, value (```hsv_image``` seen in the top left window). 
2. a filter is applied that selects only for objects in the yellow color spectrum. The range of this spectrum was found using hand tuning (```binarized_image``` seen in the bottom window).
3. a bounding box is drawn around the most dense, yellow regions of the image.

![alt text][bbox-color]

[bbox-color]: images/yellow-sign-detector.gif "Bounding box generated around the yellow parts of the image.  The video is converted to HSV colorspace, an inRange operation is performed to filter out any non yellow objects, and finally a bounding box is generated."

#### Recognizing Left, Right, and U-Turn Signs

From the last stage, we might have a cropped image that looks something like this:

![alt text][bbox-crop]

[bbox-crop]: images/uturn_red_bbox.jpg "Now you have a bounding box crop..."

What steps are next to distinguish left, from right, from u-turn?

**Keypoints and Descriptors** are calculated using the [**SIFT** (Scale Invariant Feature Transform) algorithm](https://www.cs.ubc.ca/~lowe/papers/ijcv04.pdf).

We calculate SIFT descriptors for the images in our template set, as well as our cropped image.

![alt text][sign-keypoints]

[sign-keypoints]: images/sign_keypoints.png "SIFT keypoint descriptors for the template images and cropped scene image"

Then, a [**homography**](http://docs.opencv.org/3.0-beta/doc/py_tutorials/py_feature2d/py_feature_homography/py_feature_homography.html) is calculated between the each template sign and the cropped scene sign.

The homography matrix is a ```3 x 3``` matrix which describes the perspective transformation between the keypoints. When applied to the entire image, the keypoints and the sign will be warped to the same location in the destination image as close as can be.

![alt text][matching-warp]

[matching-warp]: images/uturn2uturn_visual_diff.png "An example of a succesful warp transformation, where the scene and template signs match. The figures from left to right: Warp, Destination, Normalized Visual Difference"


The trick is that **the perspective transform between mismatching images will yield a horribly disfigured image** (see left axes of the figure below)!  

When we take the visual difference between the warped scene and template sign image, the visual difference will tell us how good of a match a sign is to each of the images in our template database. 

The image above shows "U-turn" matching "U-turn"; the visual difference is relatively small: ```1470```, in units of grayscale pixel intensity.  The image below, on the other hand, shows "Left-turn" mismatching with the "U-turn".  The visual difference between the disfigured warp and the template is much larger: ```10597```, measured in units of grayscale pixel intensity.  Indeed, the smaller visual difference indicates the best sign prediction.

![alt text][mismatching-warp]

[mismatching-warp]: images/left2uturn_visual_diff.png "An example of a poor warp transformation, where the scene and template signs do not match. The figures from left to right: Warp, Destination, Normalized Visual Difference"

## Design
Describe a design decision you had to make when working on your project and what you ultimately did (and why)? These design decisions could be particular choices for how you implemented some part of an algorithm or perhaps a decision regarding which of two external packages to use in your project.

## Structure
How did you structure your code?

## Challenges
What if any challenges did you face along the way?

## Improvements
What would you do to improve your project if you had more time?

## Lessons
Did you learn any interesting lessons for future robotic programming projects? These could relate to working on robotics projects in teams, working on more open-ended (and longer term) problems, or any other relevant topic.

