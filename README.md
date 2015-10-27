# robot-street-signs
ROS Package which enables a Neato to recognize and obey simple street signs.

## System
Describe how your system works.  Make sure to include the basic components and algorithms that comprise your project.
 
![alt text][system-overview]

[system-overview]: images/vision-nav-system-overview.png "Three stages of the vision and navigation system: 1) waypoint navigation 2) sign recognition, and 3) sign obeyance via changing the next waypoint"

Navigation and mapping is handled by the built-in ROS package ```neato_2dnav``` .  Mapping the location of the robot in the environment was handled by [```gmapping```](http://wiki.ros.org/gmapping), a package that provides laser-based SLAM (simultaneous localization and mapping).  Navigation was handled by the [```move_base```](http://wiki.ros.org/move_base) package;   our program published waypoints to the ```/move_base_simple/goal``` topic while the internals of path planning and obstacle avoidance were abstracted away.

The significant chunk of the work was spent developing a sign detection node which published to a topic, ```/predicted_sign``` once it was confident about recognizing the sign in front of it.

#### Detecting Signs in Images
Reliable detection of traffic signs and creating accurate bounding box crops is important preprocessing for further parts of the pipeline. 

1. colorspace conversion to hue, saturation, value (```hsv_image``` seen in the top left window). 
2. a filter is applied that selects only for objects in the yellow color spectrum. The range of this spectrum was found using hand tuning (```binarized_image``` seen in the bottom window).
3. a bounding box is drawn around the most dense, yellow regions of the image.

![alt text][bbox-color]

[bbox-color]: images/yellow-sign-detector.gif "Bounding box generated around the yellow parts of the image.  The video is converted to HSV colorspace, an inRange operation is performed to filter out any non yellow objects, and finally a bounding box is generated."

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

