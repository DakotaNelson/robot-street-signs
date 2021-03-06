# robot-street-signs
ROS Package which enables a Neato to recognize and obey simple street signs.

**Video Demo:** [NEATO ROBOT OBEYS TRAFFIC SIGNS](https://youtu.be/poReVhj1lSA)

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
One important design decision we made during the course of the project was the method by which we identified the signs: We used SIFT to detect keypoints in both the scene and template images, matched them, and then found a homography matrix to transform the scene image. We then matched the scene image with the correct sign by looking at the similarity between the transformed scene image and the template images. The reason we chose to actually transform the images with the homography matrices is that that transformation introduces a significant amount of error between poorly matched images. This allowed us to be much more confident in our sign choice, and to quantify our confidence, as opposed to one way we explored which would have simply compared the keypoints.

Another design decision we made was to use the ROS nav stack to control our robot when it approached the intersection, as opposed to manually sending it commands. While this took a significant amount of work to integrate properly, it was worthwhile because it allowed us to explore interesting and important ROS tools such as mapping, path-planning, and tf.

## Structure
The structure of our code was largely object-oriented. Below is a class/interaction diagram that shows our code structure:

![alt text][code_structure]

[code_structure]: images/code_structure.png "This diagram shows an overview of the code our code structure. Each box is a separate class. The solid arrows represent a sub-class instantiation, and the dotted arrows represent communcation over ROS topics. The arrows to and from the top communicate with the Neato"

On the sign identification side, we have the main, ```StreetSignRecognizer``` class which receives image data. It bounds the sign and passes the bounded image to the ```TemplateMatcher```, which does the sign identification and returns normalized confidences of each template sign to the recognizer. The recognizer cumulates these confidences until it is sure of one sign, at which point it publishes that sign the the ```/predicted_sign``` topic.

On the navigation side, we instantiate one node, ```StreetSignFollower```, which publishes ties in with the ROS nav stack and publishes goals to ```/move_base_simple/goal```. The goal that it publishes is determined by ```StateMachine```, which outputs coordinates based off of the current state. Upon hearing a sign on the ```/predicted_sign``` topic, the ```StreetSignFollower``` passes it down to the ```StateMachine``` and, if a new goal is being published, tells the recognizer that it is sleeping on the ```/imSleeping``` topic -- so that the recognizer doesn't try to recognize signs while it is in the middle of turning.



## Challenges
There were a number of challenges that we faced in this project. One, that is in the computer vision space, involved getting our keypoint detection right. We found that when we tried to run keypoint detection on the entirety of the robot's vision, it often picked up more keypoints in the background of the scene than it did on the sign. This made it impossible to match any sign reliably with what the robot was seeing, and so we had to pre-process our image before we tried to identify it. To do this, we added bounding boxes based off of color, and used only the part of the image inside of the box for identification.

Another significant challenge we faced was the integration of the sign identification with the robot's navigation. We used the ROS nav stack and it's built in goal following, and found that our initial implementation of it, which published goals relative to the robot's position, caused a lot of problems. The robot would continually publish a goal ahead of it, so that if it were too close to the wall, it would publish a goal inside the wall and try to find a way inside the wall. Additionally, if we published the identified sign too early or too late, the robot would be in an non-ideal orientation and would subsequently publish it's new goal in a bad place. To solve this problem, we found a way to publish the goals directly onto the map frame, and used trigonometry to transform the goals relative to the robot's position at the time that it started navigating. 

## Improvements
If we were given more time, we would definitely seek to make the robot navigation more robust. Currently, the robot navigates a single T-intersection based off of preplanned points. This is fine for our purposes, because we used this project to delve more into computer vision than into robot navigation. Still, we would like to improve our navigation so that the robot could navigate between multiple intersections. Given our current code structure that uses the ROS nav stack, that would involve a much more complex state machine. Alternatively, we could explore ideas that would make the Neato behave more like a self-driving car, such as line-following for navigation.

Additionally, we would seek to add capabilities for more signs. This would include having more template images, as well as improving upon our bounding box algorithm, for signs that are not predominately yellow.

## Lessons

We learned quite a bit about robotics project that can be applied to teams in the future. One important lesson that we learned and applied during the course of our project was the power of using rosbags to develop our program, before we integrated it with the robots. This allowed us to work on our identification algorithm away from the robots, and we only brought our work back to the robots when we were wanted to integrate the entire system, after all the separate pieces were done

"Pair programming" as a team of three is difficult.  As it turned out, one of our team members was unavailable (out of the state) for the biggest checkpoints in the vision timeline.  Instead, we found that putting him on writing the navigation node that listens for traffic signs was a whole lot more productive.  If it's not neccessary for another programmer to join an unfamiliar code base, then don't make them join it.  Parellizing our time and efforts was effective in the end.

Start collecting real data early.  I originally thought we would have to build a traffic sign classifier out of labeled imagery from another dataset.  however, we managed without that, and instead relied exclusively on the ROS bag files of our own robot, driving up to the signs we made. Working with the images that were native to the task we would evaluate it on ensured that there was a direct mapping from our efforts to the final product.

Do not over engineer! Spelling out our MVP and specification helped us realize that using highly-complicated techniques like convolutional neural networks would have not provided us with any benefits towards the MVP of recognizing and obeying yellow street left, right, and u-turn signs. Going forward, we could frame the situation to have a larger variety of signs or objects; in this case, the powerful generalization of a convolutional network could realize benefits in the final application.

##Testimonials and Feedback
We reserve this section for our friends, family, and professors who had great things to say about this project.


###Paul Ruvolo's comments
Our professor for the Computational Robotics class provided the following feedback about code functionality, documentation, style as well as our writeup (this README)
 
> ###Functionality
> I just can't say enough good stuff about this project!  This project is definitely in the top handful of projects I've seen in the two times I've taught CompRobo (even including final projects from the first iteration).
 
> More than the end result, I was really impressed with the process that you engaged in to reach the end result.  It seems like every time I talked to you guys, you had tried more approaches, generated more visualizations, and gained more insight into the problem of sign detection.  What was clear is that through these various methods that your team tried you were consistently learning more about computer vision.  In short, this is exactly the type of exploration that I think makes project-based learning so compelling.
 
> I know that Dakota wasn't there for some of the project, but boy was I surprised when he was able to come in and get the SLAM + ```move_base``` working so well!  I saw you in the computer lab working on this for quite a while, so I know it was a lot of work.
 
> Again, this is just such a fantastic project.  I also like that it is well-documented so that I can show it off as an example in the future!
 
> ###Documentation
> Great use of docstrings + inline comments.  The small nit is that there is some variety between the formats / styles used in the docstrings and inline comments.  Standardizing this would make things even a little nicer.
 
> ###Style
 
> Nice object-oriented design.  I also like the architecture chosen for the state controller in the ```signFollower``` code.

> ###Writeup
 
> Very insightful writeup.  I definitely like the ideas around avoiding over-engineering.  It is often tempting to break out the heavy artillery (CNNs), but as you all stated it is unclear if that would have had any advantage over the system you developed (in fact I could see it actually being worse).
 
> I'm also glad that you took the idea of collecting data early to heart.  It is clear that this paid off in the long run as you were able to leverage this data to quickly validate or discard approaches depending on how they worked on data that was representative of the actual problem setting (rather than just a proxy such as some external database of signs).
 
> I like the suggested areas for follow up.  One thing I would love to know how to do is get the local planner in the ```move_base``` stack to more agressivel prune out local obstacles.  From watching the video I see the same problem that I ran into using move_base which is that you get all of these random little obstacle blobs that show up in tha map and obstruct the robot even though they don't correspond to actual obstacles.
