# robot-street-signs
ROS Package which enables a Neato to recognize and obey simple street signs.

## System
Describe how your system works.  Make sure to include the basic components and algorithms that comprise your project.

## Design
One important design decision we made during the course of the project was the method by which we identified the signs: We used SIFT to detect keypoints in both the scene and template images, matched them, and then found a homography matrix to transform the scene image. We then matched the scene image with the correct sign by looking at the similarity between the transformed scene image and the template images. The reason we chose to actually transform the images with the homography matrices is that that transformation introduces a significant amount of error between poorly matched images. This allowed us to be much more confident in our sign choice, and to quantify our confidence, as opposed to one way we explored which would have simply compared the keypoints.

Another design decision we made was to use the ROS nav stack to control our robot when it approached the intersection, as opposed to manually sending it commands. While this took a significant amount of work to integrate properly, it was worthwhile because it allowed us to explore interesting and important ROS tools such as mapping, path-planning, and tf.

## Structure
The structure of our code was largely object-oriented. Below is a class/interaction diagram that shows our code structure:

![alt text][code_structure]

[code_structure]: images/code_structure.png "This diagram shows an overview of the code our code structure. Each box is a separate class. The solid arrows represent a sub-class instantiation, and the dotted arrows represent communcation over ROS topics. The arrows to and from the top communicate with the Neato"

On the sign identification side, we have the main, StreetSignRecognizer class which receives image data. It bounds the sign and passes the bounded image to the TemplateMatcher, which does the sign identification and returns normalized confidences of each template sign to the recognizer. The recognizer cumulates these confidences until it is sure of one sign, at which point it publishes that sign the the /predicted_sign topic.

On the navigation side, we instantiate one node, StreetSignFollower, which publishes ties in with the ROS nav stack and publishes goals to /move_base_simple/goal. The goal that it publishes is determined by StateMachine, which outputs coordinates based off of the current state. Upon hearing a sign on the /predicted_sign topic, the StreetSignFollower passes it down to the StateMachine and, if a new goal is being published, tells the recognizer that it is sleeping on the /imSleeping topic -- so that the recognizer doesn't try to recognize signs while it is in the middle of turning.



## Challenges
There were a number of challenges that we faced in this project. One, that is in the computer vision space, involved getting our keypoint detection right. We found that when we tried to run keypoint detection on the entirety of the robot's vision, it often picked up more keypoints in the background of the scene than it did on the sign. This made it impossible to match any sign reliably with what the robot was seeing, and so we had to pre-process our image before we tried to identify it. To do this, we added bounding boxes based off of color, and used only the part of the image inside of the box for identification.

Another significant challenge we faced was the integration of the sign identification with the robot's navigation. We used the ROS nav stack and it's built in goal following, and found that our initial implementation of it, which published goals relative to the robot's position, caused a lot of problems. The robot would continually publish a goal ahead of it, so that if it were too close to the wall, it would publish a goal inside the wall and try to find a way inside the wall. Additionally, if we published the identified sign too early or too late, the robot would be in an non-ideal orientation and would subsequently publish it's new goal in a bad place. To solve this problem, we found a way to publish the goals directly onto the map frame, and used trigonometry to transform the goals relative to the robot's position at the time that it started navigating. 

## Improvements
If we were given more time, we would definitely seek to make the robot navigation more robust. Currently, the robot navigates a single T-intersection based off of preplanned points. This is fine for our purposes, because we used this project to delve more into computer vision than into robot navigation. Still, we would like to improve our navigation so that the robot could navigate between multiple intersections. Given our current code structure that uses the ROS nav stack, that would involve a much more complex state machine. Alternatively, we could explore ideas that would make the Neato behave more like a self-driving car, such as line-following for navigation.

Additionally, we would seek to add capabilities for more signs. This would include having more template images, as well as improving upon our bounding box algorithm, for signs that are not predominately yellow.

## Lessons
Did you learn any interesting lessons for future robotic programming projects? These could relate to working on robotics projects in teams, working on more open-ended (and longer term) problems, or any other relevant topic.
