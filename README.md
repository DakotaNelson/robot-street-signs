# robot-street-signs
ROS Package which enables a Neato to recognize and obey simple street signs.

## System
Describe how your system works.  Make sure to include the basic components and algorithms that comprise your project.

## Design
Describe a design decision you had to make when working on your project and what you ultimately did (and why)? These design decisions could be particular choices for how you implemented some part of an algorithm or perhaps a decision regarding which of two external packages to use in your project.

## Structure
How did you structure your code?

## Challenges
There were a number of challenges that we faced in this project. One, that is in the computer vision space, involved getting our keypoint detection right. We found that when we tried to run keypoint detection on the entirety of the robot's vision, it often picked up more keypoints in the background of the scene than it did on the sign. This made it impossible to match any sign reliably with what the robot was seeing, and so we had to pre-process our image before we tried to identify it. To do this, we added bounding boxes based off of color, and used only the part of the image inside of the box for identification.

Another significant challenge we faced was the integration of the sign identification with the robot's navigation. We used the ROS nav stack and it's built in goal following, and found that our initial implementation of it, which published goals relative to the robot's position, caused a lot of problems. The robot would continually publish a goal ahead of it, so that if it were too close to the wall, it would publish a goal inside the wall and try to find a way inside the wall. Additionally, if we published the identified sign too early or too late, the robot would be in an non-ideal orientation and would subsequently publish it's new goal in a bad place. To solve this problem, we found a way to publish the goals directly onto the map frame, and used trigonometry to transform the goals relative to the robot's position at the time that it started navigating. 

## Improvements
What would you do to improve your project if you had more time?

## Lessons
Did you learn any interesting lessons for future robotic programming projects? These could relate to working on robotics projects in teams, working on more open-ended (and longer term) problems, or any other relevant topic.
