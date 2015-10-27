# robot-street-signs
ROS Package which enables a Neato to recognize and obey simple street signs.

## System
Describe how your system works.  Make sure to include the basic components and algorithms that comprise your project.

## Design
Describe a design decision you had to make when working on your project and what you ultimately did (and why)? These design decisions could be particular choices for how you implemented some part of an algorithm or perhaps a decision regarding which of two external packages to use in your project.

## Structure
How did you structure your code?

## Challenges
What if any challenges did you face along the way?

## Improvements
If we were given more time, we would definitely seek to make the robot navigation more robust. Currently, the robot navigates a single T-intersection based off of preplanned points. This is fine for our purposes, because we used this project to delve more into computer vision than into robot navigation. Still, we would like to improve our navigation so that the robot could navigate between multiple intersections. Given our current code structure that uses the ROS nav stack, that would involve a much more complex state machine. Alternatively, we could explore ideas that would make the Neato behave more like a self-driving car, such as line-following for navigation.

Additionally, we would seek to add capabilities for more signs. This would include having more template images, as well as improving upon our bounding box algorithm, for signs that are not predominately yellow.

## Lessons
Did you learn any interesting lessons for future robotic programming projects? These could relate to working on robotics projects in teams, working on more open-ended (and longer term) problems, or any other relevant topic.
