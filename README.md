# robot-street-signs
ROS Package which enables a Neato to recognize and obey simple street signs.

## System
Describe how your system works.  Make sure to include the basic components and algorithms that comprise your project.

## Design
Describe a design decision you had to make when working on your project and what you ultimately did (and why)? These design decisions could be particular choices for how you implemented some part of an algorithm or perhaps a decision regarding which of two external packages to use in your project.

## Structure
The structure of our code was largely object-oriented. Below is a class/interaction diagram that shows our code structure:

![alt text][code_structure]

[code_structure]: images/code_structure.png "This diagram shows an overview of the code our code structure. Each box is a separate class. The solid arrows represent a sub-class instantiation, and the dotted arrows represent communcation over ROS topics. The arrows to and from the top communicate with the Neato"

On the sign identification side, we have the main, StreetSignRecognizer class which receives image data. It bounds the sign and passes the bounded image to the TemplateMatcher, which does the sign identification and returns normalized confidences of each template sign to the recognizer. The recognizer cumulates these confidences until it is sure of one sign, at which point it publishes that sign the the /predicted_sign topic.

On the navigation side, we instantiate one node, StreetSignFollower, which publishes ties in with the ROS nav stack and publishes goals to /move_base_simple/goal. The goal that it publishes is determined by StateMachine, which outputs coordinates based off of the current state. Upon hearing a sign on the /predicted_sign topic, the StreetSignFollower passes it down to the StateMachine and, if a new goal is being published, tells the recognizer that it is sleeping on the /imSleeping topic -- so that the recognizer doesn't try to recognize signs while it is in the middle of turning.



## Challenges
What if any challenges did you face along the way?

## Improvements
What would you do to improve your project if you had more time?

## Lessons
Did you learn any interesting lessons for future robotic programming projects? These could relate to working on robotics projects in teams, working on more open-ended (and longer term) problems, or any other relevant topic.
