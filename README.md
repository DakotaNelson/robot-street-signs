# robot-street-signs
ROS Package which enables a Neato to recognize and obey simple street signs.

## System
Describe how your system works.  Make sure to include the basic components and algorithms that comprise your project.

## Design
One important design decision we made during the course of the project was the method by which we identified the signs: We used SIFT to detect keypoints in both the scene and template images, matched them, and then found a homography matrix to transform the scene image. We then matched the scene image with the correct sign by looking at the similarity between the transformed scene image and the template images. The reason we chose to actually transform the images with the homography matrices is that that transformation introduces a significant amount of error between poorly matched images. This allowed us to be much more confident in our sign choice, and to quantify our confidence, as opposed to one way we explored which would have simply compared the keypoints.

Another design decision we made was to use the ROS nav stack to control our robot when it approached the intersection, as opposed to manually sending it commands. While this took a significant amount of work to integrate properly, it was worthwhile because it allowed us to explore interesting and important ROS tools such as mapping, path-planning, and tf.

## Structure
How did you structure your code?

## Challenges
What if any challenges did you face along the way?

## Improvements
What would you do to improve your project if you had more time?

## Lessons
Did you learn any interesting lessons for future robotic programming projects? These could relate to working on robotics projects in teams, working on more open-ended (and longer term) problems, or any other relevant topic.
