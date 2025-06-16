# Technical Reflection

This file summarizes our working with the project, the final implementation and how we could have
improved it having had more time.

## Bidirectional Communication

### What we achieved

Successfully established ROS 2-based communication between the physical TurtleBot3 and a virtual
TurtleBot3 in Gazebo. This allowed us to implement a python ROS2 package “smart_explorer” that
enables the robot to wander around an area autonomously.

Implemented a ROS2 node “yolo_detector” that receives image data from the real robot and processes
it for object classification using a YOLOv8 machine learning model. Classifying the detected
objects, into three types of recyclable trash or non-trash, and publishing this information to a
ROS2 topic “image_raw”. Based off this information implemented a ROS2 package “trash_map”, which
logs all trash detected and “picked up”, managing the storage limitations of the robot and
creating a trash distribution map. We also added that the digital part of the system sends a
specific command on how to handle the trash, allowing the robot to properly recycle.

Enabled the virtual robot to mirror the real robot’s movements and
sensor data in near real-time. Using a virtual environment that mirrored the real world
environments basic skeleton, we were able implement a filter ROS2 node “phyvir”, that detects any
discrepancies (in theory any discrepancy between the mirrored environment is potential trash)
between the environments. We used the lidar information provided by the real world robot and
virtual robot to calculate the location of these discrepancies and send the robot to investigate
further using the camera.

### What we did not achieve

Whereas we successfully implemented most of our functional requirements in their own
containerized environments, we ran into issues when combining them to work together. This
limits the amount of bidirectionality our limited proof of concept presents, though we
note that this does not reflect in the feasibility of our technical solution -- it is
solely a limitation of our implementation and an opening for future work and improvements.

### Bottlenecks or challenges

Throughout working on bidirectionality of our project the biggest challenge we ran into was
an issue synchronizing the physical and digital robots. The physical robot experienced minor
real-world wheel drifting and discrepancies of the real-world wheels that do not reflect in
the virtual environment, which runs an ideal version of the robot.

### What we could improve or expand

Implement a more robust communication protocol with buffering and error handling. This could
potentially fix issues we currently face with desynchronization and overall lag between the two
states. Integrate feedback loops to allow the virtual environment to influence real-world robot
behaviour more dynamically. This would allow the robot to be more efficient and autonomous. For
example this would include digital trash cans that mark areas that have been recently cleaned.

## State Synchronization

### What we achieved

Real-time synchronization of movement and sensor data (e.g., LiDAR) between the physical and
virtual robots. We remapped the LiDAR scan ROS2 scans, to manage conflict between the virtual
LiDAR and physical LiDAR. For this we also implemented an ROS2 filter node that detects
discrepancies between the two sensors, and prioritizes the physicals sensors data, and also
calculates the location of the discrepancies and publishes to a “discrepancy” ROS2 topic.

### What we did not achieve

In terms of state synchronization the primary thing we did not have time to achieve is displaying
objects that appear in the physical world in the virtual world. Although we initially intended
on implementing this, after careful consideration of our timeline we decided to focus on more
pressing aspects of the project, as this may be considered "cosmetic" in a way.

Additionally within our implementation there is a high likelihood that the physical robot gets
desynchronized from its virtual counterpart, resulting in issues with certain packages such as
our `phyvir` package for physical-virtual LiDAR filtering. This is something we initially did
not anticipate and did not plan for, though being a problem of implementing localization for the
Gazebo simulated robot we decided it was not worth the extensive amount of effort in the scope of
the proof of concept.

### Bottlenecks or challenges

Similar to the case of Bidirectional Communication we felt quite challenged by the physical and
virtual environments becoming desynchronized over time. Having had only a few sessions to work
with the physical environment, made it difficult to understand what the issue is and how we could
solve it. This not only made it difficult to work in the environment, but also resulted in
uncertainty of how to approach other modules. Ultimately we would have benefitted from
understanding the problem earlier.

### What we could improve or expand

Improve time-stamping and synchronization mechanisms to reduce drift between environments. This
would help us greatly in making sure the states stay synchronised.  In the future we could also
implement proper physics into the virtual world, so the friction and other external factors have
the same effect on the virtual as the physical. This would also greatly help making sure the
robots stay in sync.

## Object Interaction

### What we achieved

There are 2 main ways our system implements environmental and object interaction.

Firstly, we implemented a filter ROS2 node to detect discrepancies between the virtual worlds
LiDAR data and physical LiDAR data, and calculate this discrepancies location. This is useful
because any discrepancy between the virtual world and the physical is potential trash for the robot
to pickup. We then publish this information to a ROS2 topic called “discrepancy”. When this is
published, another ROS2 node we developed responsible for navigation, switches from wandering mode
to object mode, and the robot approaches the objects location and parks in front of it.

Secondly, utilizing this object detection feature,  we connected a camera to the robot and
developed an ROS2 node which publishes the image from the camera to an ROS2 topic called
“image_raw”. Based off this, we created the ROS2 package “yolo_detector”, that uses a YOLOv8
machine learning model to process the images, and publish information about them into two topics
such as “trash_num” and “trash_type”.

So in summary we use the navigation feature to find and approach object, and use the camera and
machine learning to classify these objects.

### What we did not achieve

Within the scope of our project we limited ourselves to what is feasible both in terms of time,
but also resources. Because of this we decided to limit our robot in terms of interaction with
the physical environment, mainly by the fact that it doesn't pick up trash, whereas our ideal
technical solution would. This would require a robotic arm and a few more weeks to get it working,
which we leave as future work.

### Bottlenecks or challenges

For object interaction we faced challenges regarding our physical-virtual filtering code not doing
what we expected it to do as we wrote it before the lab session. This was an exceptionally
difficult issue to work with unless we were on location physically. This proved to be
challenging as we ended up running into issues in lab sessions that we intended to use to test
other aspects of our project.

### What we could improve or expand

In the future there are 2 main things would like to make improvements on. 
Firstly the camera feed quality and the accuracy of our machine learning model. This would allow us to ensure that the robot works properly and does not make any mistakes.
Secondly implementing a robotic arm to actually be able to pick up trash. This would allow us to showcase the functionality of our robot better.
