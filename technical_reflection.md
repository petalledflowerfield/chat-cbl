# Technical Reflection

This file summarizes our working with the project, the final implementation and how we could have
improved it having had more time.

## Bidirectional Communication

### What we achieved

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

## State Synchronization

### What we achieved

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

## Object Interaction

### What we achieved

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
