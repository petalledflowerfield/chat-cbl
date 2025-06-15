# Technical Reflection

This file summarizes our working with the project, the final implementation and how we could have
improved it having had more time.

## What we achieved

## What we did not achieve

One primary aspect of our project that we did not manage to achieve was representing the various
dynamic objects in the physical world within the virtual world. Although it may have proven to be
quite simple, in the grand scheme of things we wanted to focus on other core functionalities such
as object detection and classification or autonomous navigation. If we had an extra week or two it
is likely that we would get to fully implement this aspect as we already put in a bit of effort
to create some models for this, though we decided it would be more worth our time to focus on the
rest.

An additional part that we would have liked to achieve, although one of our restrictions, is
having a robotic arm on the robot to physically pick up trash that is in front of it. This forms
a big part of "future work" for our project, as we did not have the resources nor time to approach
it. If we were to be given more time and the resources to do so, it would have been a fun challenge
to implement a more physical aspect to our robot.

However, overall the biggest thing we failed to achieve within our project was properly chaining
all functionality together. On the one hand we managed to write code that in theory ties the
procedures together, but on the other hand executing and testing this in the lab session revealed
that we were ill-prepared and needed to reflect further on our approach to the problem. For the
most part executing our packages in the virtual environment forms a coherent system, however
the introduction of a physical aspect showed holes in our implementation, such as a lack of
localization of the simulated robot in the Gazebo environment, resulting in desynchronization
between the physical and digital robots. This triggered a domino affect causing the phyvir package
to have trouble processing differences between the two environments.

## Bottlenecks or challenges

## What we could improve or expand
