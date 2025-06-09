# Patches

A collection of git patch files for working with existing packages that we need to tweak
internally.

* `turtlebot3_gazebo-humble-square_world.patch` - Extends `turtlebot3_gazebo` by a square world,
similar to that of what we work with in lab sessions, in addition to a remapped `/scan` topic to
`/virtual_scan` for processing by phyvir.
