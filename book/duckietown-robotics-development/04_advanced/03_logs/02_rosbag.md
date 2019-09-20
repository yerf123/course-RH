

## `rosbag` {#rosbag}

A bag is a file format in ROS for storing ROS message data. Bags, so named
because of their .bag extension, have an important role in ROS.
Bags are typically created by a tool like
[`rosbag`](http://wiki.ros.org/rosbag/Commandline), which subscribe to one or
more ROS topics, and store the serialized message data in a file as it is received.
These bag files can also be played back in ROS to the same topics they were
recorded from, or even remapped to new topics.

### `rosbag record`

The command
[`rosbag record`](http://wiki.ros.org/rosbag/Commandline#record)
records a bag file with the contents of specified topics.


### `rosbag info`

The command
[`rosbag info`](http://wiki.ros.org/rosbag/Commandline#info)
summarizes the contents of a bag file.


### `rosbag play`

The command
[`rosbag play`](http://wiki.ros.org/rosbag/Commandline#play)
plays back the contents of one or more bag files.


### `rosbag check`

The command
[`rosbag check`](http://wiki.ros.org/rosbag/Commandline#check)
determines whether a bag is playable in the current system, or if it can be migrated.

### `rosbag fix`

The command
[`rosbag fix`](http://wiki.ros.org/rosbag/Commandline#fix)
repairs the messages in a bag file so that it can be played in the current system.

### `rosbag filter`

The command
[`rosbag filter`](http://wiki.ros.org/rosbag/Commandline#filter)
converts a bag file using Python expressions.

### `rosbag compress`

The command
[`rosbag compress`](http://wiki.ros.org/rosbag/Commandline#compress)
compresses one or more bag files.

### `rosbag decompress`

The command
[`rosbag decompress`](http://wiki.ros.org/rosbag/Commandline#decompress)
decompresses one or more bag files.

### `rosbag reindex`

The command
[`rosbag reindex`](http://wiki.ros.org/rosbag/Commandline#reindex)
re-indexes one or more broken bag files.
