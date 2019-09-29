

# Working with logs {#ros-logs status=ready}

In this section you will work with logs in ROS


<div class='requirements' markdown='1'>
  Requires: [Laptop setup](+opmanual_duckiebot#laptop-setup)
  Requires: [Docker poweruser skills](#docker-poweruser)
  Requires: [Developer knowledge of ROS](#dt-infrastructure) 
  Results: Reading and processing bag files
</div>

<minitoc/>


## Rosbag {#rosbag}

A bag is a file format in ROS for storing ROS message data. Named this way because of their .bag extension, they have an important role in ROS. Bags are typically created by a tool like `rosbag`, which subscribe to one or more ROS topics, and store the serialized message data in a file as it is received. These bag files can also be played back in ROS to the same topics they were recorded from, or even remapped to new topics. This can be very useful to accuratly measure the improvement of a certain algorithm: record some data, and play it to your successive iterations of the algorithm. Of course, be careful not to overfit your algorithm to this specific batch of data.

Please go through [this](http://wiki.ros.org/rosbag/Commandline) link for more information.

## Rosbag: Recording {#rosbag-record}

You can use the following command to record bag files
```bash
rosbag record TOPIC_1 TOPIC_2 TOPIC_3
```
or simply
```bash
rosbag record -a
```
to record all messages being published.


## Rosbag API: Reading {#rosbag-read}

The following code snippet is a basic usage of the `rosbag` API to read bag files

```python
import rosbag
bag = rosbag.Bag('test.bag')
for topic, msg, t in bag.read_messages(topics=['chatter', 'numbers']):
    print msg
bag.close()
```

## Rosbag API: Writing {#rosbag-write}

The following code snippet is a basic usage of the `rosbag` API to create bag files

```python
import rosbag
from std_msgs.msg import Int32, String

bag = rosbag.Bag('test.bag', 'w')

try:
    s = String()
    s.data = 'foo'

    i = Int32()
    i.data = 42

    bag.write('chatter', s)
    bag.write('numbers', i)
finally:
    bag.close()
```

## Exercises {#rosbag-exercise}

#### Analyze bag files {#exercise:rosbag-stats}

Download [this](https://www.dropbox.com/s/11t9p8efzjy1az9/example_rosbag_H3.bag?dl=1) bag file. Using the following concepts,

- [Getting data in and out of your container](#docker-poweruser)
- [Creating a basic Duckietown ROS enabled Docker image](#basic-structure)

create a Docker image which can analyze bag files and produce the following output. The min, max, average, and median values printed are statistics of the time difference between two consecutive messages. 

```
/tesla/camera_node/camera_info:
  num_messages: 653
  period:
    min: 0.01
    max: 0.05
    average: 0.03
    median: 0.03

/tesla/line_detector_node/segment_list:
  num_messages: 198
  period:
    min: 0.08
    max: 0.17
    average: 0.11
    median: 0.1

/tesla/wheels_driver_node/wheels_cmd:
  num_messages: 74
  period:
    min: 0.02
    max: 4.16
    average: 0.26
    median: 0.11
``` 
Make sure to mount the bag file to the Docker image, not copy. 
