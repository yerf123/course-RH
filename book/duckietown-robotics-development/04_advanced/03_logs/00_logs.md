

# Working with logs {#ros-logs status=ready}

Robotics is innately married to hardware. However, when we develop and test our robots' software, it is often the case that we don't want to have to waste time to test on hardware after every small change. With bigger and more powerful robots, it might be the case that a softwre can result in a robot actuation that breaks it or even endanger human life! But if one can evaluate how a robot or a piece of code would behave before deploying on the actual platform then quite some headaches can be prevented. That is why working in simulation and from logs is so important in robotics. In this section you will learn how to work with logs in ROS.


<div class='requirements' markdown='1'>
  Requires: [Laptop setup](+opmanual_duckiebot#laptop-setup)
  
  Requires: [Docker poweruser skills](#docker-poweruser)
  
  Requires: [Developer knowledge of ROS](#dt-infrastructure) 
  
  Results: Reading and processing bag files
</div>

<minitoc/>


## Rosbag {#rosbag}


A bag is a file format in ROS for storing ROS message data. Bags, named so because of their `.bag` extension, have an important role in ROS. Bags are typically created by a tool like `rosbag`, which subscribes to one or more ROS topics, and stores the serialized message data in a file as it is received. These bag files can also be played back in ROS to the same topics they were recorded from, or even remapped to new topics.


Please go through [this](http://wiki.ros.org/rosbag/Commandline) link for more information.

## Rosbag: Recording {#rosbag-record}

You can use the following command to record bag files

    $ rosbag record TOPIC_1 TOPIC_2 TOPIC_3

or simply

    $ rosbag record -a

to record all messages being published.


## Rosbag Python API: Reading {#rosbag-read}

The following code snippet is a basic usage of the `rosbag` API to read bag files:

```python
import rosbag
bag = rosbag.Bag('test.bag')
for topic, msg, t in bag.read_messages(topics=['chatter', 'numbers']):
    print msg
bag.close()
```

## Rosbag Python API: Writing {#rosbag-write}

The following code snippet is a basic usage of the `rosbag` API to create bag files:

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


#### Record bag file {#exercise:rosbag-record}

Using the following concepts, 

- [Getting data in and out of your container](#docker-poweruser)
- [Communication between laptop and Duckiebot](#ros-multi-agent)

create a Docker container with a folder on your laptop mounted on the container. However, this time, instead of exporting the `ROS_MASTER_URI` and `ROS_IP` after entering the container, do it with the `docker run` command. You already know it from [here](#exercise:ex-docker-envvar).

Run the [lane following demo](+opmanual_duckiebot#demo-lane-following). Once your Duckiebot starts moving, record the camera images and the wheel commands from your duckiebot using `rosbag`. Navigate to the mounted folder using the `cd` command and then run (TODO: Check command)

    duckiebot-container $ rosbag record /![MY_ROBOT]/camera_node/image/compressed /![MY_ROBOT]/wheels_driver_node/wheels_cmd
  
Record the bag file for 30 seconds and then stop the recording using `Ctrl+C`. Then stop the demo as well.

</end>

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

Note: Make sure to mount the bag file to the Docker container, instead of copying it. 

Run the same analysis with the bag file you recorded in the previous exercise.

</end>

#### Processing bag files {#exercise:rosbag-process}

Using the following concepts, 

- [Getting data in and out of your container](#docker-poweruser)
- [Creating a basic Duckietown ROS enabled Docker image](#basic-structure)


create a Docker image which can process a bag file. For every image message in the bag file, extract the timestamp from the message, draw it on top of the image, and write it to a new bag file. Use the bag file you recorded for this exercise. The new bag file should be generated in the mounted folder.

</end>
