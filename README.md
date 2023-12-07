# libcaer_driver

A ROS2 driver for event based cameras using Inilab's Libcaer.
This driver is not written or supported by Inilabs.

![banner image](images/davis_240C.png)

In contrast to the [well established ROS1 driver from the University of Zuerich](https://github.com/uzh-rpg/rpg_dvs_ros), this driver publishes messages using the more efficient [event_camera_msgs](https://github.com/ros-event-camera/event_camera_msgs/) format.

The events can be decoded and displayed using the following ROS/ROS2 packages:

- [event_camera_codecs](https://github.com/ros-event-camera/event_camera_codecs)
  has C++ routines to decode event_camera_msgs.
- [event_camera_py](https://github.com/ros-event-camera/event_camera_py)
  module for fast event decoding in python.
- [event_camera_renderer](https://github.com/ros-event-camera/event_camera_renderer)
  a node / nodelet that renders and publishes ROS image messages.
- [event_camera_tools](https://github.com/ros-event-camera/event_camera_tools)
  a set of tools to echo, monitor performance and convert
  [event_camera_msgs](https://github.com/ros-event-camera/event_camera_msgs) to legacy formats and into "RAW" format.

## Supported platforms

Tested on the following platforms:

- ROS2 Humble on Ubuntu 22.04 LTS

Tested on the following hardware:

- [Davis 240C](https://inivation.com/wp-content/uploads/2019/08/DAVIS240.pdf)
- [DVXplorer](https://shop.inivation.com/collections/dvxplorer)

## How to build

Prerequisites:

Install ``vcs`` (ubuntu package ``python3-vcstool``).

Make sure you have your ROS2 environment sourced such that the ROS_VERSION environment variable is set.

Create a workspace (``libcaer_driver_ws``), clone this repo, and use ``vcs``
to pull in the remaining dependencies:

```
pkg=libcaer_driver
mkdir -p ~/${pkg}_ws/src
cd ~/${pkg}_ws
git clone https://github.com/ros-event-camera/${pkg}.git src/${pkg}
cd src
vcs import < ${pkg}/${pkg}.repos
cd ..
```

Now build (the cmake flag to export compile commands is optional):
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
. install/setup.bash
```

This driver provides its own version of ``libcaer``, but you still need to copy the udev file into place
and modify the group permissions:
```
sudo cp src/libcaer/lib/udev/rules.d/65-inivation.rules /etc/udev/rules.d/
sudo usermod -aG video ${USER}
sudo usermod -aG plugdev ${USER}
sudo udevadm trigger
sudo service udev restart
```
Now you need to log out and back in to the host in order for the updated group permissions to take hold.


## Driver Features

Parameters:

- ``device_type``: "davis", "dvxplorer", ...
- ``serial``: specifies serial number of camera to open (useful if you have multiple cameras connected). You can learn the serial number via ``lsusb -v -d 152a: | grep iSerial``, or just start the driver with the serial number left blank, and look at the console log.
- ``event_message_time_threshold``: (in seconds) minimum time span of
  events to be aggregated in one ROS event message before message is sent. Defaults to 1ms.
- ``event_message_size_threshold``: (in bytes) minimum size of events
  (in bytes) to be aggregated in one ROS event message before message is sent. Defaults to 1MB.
- ``statistics_print_interval``: time in seconds between statistics printouts.
- ``send_queue_size``: outgoing ROS message send queue size (defaults
  to 1000 messages).
- ``frame_id``: the frame id to use in the ROS message header
- ``save_biases``: write out current bias settings to bias file. For
  this to work the ``bias_file`` parameter must be set to a non-empty value.

# How to use (ROS2):

For efficient recording of the events you need to run the
driver and the recorder in the same address space as ROS2 composable
nodes. For this you will need to install the
[composable recorder](https://github.com/berndpfrommer/rosbag2_composable_recorder)
into your workspace as well (see below).

```
ros2 launch libcaer_driver driver_node.launch.py        # (run as node)
ros2 launch libcaer_driver driver_composition.launch.py # (run as composable node)
```

To use the combined driver/recorder and start the recording:
```
ros2 launch libcaer_driver recording_driver.launch.py
ros2 run rosbag2_composable_recorder start_recording.py
```
To stop the recording you have to kill (Ctrl-C) the recording driver.

To visualize the events, run a ``renderer`` node from the
[event_camera_renderer](https://github.com/ros-event-camera/event_camera_renderer) package:
```
ros2 launch event_camera_renderer renderer.launch.py
```
The renderer node publishes an image that can be visualized with e.g. ``rqt_image_view``

## CPU load

Here are some approximate performance numbers on a 16 thread (8-core) AMD Ryzen 7480h laptop with max clock speed of 2.9GHz. The below numbers were obtained with a DvXplorer (bias sensitivity set to 4, sensor illuminated uniformly with square wave at 600Hz, delivering about 130-140 MeVs, 1150MB/s). Note that at this data rate, the decoding of the USB packets by libcaer saturates the CPU.

THESE NUMBERS ARE PLACEHOLDERS, IGNORE FOR NOW!

| settings                       |CPU load         | note                                  |
|--------------------------------|-----------------|---------------------------------------|
| driver no subscriber           | 22%             | topic not published                   |
| driver with subscriber         | 35%             | does interprocess communication       |
| driver + rosbag record node    | 80%             | cpu load for combined driver + record |
| driver + rosbag record composable | 58%          | single process no ipc but disk/io     |

### About ROS time stamps

A ROS message potentially aggregates multiple libcaer packets. The ROS message header stamp is the host arrival time of the *first* libcaer packet contained in the ROS message.

## License

This software is issued under the Apache License Version 2.0.
