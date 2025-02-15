# Sensors Package
This ROS package configures and communicates with various sensors (the Depth Sensor, IMU, and the Teledyne Navigator Doppler Velocity Log (DVL)).

## Depth Sensor

## IMU

## Teledyne Navigator

[status]: https://dev.mcgillrobotics.com/buildStatus/icon?job=ros-teledyne-navigator/master
[url]: https://dev.mcgillrobotics.com/job/ros-teledyne-navigator/job/master
[![status]][url]

This ROS package configures and communicates with the Teledyne Navigator
Doppler Velocity Log (DVL). **This has only been tested on ROS Kinetic and
Melodic over RS232.**

### Setting up

You must clone this repository as `teledyne_navigator` into your `catkin`
workspace's `src` directory:

```bash
roscd
cd src
git clone https://github.com/mcgill-robotics/ros-teledyne-navigator.git teledyne_navigator
```

### Dependencies

Before proceeding, make sure to install all the dependencies by running:

```bash
rosdep update
rosdep install teledyne_navigator
```

### Compiling

You **must** compile this package before being able to run it. You can do so
by running:

```bash
catkin_make
```

from the root of your workspace.

### Running

To run, simply connect the DVL over RS232 and launch the package with:

```bash
roslaunch teledyne_navigator teledyne_navigator.launch port:=</path/to/dvl>
```

The following run-time ROS launch arguments are available:

- `port`: Serial port to read from, default: `/dev/dvl`.
- `baudrate`: Serial baud rate, default: `9600`.
- `timeout`: Serial read timeout in seconds, default: `1.0`.
- `frame`: `tf` frame to stamp the messages with, default: `dvl`.

The package will keep trying to connect to the DVL until it is successful.

The `teledyne_navigator` node will output to the following ROS topic:

- `~ensemble`: DVL scan data as an `Ensemble` message.

### Limitations

This driver is limited to the PD5 output format and the earth-coordinate frame
(ENU). You may change other parameters as you please using TRDI Toolz and save
them with `CK`, but the `PD` and `EX` commands will be overridden.

### Contributing

Contributions are welcome. Simply open an issue or pull request on the matter,
and it will be accepted as long as it does not complicate the code base too
much.

As for style guides, we follow the ROS Python Style Guide for ROS-specifics and
the Google Python Style Guide for everything else.

### Linting

We use [YAPF](https://github.com/google/yapf) for all Python formatting needs.
You can auto-format your changes with the following command:

```bash
yapf --recursive --in-place --parallel .
```

We also use [catkin_lint](https://github.com/fkie/catkin_lint) for all `catkin`
specifics. You can lint your changes as follows:

```bash
catkin lint --explain -W2 .
```

### License

See [LICENSE](LICENSE).

