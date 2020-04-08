# Porting a project from ROS1 to ROS2 - our experience

At Husarion we have developed an open-sourced GUI for managing autonomous mobile robot destinations. 
It is a web browser based and allows user to set single destination or set of destinations as a sequnece or loop. 
Everything with user friendly and intuitive graphic interface and out of the box experience without a single line of code. 
The project is named [Route Admin Panel](https://husarion.com/software/route-admin-panel/), or in short version RAP. 
It was initially created for ROS1 and successfuly tested with Kinetic and Melodic.

RAP is a pre-installed web user interface on 
[ROSbot 2.0](https://husarion.com/manuals/rosbot-manual/) 
providing a very handy way to start using it even with no coding. 
Increasing [efforts](https://metrics.ros.org/rosdistro_rosdistro.html) of ROS community on development on ROS2 
as well as questions from our users convinced us provide dedicated [ROS2 image for ROSbot](https://husarion.com/downloads/). 
It comes with ROS Dashing with all desktop tools. 
The communication middleware of our choice is Cyclone DDS as we consider it as most stable and effective. 
Moreover, what is important form a developer perspective, we find its documentation as complete and comprehensive. 
Images are prepared for ARM architecture with supported device `ASUS TinkerBoard` as well as for Intel architecture with supported device `UpBoard`.

We decided that the default software working out-of-the-box, also beeing the showcase for ROSbot capabilites, 
will be RAP, thus it needed to be ported to ROS2. 
Below are experiences we gathered and challengen we faced during the process.

## Architecture

RAP is a `nodejs` application, which consists of a backend server and frontend web browser app. 
Communication between server and browser is ROS agnostic, we are using JSON structure for this part. 
This means that there is no need do make any changes in browser app and user experience will stay the same.

Translation to ROS messages is handled on server side, RAP for ROS1 is using (rosnodejs)[http://wiki.ros.org/rosnodejs] for interfacing between ROS and server. 
There is no rosnodejs for ROS2, the replacement package is (rclnodejs)[https://github.com/RobotWebTools/rclnodejs], 
it provides roughly the same functionality with interfaces adjusted to ROS2.

RAP is subscribing `TF` messages to update rosbot position on a map and `Image` with map through `CompressedImage` transport plugin, 
furthermore in ROS1 it was making use of `/move_base` action API which was replaced with `nav2` stack.

We tested it with (NodeJS)[https://nodejs.org/en/] version 10.18.

## Porting process

Knowing the architecture, we can determine that main thig to change is to replace `rosnodejs` with `rclnodejs`. 
Many interfaces in both libraries are the same, thus swithcing `TF` topic was mostly replacing library name and refomratting messsages.

First significant issue was encounetered while switching `Image` topic. 
In order to save the bandwidth, map is transferred to browser as a PNG compressed image. 
In ROS 1 there was a dedicated node which subscribes `/map` topic with `nav_msgs/OccupancyGrid` message type and 
republishes is on `/map_img` topic with use of `image_transport` plugin. 
Server side is subscribing the `CompressedImage` from `image_transport` and transfers it directly to browser app. 
For ROS2 it was necessary to implement node for conversion of `nav_msgs/OccupancyGrid` to `sensor_msgs/CompressedImage`. 
This process is described in section [Map to image converter](#map-to-image-converter). 
Also there was an issue with parameter declaration for CompressedImage transport plugin. 
Parameter declaration issue is described in [CompressedImage transport plugin issue](#compressedimage-transport-plugin-issue)

The most breaking change was setting destination for navigation stack, as it was redesigned in ROS2. 
Details of swithcing to `navigation2` stack are in [Path planning and destination setting](#path-planning-and-destination-setting).

## Map to image converter

Node for converting `nav_msgs::msg::OccupancyGrid` to `sensor_msgs::msg::Image` was rewritten for ROS2 compliancy. 
Task for this node is pretty straghtforward, subscribe `/map` topic, convert occupancy grid into image and publish it.

First, initialize subscriber to `/map` topic:
```
map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 1, std::bind(&MapAsImageProvider::mapUpdate, this, std::placeholders::_1));
```

Initialize `/map_image` publisher as `ImageTransport` plugin:
```
image_transport_ = new image_transport::ImageTransport(static_cast<rclcpp::Node::SharedPtr>(this));
image_transport_publisher_full_ = image_transport_->advertise("/map_image", 1, true);
```

Then in subscriber callback function, process map and publish it as an image:

```
tmp_image_data = *cv_img_full_.toImageMsg();
image_transport_publisher_full_.publish(tmp_image_data);
```

When initializing image transport, there is no possibility to choose image format or compression ratio. 
The plugins are loaded at runtime when node is launched, exact configuration depends on image transport plugins available in the system.

## CompressedImage transport plugin issue

RAP is subscribing map image as `CompressedImage` with PNG compression. 
(Image_transport_plugins)[https://github.com/ros-perception/image_transport_plugins/tree/ros2]
 does support PNG compression, but at first try, it was constantly publishing as JPEG which is default format. 
 Even when `format: png` parameter was set. Turns out, that since Dashing , parameter declaration changed as described in 
 (documentation)[https://index.ros.org/doc/ros2/Releases/Release-Dashing-Diademata/#declaring-parameters]. 
 Unfortunately, the declaration API was not impelmented in `ros2` branch of `image_transport_plugins`. 

We have prepared a forked repository with required changes, as for the time of writing, 
according (PR)[https://github.com/ros-perception/image_transport_plugins/pull/52] is pending.

### Parameter declaration

So how are parameters declared now?

We will discuss it on compression format example:

```
rcl_interfaces::msg::ParameterDescriptor format_description;
format_description.name = "format";
format_description.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
format_description.description = "Compression method";
format_description.read_only = false;
format_description.additional_constraints = "Supported values: [jpeg, png]";
config_.format = node->declare_parameter("format", kDefaultFormat, format_description);
```

We begin with ParameterDescriptor structure which will contain all relevant info regarding a parameter thath we want to set:
```
rcl_interfaces::msg::ParameterDescriptor format_description;
```

Set a name for the parameter, this is a string that identifies parameter, 
the same name is to be used when setting value for the parameter, e.g. in parameters `.yaml` file.

```
format_description.name = "format";
```

Choose parameter datatype, it could be one of the following:
  - `PARAMETER_NOT_SET`
  - `PARAMETER_BOOL`
  - `PARAMETER_INTEGER`
  - `PARAMETER_DOUBLE`
  - `PARAMETER_STRING`
  - `PARAMETER_BYTE_ARRAY`
  - `PARAMETER_BOOL_ARRAY`
  - `PARAMETER_INTEGER_ARRAY`
  - `PARAMETER_DOUBLE_ARRAY`
  - `PARAMETER_STRING_ARRAY`

```
format_description.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
```

Provide parameter description, this should explain what could be set or how does the parameter affect node operation.

```
format_description.description = "Compression method";
```

Determine if value can be changed after it has been initialized, if `true`, value will not be ovorwritten.

```
format_description.read_only = false;
```

Describe parameter constraints, `ParameterDescriptor` provides `floating_point_range` and `integer_range` 
fields which can be used to limit numeric values set in parameter, in our case they does not apply as we are using string parameter. 
For the purpose of determinig possible values, we can use `additional_constraints` field, 
this should be plain English description of constraints which cannot be expressed with the available constraints. 

```
format_description.additional_constraints = "Supported values: [jpeg, png]";
```

Finally, declare parameter:
```
config_.format = node->declare_parameter("format", kDefaultFormat, format_description);
```

Return value of `declare_parameter` method is parameter resulting value. 
If, at run-time, the user has provided an initial value then it will be set in this method, otherwise the given default_value will be set. 

## Path planning and destination setting

Path planning in ROS2 was almost redesigned from scratch, this allows to implement new workflow while keeping the functionality, 
for details of implementation please refer to [Navigation 2 docs](https://ros-planning.github.io/navigation2/concepts/index.html#). 

Altough the changes are fundamental, they refer mainly to navigation stack internal issues. 
When using Rviz, default settings allows us to set destination for ROSbot and drive towards it smoothly.

Setting the destination and monitoring its progress is available via [actions](http://design.ros2.org/articles/actions.html) interface, 
their implementation changed greatly. 
In ROS1, actions are implemented as a separeate library, while in ROS2 they should be included in client library.

When we started the process of porting RAP to ROS2, rclnodejs did not have support for actions interface. 
We decided to make our own implementation of actions interface and eventually create a pull request. 
During our development, the other team was concurrently working on the same functionality. 
The status for the time of writing this article is that rclnodejs has implementation of actions available for ROS2 Eloquent. 
As we want RAP to be compatible with ROS Dashing, we will keep with our [custom fork](https://github.com/lukaszmitka/rclnodejs).

As defined in actions design, their support is already implemented in [`rcl` library](https://github.com/ros2/rcl). 
To add actions support in rclnodejs it is required to add appropriate language bindings between 
`JavaScript` in `rclnodejs` and `C` in `rcl`. Also, according classes to process actions needs to be implemented. 

Last thing is to provide data type generator for actions. 
Every ROS message, service or action is defined in `.msg`, `.srv` or `.action` accordingly. 
Each of them can also be defined with use of universal `.idl` file. 
Both methods are not directly usable in `Node.js`, thus they need to be converted to according `JavaScript` classes prior to usage. 
Furthermore, actions must be converted during install time on user machine to porvide support for user defined actions.

## Summary

Porting the project from ROS1 to ROS2 was quite challneging due to variety od tools and components used. 
It was also as much valuable experience, that's why we decided to share it in this article.

We have discussed usage of `image_transport` plugin, parameter declaration and intergation of `Node.js` library with ROS2 interface. 
If any of these issues are interesting for you, please let us know in rthe comment below.

RAP is an open source project and code for ROS1 and ROS2 is available as separate branches 
[on the GitHub](https://github.com/husarion/route_admin_panel). 

I hope description of our experiences gathered during porting open source project from ROS1 to ROS2 will be helpful for ROS community.

All the best!