# Robot Recorder: Record ROS for web-animation

This package can be used to record the joints and/or the mobile changes of a ROS system as three.js animation.
The result can e.g. be display in gh-pages. Find example here: 

## Examples

## Examples

<a href="https://ipa-jfh.github.io/urdf-animation/application_scan_and_plan/">
    <img src="https://user-images.githubusercontent.com/17281534/46005937-aafba700-c0b6-11e8-9d8f-0148392488f1.gif" width="430" height="250">
    >> 3D animation
</a>
<a href="https://ipa-jfh.github.io/urdf-animation/manipulator_ur5/">
    <img src="https://user-images.githubusercontent.com/17281534/46006694-6a9d2880-c0b8-11e8-9016-dc5c2799cf5c.gif" width="430" height="250">
    >> 3D animation
</a>
<a href="https://ipa-jfh.github.io/urdf-animation/mobile_robot_turtlebot3/">
    <img src="https://user-images.githubusercontent.com/17281534/46012246-e30be580-c0c8-11e8-953b-244bf7070d7b.gif" width="430" height="250">
    >> 3D animation
</a>


## Usecases

- Show a 3D demo of your ROS application
- Inspect failed CI test result(s)
- Visualize parameter studies

## How to use?

In order to reduce the recorded data it does not apply a fixed rate but keyframes. Furthermore, it will not start before the first movement has occured.

1. "Automatic mode": By default the node records* as soon as it is started and saves data as soon as it is closed.  
    -> Set arg `manual` to `false` in [record.launch][7]

1. "Manual mode": Control the tool via ROS services.  
    -> Set arg `manual` to `true` in [record.launch][7]  
    
    Available ROS services (private ns `~` is by default `robot_recorder/`):
    - `~preconfigure` (OPTIONAL): Start ROS subscribers and load `robot_description` param (Requires 2 secs).
    - `~start`: Preconfigures if not yet done and then starts recording*.
    - `~pause`: Pause and unpause the recording.
    - `~stop`: Stops the recording and saves it to the given file path (arg `output_file` in [record.launch][7])  
    ATTENTION: It will overwrite exiting files!
    
*Once the recorder is started it will wait until the robot "moves" (new `tf` or `joint_states` msgs) before it starts the actual recording. Throttling by change (see _What can be recorded?_) is recommended.

## What can be recorded?

1. `/joint_states` 

    This topic provides the changes of the robot joints and represents the **internal motion** of the robot. It can be throttled by [rate][1] and by [change][2] to reduce the recorded keyframes.
  
1. `/tf`

    This topic should be used to get the **external motion** of the robot with respect to a fixed world/map. It can be throttled by [change][3] to reduce the recorded keyframes. The referenced node considers by default only the tf change between the frames `map` <--> `base_link`.
  
## Which output-format is used?

The recorded robot movements are saved as animation of the `JSON Object format` ([three.js example][4]) which can be utilized by the three.js [animation system][5].

## How is the URDF visualized in the three.js scene?

With the `urdf-loader` tool: https://github.com/gkjohnson/urdf-loaders

## How to add animations and a GIF converter?

With the `urdf-animation` tool, which wraps the `urdf-loader`: https://github.com/ipa-jfh/urdf-animation

[1]: http://wiki.ros.org/tf#change_notifier
[2]: ./scripts/throttle_joints_by_change
[3]: http://wiki.ros.org/topic_tools/throttle
[4]: https://threejs.org/examples/#webgl_animation_keyframes_json
[5]: https://threejs.org/docs/#manual/introduction/Animation-system
[6]: ./docs/index_to_gif.html
[7]: ./robot_recorder_core/launch/record.launch
