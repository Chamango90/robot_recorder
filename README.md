# Robot Recorder: Record ROS for web-animation

This package can be used to record the joints and/or the mobile changes of a ROS system as three.js animation.
The result can e.g. be display in gh-pages. Find example here: 

## Examples

<a href="https://ipa-jfh.github.io/robot_recorder/">
    <img src="https://user-images.githubusercontent.com/17281534/40248316-d6fe0c0a-5acf-11e8-9d53-72547f7f4cf2.gif" width="430" height="250">
</a>
<a href="https://ipa-jfh.github.io/robot_recorder/tb3_burger">
    <img src="https://user-images.githubusercontent.com/17281534/41863144-32257e44-78a6-11e8-98d4-2f6269a4cf3e.gif" width="430" height="250">
</a>

Live demos: [UniversalRobot](https://ipa-jfh.github.io/robot_recorder/), [Turtlebot3](https://ipa-jfh.github.io/robot_recorder/tb3_burger)

In order to reduce the recorded data it does not apply a fixed rate but keyframes. Furthermore, it will not start before the first movement has occured.

## Usecases

- Show a 3D demo of your ROS robot or application in the browser
- Show the CI test result(s) of your application

## How to use?

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

The example [index_to_gif.html][6] shows how to generate a GIF out of the animated three.js stage.

[![record_to_gif](https://user-images.githubusercontent.com/17281534/40250138-a4dc6266-5ad5-11e8-8672-a7fb25e5976d.png)](https://ipa-jfh.github.io/robot_recorder/index_to_gif)

Live demo: https://ipa-jfh.github.io/robot_recorder/index_to_gif

## How is the URDF visualized in the three.js scene?

With the `urdf-loader` tool: https://github.com/gkjohnson/urdf-loaders

[1]: http://wiki.ros.org/tf#change_notifier
[2]: ./scripts/throttle_joints_by_change
[3]: http://wiki.ros.org/topic_tools/throttle
[4]: https://threejs.org/examples/#webgl_animation_keyframes_json
[5]: https://threejs.org/docs/#manual/introduction/Animation-system
[6]: ./docs/index_to_gif.html
[7]: ./robot_recorder_core/launch/record.launch
