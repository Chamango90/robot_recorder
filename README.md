# Robot Recorder: Offline ROS -> three.js

This package can be used to record the joints and/or the mobile changes of a ROS system as three.js animation.
The result can e.g. be display in gh-pages. Find an example here: https://ipa-jfh.github.io/robot_recorder/

[![record_1](https://user-images.githubusercontent.com/17281534/40248316-d6fe0c0a-5acf-11e8-9d53-72547f7f4cf2.gif)](https://ipa-jfh.github.io/robot_recorder/)

In order to reduce the recorded data it does not apply a fixed rate but keyframes. Furthermore, it will not start before the first movement has occured.

### Usecases

- Show a 3D demo of your ROS robot or application
- Show the result of CI

## How to use?

1. "Automatic mode": By default the node records as soon as it is started and saves data as soon as it is closed. 

1. "Manual mode": Use the ROS services `~start` and `~stop`.

## What can be recorded?

1. `/joint_states` 

    This topic provides the changes of the robot joints and represents the **internal motion** of the robot. It can be throttled by [rate][1] and by [change][2] to reduce the recorded keyframes.
  
1. `/tf`

    This topic should be used to get the **external motion** of the robot with respect to a fixed world/map. It can be throttled by [change][3] to reduce the recorded keyframes. The referenced node considers by default only the tf change between the frames `map` <--> `base_link`.
  
## Which output-format is used?

The recorded robot movements are saved as animation of the `JSON Object format` ([three.js example][4]) which can be utilized by the three.js [animation system][5].

The example [index_to_gif.html][6] shows how to generate a GIF out of the animated three.js stage.

[![record_to_gif](https://user-images.githubusercontent.com/17281534/40250138-a4dc6266-5ad5-11e8-8672-a7fb25e5976d.png)](https://ipa-jfh.github.io/robot_recorder/index_to_gif)

https://ipa-jfh.github.io/robot_recorder/index_to_gif

## How is the URDF visualized?

With the help of https://github.com/gkjohnson/urdf-loaders

[1]: http://wiki.ros.org/tf#change_notifier
[2]: ./scripts/throttle_joints_by_change
[3]: http://wiki.ros.org/topic_tools/throttle
[4]: https://threejs.org/examples/#webgl_animation_keyframes_json
[5]: https://threejs.org/docs/#manual/introduction/Animation-system
[6]: ./docs/index_to_gif.html
