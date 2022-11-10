<!-- License

Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University (a.whit)

Contributors:
  a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
-->

## Installation

This package can be added to any [ROS2 workspace].

```bash
git clone https://github.com/ricmua/ros_parameter_collections.git path/to/workspace/src/
```

ROS2 workspaces are built using [colcon], from within a 
[configured ROS2 environment].

```bash
cd path/to/workspace
source path/to/ros/setup.bash
colcon build
```

### Microsoft Windows

The above examples are tailored to Linux installations. The installation 
commands will differ slightly for the Windows Operating system.

The command to initialize a configured ROS2 environment uses the `call` 
function on a `setup.bat`, instead of `source` on `setup.bash`. The path to the 
ROS2 installation will likely also differ.

```
call path/to/ros/setup.bat
```

The `--merge-install` [colcon build flag] is recommended:

> for workspaces with many packages otherwise the environment variables might 
  exceed the supported maximum length.

```
colcon build --merge-install
```

<!---------------------------------------------------------------------
   References
---------------------------------------------------------------------->

[ROS2]: https://docs.ros.org/en/humble/index.html

[ROS2 workspace]: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html

[colcon]: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html

[colcon build flag]: https://colcon.readthedocs.io/en/released/reference/verb/build.html

[configured ROS2 environment]: https://docs.ros.org/en/humble/Tutorials/Configuring-ROS2-Environment.html

