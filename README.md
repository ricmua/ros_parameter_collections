---
title: ROS2 Parameter Collections Package
author: a.whit ([email](mailto:nml@whit.contact))
date: July 2022
---

<!-- License

Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University (a.whit)

Created by: a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
-->

# ROS2 Parameter Collections

A [ROS2 package] that provides an interface for treating [ROS2 parameters] 
(and parameter groups) like standard Python container data types (e.g., `dict`, 
`list`, `set`, and `tuple`).

For more information about Python containers, see the documentation for 
[emulating container types], as well as the documentation for the 
[collections package]. The [table of collection types] in the `collections.abc` 
documentation can be especially informative.

For more information about working with ROS2 parameters, see the 
[Understanding parameters tutorial].

## Available collection types

At present, only the following classes from [collections.abc] are implemented:

* `Container`
* `Iterable`
* `Sized`
* `Collection`
* `Mapping`
* `MutableMapping`

~~In addition, the `Dict` class is defined as a child of `MutableMapping` 
that implements a subset of [dict] functionality. In particular, the 
[setdefault] method is implemented.~~

## Installation

This package can be added to any [ROS2 workspace]. ROS2 workspaces are built using [colcon]. See the [installation documentation](doc/markdown/installation.md) 
for further information.

### Testing

See the [testing documentation](doc/markdown/testing.md) for further 
information.

## Example

Forthcoming.[^docstrings]

[^docstrings]: See examples in the docstrings of each Python module.

## License

Copyright 2022 [Neuromechatronics Lab][neuromechatronics], 
Carnegie Mellon University

Contributors: 

* a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.

<!---------------------------------------------------------------------
   References
---------------------------------------------------------------------->

[Python path]: https://docs.python.org/3/tutorial/modules.html#the-module-search-path

[doctest]: https://docs.python.org/3/library/doctest.html

[rewarding outcome]: https://en.wikipedia.org/wiki/Reinforcement

[neural codes]: https://en.wikipedia.org/wiki/Neuronal_ensemble#Background

[motor cortex]: https://en.wikipedia.org/wiki/Primary_motor_cortex#Movement_coding

[center-out task]: https://pubmed.ncbi.nlm.nih.gov/3411362/

[pytransitions]: https://github.com/pytransitions/transitions

[doctest]: https://docs.python.org/3/library/doctest.html

[ros_transitions]: https://github.com/ricmua/ros_transitions

[separation of concerns]: https://en.wikipedia.org/wiki/Separation_of_concerns

[ros_force_dimension]: https://github.com/ricmua/ros_force_dimension

[ROS2]: https://docs.ros.org/en/humble/index.html

[Unity3D]: https://en.wikipedia.org/wiki/Unity_(game_engine)

[unity_spheres_environment]: https://github.com/ricmua/unity_spheres_environment

[setuptools]: https://setuptools.pypa.io/en/latest/userguide/quickstart.html#basic-use

[neuromechatronics]: https://www.meche.engineering.cmu.edu/faculty/neuromechatronics-lab.html

[pip install]: https://pip.pypa.io/en/stable/cli/pip_install/

[pytest]: https://docs.pytest.org/

[unittest]: https://docs.python.org/3/library/unittest.html

[ROS2 workspace]: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html

[Python collections]: https://docs.python.org/3/library/collections.abc.html#collections-abstract-base-classes

[ROS2 package]: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html#what-is-a-ros-2-package

[ROS2 parameters]: https://docs.ros.org/en/humble/Concepts/About-ROS-2-Parameters.html



[table of collection types]: https://docs.python.org/3/library/collections.abc.html#collections-abstract-base-classes
       
[collections.abc]: https://docs.python.org/3/library/collections.abc.html

[collections package]: https://docs.python.org/3/library/collections.html

[emulating container types]: https://docs.python.org/3/reference/datamodel.html#emulating-container-types

[Understanding parameters tutorial]: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html

[setdefault]: https://docs.python.org/3/library/stdtypes.html#dict.setdefault

