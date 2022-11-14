""" A [ROS2 package] that provides an interface for treating [ROS2 parameters] 
    (and parameter groups) like standard Python container data types (e.g., 
    `dict`, `list`, `set`, and `tuple`).

For more information about Python containers, see the documentation for 
[emulating container types], as well as the documentation for the 
[collections package]. The [table of collection types] in the `collections.abc` 
documentation can be especially informative.

For more information about working with ROS2 parameters, see the 
[Understanding parameters tutorial].

References
----------

[ROS2 package]: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html#what-is-a-ros-2-package

[ROS2 parameters]: https://docs.ros.org/en/humble/Concepts/About-ROS-2-Parameters.html

[table of collection types]: https://docs.python.org/3/library/collections.abc.html#collections-abstract-base-classes
       
[collections.abc]: https://docs.python.org/3/library/collections.abc.html

[collections package]: https://docs.python.org/3/library/collections.html

[emulating container types]: https://docs.python.org/3/reference/datamodel.html#emulating-container-types

[Understanding parameters tutorial]: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html

[setdefault]: https://docs.python.org/3/library/stdtypes.html#dict.setdefault
"""


# Copyright 2022 Carnegie Mellon University Neuromechatronics Lab (a.whit)
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# Contact: a.whit (nml@whit.contact)

from .container import Container
from .iterable import Iterable
from .sized import Sized
from .collection import Collection
from .mapping import Mapping
from .mutable_mapping import MutableMapping

