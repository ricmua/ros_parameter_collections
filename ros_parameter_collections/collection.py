""" Implementation of the `Collection` abstract base class that provides an 
    interface with the parameters of a ROS2 node.

Any implementation of the `Collection` class must define the [__contains__], 
[__iter__], and [__len__] methods. For more information about the 
Collection class, see the documentation for the [collections.abc] package.

Example
-------

This example must be run from within a [configured ROS2 environment].

Initialize a ROS2 interface.

>>> import rclpy
>>> rclpy.init()

Create a ROS2 node.

>>> import rclpy.node
>>> node = rclpy.node.Node('test')

Create a collection interface to the node parameters.

>>> collection = Collection()
>>> collection.node = node

With no parameters declared, verify that the length is zero.

>>> len(collection) == 0
True

Initialize several ROS2 parameters.

>>> parameter_map = dict(one=1, two=2.0, three='three')
>>> parameters = [node.declare_parameter(k, v) 
...               for (k, v) in parameter_map.items()]

Verify that the length is three.

>>> len(collection)
3

Test the iterator protocol.

>>> [k1 == k2 for (k1, k2) in zip(parameter_map, collection)]
[True, True, True]
>>> next(iter(collection))
'one'

Test the existence of a parameter.

>>> 'one' in collection
True

Verify that the length decreases as parameters are removed.

>>> node.undeclare_parameter('one')
>>> len(collection)
2
>>> next(iter(collection))
'two'

Test the existence of a parameter that is not declared.

>>> 'one' in collection
False

Clean up by destroying the node and shutting down the ROS2 interface.

>>> del collection
>>> node.destroy_node()
>>> rclpy.shutdown()

References
----------

[__contains__]: https://docs.python.org/3/reference/datamodel.html#object.__contains__

[__iter__]: https://docs.python.org/3/reference/datamodel.html#object.__iter__

[__len__]: https://docs.python.org/3/reference/datamodel.html#object.__len__

[collections.abc]: https://docs.python.org/3/library/collections.abc.html

[Python len]: https://docs.python.org/3/library/functions.html#len

[configured ROS2 environment]: https://docs.ros.org/en/humble/Tutorials/Configuring-ROS2-Environment.html

[get_parameters_by_prefix]: https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node.get_parameters_by_prefix

"""


# Copyright 2022 Carnegie Mellon University Neuromechatronics Lab (a.whit)
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# Contact: a.whit (nml@whit.contact)


# Import abstract base classes.
import collections.abc

# Local imports.
from ros_parameter_collections.container import Container
from ros_parameter_collections.iterable import Iterable
from ros_parameter_collections.sized import Sized


# Implement the collection class.
class Collection(Container, Iterable, Sized, collections.abc.Collection):
    """ Implementation of `collections.abc.Collection` as an interface to the 
        parameters of a ROS2 node.
    
    Attributes
    ----------
    node : rclpy.node.Node
        ROS2 node that this collection interface will wrap.
    """
    
    pass
    
  

# Main.
if __name__ == '__main__':
    import doctest
    doctest.testmod()
    
  

