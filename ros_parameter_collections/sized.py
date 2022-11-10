""" Implementation of the `Sized` abstract base class that provides an 
    interface with the parameters of a ROS2 node.

Any implementation of the `Sized` class must define the [__len__] 
method, which is used by [Python len] function. For more information about the 
Sized class, see the documentation for the [collections.abc] package.

The number of ROS2 parameters is determined using the 
[get_parameters_by_prefix] method of `rclpy.node.Node`.

Example
-------

This example must be run from within a [configured ROS2 environment].

Initialize a ROS2 interface.

>>> import rclpy
>>> rclpy.init()

Create a ROS2 node.

>>> import rclpy.node
>>> node = rclpy.node.Node('test')

Create a container interface to the node parameters.

>>> sized = Sized()
>>> sized.node = node

With no parameters declared, verify that the length is zero.

>>> len(sized) == 0
True

With one parameter declared, verify that the length is one.

>>> parameter = node.declare_parameter('key', 'value')
>>> len(sized) == 1
True

With two parameters declared, verify that the length is two.

>>> another_parameter = node.declare_parameter('another_key', 'another_value')
>>> len(sized) == 2
True

Verify that the length decreases as parameters are removed.

>>> node.undeclare_parameter('another_key')
>>> len(sized) == 1
True

Clean up by destroying the node and shutting down the ROS2 interface.

>>> del sized
>>> node.destroy_node()
>>> rclpy.shutdown()

References
----------

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


# Implement the collection class.
class Sized(collections.abc.Sized):
    """ Implementation of `collections.abc.Sized` as an interface to the 
        parameters of a ROS2 node.
    
    Attributes
    ----------
    node : rclpy.node.Node
        ROS2 node that this container interface will wrap.
    """
    
    def __len__(self):
        """ Return the number of parameters declared for a ROS2 node.
        
        Returns
        -------
        N : int
            The number of declared ROS2 parameters.
        """
        # An empty string matches all parameters.
        parameter_map = self.node.get_parameters_by_prefix('')
        
        # Exclude the `use_sim_time` parameter.
        parameter_map = {k: v 
                         for (k, v) in parameter_map.items() 
                         if (k not in ['use_sim_time'])}
        
        # Return the result.
        return len(parameter_map)
    
  

# Main.
if __name__ == '__main__':
    import doctest
    doctest.testmod()
    
  

