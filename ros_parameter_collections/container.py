""" Implementation of the `Container` abstract base class that provides an 
    interface with the parameters of a ROS2 node.

Any implementation of the `Container` class must define the [__contains__] 
method. For more information about the Container class, see the documentation 
for the [collections.abc] package.

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

>>> container = Container()
>>> container.node = node

Initialize a ROS2 parameter.

>>> parameter = node.declare_parameter('key', 'value')

Test the existence of the parameter using the container interface.

>>> 'key' in container
True

Test the existence of a parameter that has not been declared.

>>> 'invalid' in container
False

Clean up by destroying the node and shutting down the ROS2 interface.

>>> del container
>>> node.destroy_node()
>>> rclpy.shutdown()

References
----------

[__contains__]: https://docs.python.org/3/reference/datamodel.html#object.__contains__

[collections.abc]: https://docs.python.org/3/library/collections.abc.html

[configured ROS2 environment]: https://docs.ros.org/en/humble/Tutorials/Configuring-ROS2-Environment.html

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
class Container(collections.abc.Container):
    """ Implementation of `collections.abc.Container` as an interface to the 
        parameters of a ROS2 node.
    
    Attributes
    ----------
    node : rclpy.node.Node
        ROS2 node that this container interface will wrap.
    """
    
    def __contains__(self, item):
        """ Test whether or not a parameter has been declared.
        
        ```{todo}
        Support for hierarchical parameter keys via get_parameters_by_prefix.
        ```
        
        Parameters
        ----------
        item : str
            A ROS2 parameter key / name.
        
        Returns
        -------
        has_parameter : bool
            True if the parameter has been declared by the node.
        """
        return self.node.has_parameter(item)
    
  

# Main.
if __name__ == '__main__':
    import doctest
    doctest.testmod()
    
  

