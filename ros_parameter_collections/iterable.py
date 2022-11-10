""" Implementation of the `Iterable` abstract base class that provides an 
    interface with the parameters of a ROS2 node.

Any implementation of the `Iterable` class must define the [__iter__] 
method, which is used in the construction of [Python iterators], which support 
the [Python iterator protocol]. For more information about the Iterable class, 
see the documentation for the [collections.abc] package.

Iteration of ROS2 parameters is implemented using the 
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

>>> iterable = Iterable()
>>> iterable.node = node

Initialize several ROS2 parameters.

>>> parameter_map = dict(one=1, two=2.0, three='three')
>>> parameters = [node.declare_parameter(k, v) 
...               for (k, v) in parameter_map.items()]

Test the iterator protocol.

>>> [k1 == k2 for (k1, k2) in zip(parameter_map, iterable)]
[True, True, True]
>>> next(iter(iterable))
'one'

Clean up by destroying the node and shutting down the ROS2 interface.

>>> del iterable
>>> node.destroy_node()
>>> rclpy.shutdown()

References
----------

[__iter__]: https://docs.python.org/3/reference/datamodel.html#object.__iter__

[collections.abc]: https://docs.python.org/3/library/collections.abc.html

[Python iterators]: https://docs.python.org/3/glossary.html#term-iterator

[Python iterator protocol]: https://docs.python.org/3/library/stdtypes.html#typeiter

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
class Iterable(collections.abc.Iterable):
    """ Implementation of `collections.abc.Iterable` as an interface to the 
        parameters of a ROS2 node.
    
    Attributes
    ----------
    node : rclpy.node.Node
        ROS2 node that this container interface will wrap.
    """
    
    def __iter__(self):
        """ Return parameter key iterator.
        
        Returns
        -------
        parameters_map : dict
            Since the dict type implements the iterator protocol for the keys 
            of the mapping, this class effectively implements the same 
            protocol for the parameters keys / names.
        """
        # An empty string matches all parameters.
        parameter_map = self.node.get_parameters_by_prefix('')
        
        # Exclude the `use_sim_time` parameter.
        parameter_map = {k: v 
                         for (k, v) in parameter_map.items() 
                         if (k not in ['use_sim_time'])}
        
        # Return the result.
        return iter(parameter_map)
    
  

# Main.
if __name__ == '__main__':
    import doctest
    doctest.testmod()
    
  

