""" Implementation of the `Mapping` abstract base class that provides an 
    interface with the parameters of a ROS2 node.

A [mapping] is a dict-like data structure that pairs keys with values. For more 
information about the Mapping class, see the documentation for the 
[collections.abc] package.

Example
-------

This example must be run from within a [configured ROS2 environment].

Initialize a ROS2 interface.

>>> import rclpy
>>> rclpy.init()

Create a ROS2 node.

>>> import rclpy.node
>>> node = rclpy.node.Node('test')

Create a mapping interface to the node parameters.

>>> mapping = Mapping()
>>> mapping.node = node

With no parameters declared, verify that the length is zero.

>>> len(mapping) == 0
True

Initialize several ROS2 parameters.

>>> parameter_map = dict(one=1, two=2.0, three='three')
>>> parameters = [node.declare_parameter(k, v) 
...               for (k, v) in parameter_map.items()]

Test parameter access.

>>> mapping['one']
1
>>> mapping['two']
2.0
>>> mapping['three']
'three'

Verify that the length is three.

>>> len(mapping)
3

Test the iterator protocol.

>>> [k1 == k2 for (k1, k2) in zip(parameter_map, mapping)]
[True, True, True]
>>> next(iter(mapping))
'one'

Verify that the length decreases as parameters are removed.

>>> node.undeclare_parameter('one')
>>> len(mapping)
2
>>> next(iter(mapping))
'two'

Test access to a missing key.

>>> try: mapping['one']
... except KeyError as e: print('Key not found')
Key not found

Test the items iterator.

>>> tuple(mapping.items())
(('two', 2.0), ('three', 'three'))

Test the get accessor.

>>> mapping.get('four', 4.0)
4.0
>>> mapping.get('two', 4.0)
2.0
>>> mapping.get('four')

Test the equality operator using a dict copy.

>>> data = {k: v for (k, v) in mapping.items()}
>>> mapping == data
True
>>> data['two'] = 'two'
>>> mapping == data
False

Clean up by destroying the node and shutting down the ROS2 interface.

>>> del mapping
>>> node.destroy_node()
>>> rclpy.shutdown()

References
----------

[__contains__]: https://docs.python.org/3/reference/datamodel.html#object.__contains__

[__iter__]: https://docs.python.org/3/reference/datamodel.html#object.__iter__

[__len__]: https://docs.python.org/3/reference/datamodel.html#object.__len__

[collections.abc]: https://docs.python.org/3/library/mappings.abc.html

[Python len]: https://docs.python.org/3/library/functions.html#len

[configured ROS2 environment]: https://docs.ros.org/en/humble/Tutorials/Configuring-ROS2-Environment.html

[get_parameters_by_prefix]: https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node.get_parameters_by_prefix

[mapping]: https://docs.python.org/3/glossary.html#term-mapping
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

# ROS2 imports.
from rclpy.exceptions import ParameterNotDeclaredException

# Local imports.
from ros_parameter_collections.collection import Collection


# Implement the mapping class.
class Mapping(Collection, collections.abc.Mapping):
    """ Implementation of `mappings.abc.Mapping` as an interface to the 
        parameters of a ROS2 node.
    
    Attributes
    ----------
    node : rclpy.node.Node
        ROS2 node that this mapping interface will wrap.
    """
    
    def __getitem__(self, key):
        """ Access a ROS2 parameter by key.
        
        ```{todo}
        Support for hierarchical parameter keys via get_parameters_by_prefix.
        ```
        
        Parameters
        ----------
        key : str
            A ROS2 parameter key / name.
        
        Returns
        -------
        value
            Value of the parameter associated with the provided key.
        
        Raises
        ------
        TypeError
            If the key is not a string.
        KeyError
            If no parameter is associated with the provided key.
        """
        
        # Verify that the key is a string.
        if not isinstance(key, str): raise TypeError('Key must be a string')
        
        # Try to retrieve the parameter.
        # Catch get_parameter 'not declared' exception.
        try: value = self.node.get_parameter(key).value
        except ParameterNotDeclaredException as e: raise KeyError(*e.args)
        
        # Return the result.
        return value
        
    
  

# Main.
if __name__ == '__main__':
    import doctest
    doctest.testmod()
    
  

