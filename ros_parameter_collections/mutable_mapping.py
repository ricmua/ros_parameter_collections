""" Implementation of the `MutableMapping` abstract base class that provides an 
    interface with the parameters of a ROS2 node.

A [mapping] is a dict-like data structure that pairs keys with values. For more 
information about the MutableMapping class, see the documentation for the 
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

>>> mapping = MutableMapping()
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

>>> del mapping['one']
>>> len(mapping)
2
>>> next(iter(mapping))
'two'

Test assignment.

>>> mapping['four'] = 'IV'
>>> len(mapping)
3

Test access to a missing key.

>>> try: mapping['one']
... except KeyError as e: print('Key not found')
Key not found

Test pop.

>>> four = mapping.pop('four')
>>> 'four' in mapping
False
>>> four
'IV'

Test update.

>>> mapping.update(dict(four='4', five='V'))
>>> tuple(mapping.items())
(('two', 2.0), ('three', 'three'), ('four', '4'), ('five', 'V'))

Test popitem.

>>> mapping.popitem()
('two', 2.0)
>>> len(mapping)
3

Test clear.

>>> mapping.clear()
>>> len(mapping)
0

Test setdefault.

>>> mapping.setdefault('one', 1)
1
>>> mapping.setdefault('one', 'one')
1

Clean up by destroying the node and shutting down the ROS2 interface.

>>> del mapping
>>> node.destroy_node()
>>> rclpy.shutdown()

References
----------

[collections.abc]: https://docs.python.org/3/library/mappings.abc.html

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
import rclpy.parameter
import rclpy.exceptions
from rcl_interfaces.msg import SetParametersResult
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.exceptions import ParameterImmutableException

# Local imports.
from ros_parameter_collections.mapping import Mapping


# Establish a mapping between rclpy paramter types and builtin types.
parameter_type = rclpy.parameter.Parameter.Type
type_cast_map \
  = {parameter_type.NOT_SET:       lambda v: None,
     parameter_type.BOOL:          bool, 
     parameter_type.INTEGER:       int,
     parameter_type.DOUBLE:        float,
     parameter_type.STRING:        str,
     parameter_type.BOOL_ARRAY:    lambda v: [bool(vi) for vi in v],
     parameter_type.INTEGER_ARRAY: lambda v: [int(vi) for vi in v],
     parameter_type.DOUBLE_ARRAY:  lambda v: [float(vi) for vi in v],
     parameter_type.STRING_ARRAY:  lambda v: [str(vi) for vi in v],
     parameter_type.BYTE_ARRAY:    lambda v: v,
    }
""" Mapping for performing builtin type casts on rclpy Parameter data types.

This is necessary because the rclpy Parameter class does not implicitly cast 
like types. See the example.

Example
-------
>>> from rclpy.parameter import Parameter
>>> type_ = Parameter.Type.DOUBLE
>>> value = 42
>>> try: p = Parameter(name='test', type_=type_, value=value)
... except ValueError as e: print(e)
Type 'Type.DOUBLE' and value '42' do not agree
>>> p = Parameter(name='test', type_=type_, value=type_cast_map[type_](value))
"""

# Implement the mapping class.
class MutableMapping(Mapping, collections.abc.MutableMapping):
    """ Implementation of `mappings.abc.MutableMapping` as an interface to the 
        parameters of a ROS2 node.
    
    Attributes
    ----------
    node : rclpy.node.Node
        ROS2 node that this mapping interface will wrap.
    """
    
    def __setitem__(self, key, value):
        """ Access a ROS2 parameter by key, and set a new value.
        
        Parameters
        ----------
        key : str
            A ROS2 parameter key / name.
        value
            A valid parameter value to be associated with the provided key.
            
        Raises
        ------
        TypeError
            If the key is not a string.
        Exception
            If the parameter cannot be set.
        """
        
        # Verify that the key is a string.
        if not isinstance(key, str): raise TypeError('Key must be a string')
        
        # Determine the parameter type. Undeclare the existing parameter, if 
        # new type does not match the old.
        type_ = rclpy.parameter.Parameter.Type.from_parameter_value(value)
        if self.node.has_parameter(key):
            if self.node.get_parameter(key).type_ != type_:
                self.node.undeclare_parameter(key)
        
        # Initialize the parameter object for assignment.
        # This is slightly complicated due to lack of implicit casting by the 
        # rclpy Parameter class.
        kwargs = dict(name=key, type_=type_, value=type_cast_map[type_](value))
        parameter = rclpy.parameter.Parameter(**kwargs)
        
        # Try to set the parameter.
        # Declare the parameter, if it does not already exist.
        # The result is rcl_interfaces.msg.SetParametersResult.
        try: results = self.node.set_parameters([parameter]) 
        except ParameterNotDeclaredException:
            parameters = [self.node.declare_parameter(key, value)]
            results = [SetParametersResult(successful=True) for p in parameters]
        for result in results:
            if not result.successful: raise Exception(result.reason)
        
    def __delitem__(self, key):
        """ Undeclare a ROS2 parameter by key.
        
        Parameters
        ----------
        key : str
            A ROS2 parameter key / name.
            
        Raises
        ------
        TypeError
            If the key is not a string.
        KeyError
            If the parameter has not been declared.
        Exception
            If the parameter is immutable.
        """
        
        # Verify that the key is a string.
        if not isinstance(key, str): raise TypeError('Key must be a string')
        
        # Try to undeclare the parameter.
        try: self.node.undeclare_parameter(key)
        except ParameterNotDeclaredException as e: raise KeyError(*e.args)
        except ParameterImmutableException as e: 
            raise Exception('Immutable parameter')
    
  

# Main.
if __name__ == '__main__':
    import doctest
    doctest.testmod()
    
  

