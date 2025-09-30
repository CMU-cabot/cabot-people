# This script is based on ROS2 point_cloud2.py under sensor_msgs_py
# with some modifications.
# https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs_py/sensor_msgs_py/point_cloud2.py

import array
from collections import namedtuple
import sys
from typing import Iterable, List, NamedTuple, Optional

import numpy as np
try:
    from numpy.lib.recfunctions import (structured_to_unstructured, unstructured_to_structured)
except ImportError:
    from sensor_msgs_py.numpy_compat import (structured_to_unstructured,
                                             unstructured_to_structured)

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


_DATATYPES = {}
_DATATYPES[PointField.INT8] = np.dtype(np.int8)
_DATATYPES[PointField.UINT8] = np.dtype(np.uint8)
_DATATYPES[PointField.INT16] = np.dtype(np.int16)
_DATATYPES[PointField.UINT16] = np.dtype(np.uint16)
_DATATYPES[PointField.INT32] = np.dtype(np.int32)
_DATATYPES[PointField.UINT32] = np.dtype(np.uint32)
_DATATYPES[PointField.FLOAT32] = np.dtype(np.float32)
_DATATYPES[PointField.FLOAT64] = np.dtype(np.float64)

DUMMY_FIELD_PREFIX = 'unnamed_field'


def read_points(
        cloud: PointCloud2,
        field_names: Optional[List[str]] = None,
        skip_nans: bool = False,
        uvs: Optional[Iterable] = None,
        reshape_organized_cloud: bool = False) -> np.ndarray:
    """
    Read points from a sensor_msgs.PointCloud2 message.

    :param cloud: The point cloud to read from sensor_msgs.PointCloud2.
    :param field_names: The names of fields to read. If None, read all fields.
                        (Type: Iterable, Default: None)
    :param skip_nans: If True, then don't return any point with a NaN value.
                      (Type: Bool, Default: False)
    :param uvs: If specified, then only return the points at the given
        coordinates. (Type: Iterable, Default: None)
    :param reshape_organized_cloud: Returns the array as an 2D organized point cloud if set.
    :return: Structured NumPy array containing all points.
    """
    assert isinstance(cloud, PointCloud2), \
        'Cloud is not a sensor_msgs.msg.PointCloud2'

    # Cast bytes to numpy array
    points = np.ndarray(
        shape=(cloud.width * cloud.height, ),
        dtype=dtype_from_fields(cloud.fields, point_step=cloud.point_step),
        buffer=cloud.data)

    # Keep only the requested fields
    if field_names is not None:
        assert all(field_name in points.dtype.names for field_name in field_names), \
            'Requests field is not in the fields of the PointCloud!'
        # Mask fields
        points = points[list(field_names)]

    # Swap array if byte order does not match
    if bool(sys.byteorder != 'little') != bool(cloud.is_bigendian):
        points = points.byteswap(inplace=True)

    # Check if we want to drop points with nan values
    if skip_nans and not cloud.is_dense:
        # Init mask which selects all points
        not_nan_mask = np.ones(len(points), dtype=bool)
        for field_name in points.dtype.names:
            # Only keep points without any non values in the mask
            not_nan_mask = np.logical_and(
                not_nan_mask, ~np.isnan(points[field_name]))
        # Select these points
        points = points[not_nan_mask]

    # Select points indexed by the uvs field
    if uvs is not None:
        # Don't convert to numpy array if it is already one
        if not isinstance(uvs, np.ndarray):
            uvs = np.fromiter(uvs, int)
        # Index requested points
        points = points[uvs]

    # Cast into 2d array if cloud is 'organized'
    if reshape_organized_cloud and cloud.height > 1:
        points = points.reshape(cloud.width, cloud.height)

    return points


def read_points_numpy(
        cloud: PointCloud2,
        field_names: Optional[List[str]] = None,
        skip_nans: bool = False,
        uvs: Optional[Iterable] = None,
        reshape_organized_cloud: bool = False) -> np.ndarray:
    """
    Read equally typed fields from sensor_msgs.PointCloud2 message as a unstructured numpy array.

    This method is better suited if one wants to perform math operations
    on e.g. all x,y,z fields.
    But it is limited to fields with the same dtype as unstructured numpy arrays
    only contain one dtype.

    :param cloud: The point cloud to read from sensor_msgs.PointCloud2.
    :param field_names: The names of fields to read. If None, read all fields.
                        (Type: Iterable, Default: None)
    :param skip_nans: If True, then don't return any point with a NaN value.
                      (Type: Bool, Default: False)
    :param uvs: If specified, then only return the points at the given
        coordinates. (Type: Iterable, Default: None)
    :param reshape_organized_cloud: Returns the array as an 2D organized point cloud if set.
    :return: Numpy array containing all points.
    """
    # NOTE: Only this assert is changed
    #assert all(cloud.fields[0].datatype == field.datatype for field in cloud.fields[1:]), \
    #    'All fields need to have the same datatype. Use `read_points()` otherwise.'
    dtypes = []
    for field in cloud.fields:
        if field.name in field_names:
            dtypes.append(field.datatype)
    assert all(dtypes[0] == dtype for dtype in dtypes[1:]), \
        'All fields need to have the same datatype. Use `read_points()` otherwise.'

    structured_numpy_array = read_points(
        cloud, field_names, skip_nans, uvs, reshape_organized_cloud)
    return structured_to_unstructured(structured_numpy_array)


def dtype_from_fields(fields: Iterable[PointField], point_step: Optional[int] = None) -> np.dtype:
    """
    Convert a Iterable of sensor_msgs.msg.PointField messages to a np.dtype.

    :param fields: The point cloud fields.
                   (Type: iterable of sensor_msgs.msg.PointField)
    :param point_step: Point step size in bytes. Calculated from the given fields by default.
                       (Type: optional of integer)
    :returns: NumPy datatype
    """
    # Create a lists containing the names, offsets and datatypes of all fields
    field_names = []
    field_offsets = []
    field_datatypes = []
    for i, field in enumerate(fields):
        # Datatype as numpy datatype
        datatype = _DATATYPES[field.datatype]
        # Name field
        if field.name == '':
            name = f'{DUMMY_FIELD_PREFIX}_{i}'
        else:
            name = field.name
        # Handle fields with count > 1 by creating subfields with a suffix consiting
        # of "_" followed by the subfield counter [0 -> (count - 1)]
        assert field.count > 0, "Can't process fields with count = 0."
        for a in range(field.count):
            # Add suffix if we have multiple subfields
            if field.count > 1:
                subfield_name = f'{name}_{a}'
            else:
                subfield_name = name
            assert subfield_name not in field_names, 'Duplicate field names are not allowed!'
            field_names.append(subfield_name)
            # Create new offset that includes subfields
            field_offsets.append(field.offset + a * datatype.itemsize)
            field_datatypes.append(datatype.str)

    # Create dtype
    dtype_dict = {
            'names': field_names,
            'formats': field_datatypes,
            'offsets': field_offsets
    }
    if point_step is not None:
        dtype_dict['itemsize'] = point_step
    return np.dtype(dtype_dict)