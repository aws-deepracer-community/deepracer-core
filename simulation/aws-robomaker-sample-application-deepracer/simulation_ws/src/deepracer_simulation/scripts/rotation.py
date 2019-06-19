#!/usr/bin/env python

"""This code is taken from
   https://github.com/scipy/scipy/blob/master/scipy/spatial/transform/rotation.py
   because there is no python 3 implementation This is a stripped down version
   intended to be used by deepracer only.
"""
import re
import numpy as np
import scipy.linalg

_AXIS_TO_IND = {'x': 0, 'y': 1, 'z': 2}

def _compose_quat(p, q):
    product = np.empty((max(p.shape[0], q.shape[0]), 4))
    product[:, 3] = p[:, 3] * q[:, 3] - np.sum(p[:, :3] * q[:, :3], axis=1)
    product[:, :3] = (p[:, None, 3] * q[:, :3] + q[:, None, 3] * p[:, :3] +
                      np.cross(p[:, :3], q[:, :3]))
    return product

def _make_elementary_quat(axis, angles):
    quat = np.zeros((angles.shape[0], 4))

    quat[:, 3] = np.cos(angles / 2)
    quat[:, _AXIS_TO_IND[axis]] = np.sin(angles / 2)
    return quat

def _elementary_quat_compose(seq, angles, intrinsic=False):
    result = _make_elementary_quat(seq[0], angles[:, 0])

    for idx, axis in enumerate(seq[1:], start=1):
        if intrinsic:
            result = _compose_quat(
                result,
                _make_elementary_quat(axis, angles[:, idx]))
        else:
            result = _compose_quat(
                _make_elementary_quat(axis, angles[:, idx]),
                result)
    return result

class Rotation(object):
    def __init__(self, quat, normalized=False, copy=True):
        self._single = False
        quat = np.asarray(quat, dtype=float)

        if quat.ndim not in [1, 2] or quat.shape[-1] != 4:
            raise ValueError("Expected `quat` to have shape (4,) or (N x 4), "
                             "got {}.".format(quat.shape))

        # If a single quaternion is given, convert it to a 2D 1 x 4 matrix but
        # set self._single to True so that we can return appropriate objects
        # in the `to_...` methods
        if quat.shape == (4,):
            quat = quat[None, :]
            self._single = True

        if normalized:
            self._quat = quat.copy() if copy else quat
        else:
            self._quat = quat.copy()
            norms = scipy.linalg.norm(quat, axis=1)

            zero_norms = norms == 0
            if zero_norms.any():
                raise ValueError("Found zero norm quaternions in `quat`.")

            # Ensure norm is broadcasted along each column.
            self._quat[~zero_norms] /= norms[~zero_norms][:, None]

    def __len__(self):
        """Number of rotations contained in this object.
        Multiple rotations can be stored in a single instance.
        Returns
        -------
        length : int
            Number of rotations stored in object.
        """
        return self._quat.shape[0]

    @classmethod
    def from_euler(cls, seq, angles, degrees=False):
        """Initialize from Euler angles.
        Rotations in 3 dimensions can be represented by a sequece of 3
        rotations around a sequence of axes. In theory, any three axes spanning
        the 3D Euclidean space are enough. In practice the axes of rotation are
        chosen to be the basis vectors.
        The three rotations can either be in a global frame of reference
        (extrinsic) or in a body centred frame of refernce (intrinsic), which
        is attached to, and moves with, the object under rotation [1]_.
        Parameters
        ----------
        seq : string
            Specifies sequence of axes for rotations. Up to 3 characters
            belonging to the set {'X', 'Y', 'Z'} for intrinsic rotations, or
            {'x', 'y', 'z'} for extrinsic rotations. Extrinsic and intrinsic
            rotations cannot be mixed in one function call.
        angles : float or array_like, shape (N,) or (N, [1 or 2 or 3])
            Euler angles specified in radians (`degrees` is False) or degrees
            (`degrees` is True).
            For a single character `seq`, `angles` can be:
            - a single value
            - array_like with shape (N,), where each `angle[i]`
              corresponds to a single rotation
            - array_like with shape (N, 1), where each `angle[i, 0]`
              corresponds to a single rotation
            For 2- and 3-character wide `seq`, `angles` can be:
            - array_like with shape (W,) where `W` is the width of
              `seq`, which corresponds to a single rotation with `W` axes
            - array_like with shape (N, W) where each `angle[i]`
              corresponds to a sequence of Euler angles describing a single
              rotation
        degrees : bool, optional
            If True, then the given angles are assumed to be in degrees.
            Default is False.
        Returns
        -------
        rotation : `Rotation` instance
            Object containing the rotation represented by the sequence of
            rotations around given axes with given angles.
        """
        num_axes = len(seq)
        if num_axes < 1 or num_axes > 3:
            raise ValueError("Expected axis specification to be a non-empty "
                             "string of upto 3 characters, got {}".format(seq))

        intrinsic = (re.match(r'^[XYZ]{1,3}$', seq) is not None)
        extrinsic = (re.match(r'^[xyz]{1,3}$', seq) is not None)
        if not (intrinsic or extrinsic):
            raise ValueError("Expected axes from `seq` to be from ['x', 'y', "
                             "'z'] or ['X', 'Y', 'Z'], got {}".format(seq))

        if any(seq[i] == seq[i+1] for i in range(num_axes - 1)):
            raise ValueError("Expected consecutive axes to be different, "
                             "got {}".format(seq))

        seq = seq.lower()

        angles = np.asarray(angles, dtype=float)
        if degrees:
            angles = np.deg2rad(angles)

        is_single = False
        # Prepare angles to have shape (num_rot, num_axes)
        if num_axes == 1:
            if angles.ndim == 0:
                # (1, 1)
                angles = angles.reshape((1, 1))
                is_single = True
            elif angles.ndim == 1:
                # (N, 1)
                angles = angles[:, None]
            elif angles.ndim == 2 and angles.shape[-1] != 1:
                raise ValueError("Expected `angles` parameter to have shape "
                                 "(N, 1), got {}.".format(angles.shape))
            elif angles.ndim > 2:
                raise ValueError("Expected float, 1D array, or 2D array for "
                                 "parameter `angles` corresponding to `seq`, "
                                 "got shape {}.".format(angles.shape))
        else:  # 2 or 3 axes
            if angles.ndim not in [1, 2] or angles.shape[-1] != num_axes:
                raise ValueError("Expected `angles` to be at most "
                                 "2-dimensional with width equal to number "
                                 "of axes specified, got {} for shape".format(angles.shape))

            if angles.ndim == 1:
                # (1, num_axes)
                angles = angles[None, :]
                is_single = True

        # By now angles should have shape (num_rot, num_axes)
        # sanity check
        if angles.ndim != 2 or angles.shape[-1] != num_axes:
            raise ValueError("Expected angles to have shape (num_rotations, "
                             "num_axes), got {}.".format(angles.shape))

        quat = _elementary_quat_compose(seq, angles, intrinsic)
        return cls(quat[0] if is_single else quat, normalized=True, copy=False)

    def as_quat(self):
        """ Represent as quaternions.
            Rotations in 3 dimensions can be represented using unit norm
            quaternions [1]_. The mapping from quaternions to rotations is
            two-to-one, i.e. quaternions ``q`` and ``-q``, where ``-q`` simply
            reverses the sign of each component, represent the same spatial
            rotation.
            Returns
            -------
            quat : `numpy.ndarray`, shape (4,) or (N, 4)
            Shape depends on shape of inputs used for initialization.
        """
        if self._single:
            return self._quat[0].copy()
        else:
            return self._quat.copy
