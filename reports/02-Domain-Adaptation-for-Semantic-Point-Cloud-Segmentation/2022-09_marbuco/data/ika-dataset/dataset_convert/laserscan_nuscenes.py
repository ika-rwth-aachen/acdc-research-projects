#!/usr/bin/env python3
import numpy as np
from nuscenes.utils.data_classes import PointCloud
from nuscenes.utils.data_io import load_bin_file


class LidarPointCloud(PointCloud):

  @staticmethod
  def nbr_dims() -> int:
    """
    Returns the number of dimensions.
    :return: Number of dimensions.
    """
    return 5

  @classmethod
  def from_file(cls, file_name: str) -> 'LidarPointCloud':
    """
    Loads LIDAR data from binary numpy format. Data is stored as (x, y, z, intensity, ring index).
    :param file_name: Path of the pointcloud file on disk.
    :return: LidarPointCloud instance (x, y, z, intensity, ring index).
    """

    assert file_name.endswith('.bin'), 'Unsupported filetype {}'.format(file_name)

    scan = np.fromfile(file_name, dtype=np.float32)
    points = scan.reshape((-1, 5))[:, :cls.nbr_dims()]
    return cls(points.T)


class LidarSegPointCloud:
  """
  Class for a point cloud.
  """

  def __init__(self, points_path: str = None, labels_path: str = None):
    """
    Initialize a LidarSegPointCloud object.
    :param points_path: Path to the bin file containing the x, y, z and intensity of the points in the point cloud.
    :param labels_path: Path to the bin file containing the labels of the points in the point cloud.
    """
    self.points, self.labels = None, None
    if points_path:
      self.load_points(points_path)
    if labels_path:
      self.load_labels(labels_path)

  def load_points(self, path: str) -> None:
    """
    Loads the x, y, z and intensity of the points in the point cloud.
    :param path: Path to the bin file containing the x, y, z, intensity and ring index of the points in the point cloud.
    """
    self.points = LidarPointCloud.from_file(path).points.T  # [N, 5], where N is the number of points.
    if self.labels is not None:
      assert len(self.points) == len(self.labels), 'Error: There are {} points in the point cloud, ' \
                                                   'but {} labels'.format(len(self.points), len(self.labels))

  def load_labels(self, path: str) -> None:
    """
    Loads the labels of the points in the point cloud.
    :param path: Path to the bin file containing the labels of the points in the point cloud.
    """
    self.labels = load_bin_file(path)
    if self.points is not None:
      assert len(self.points) == len(self.labels), 'Error: There are {} points in the point cloud, ' \
                                                   'but {} labels'.format(len(self.points), len(self.labels))



class LaserScan:
  """Class that contains LaserScan with x,y,z,r"""
  EXTENSIONS_SCAN = ['.bin','.npy']

  def __init__(self, project=False, H=64, W=1024, fov_up=3.0, fov_down=-25.0, use_ring_projection=True):
    self.project = project
    self.proj_H = H
    self.proj_W = W
    self.proj_fov_up = fov_up
    self.proj_fov_down = fov_down
    self.use_ring_projection = use_ring_projection
    self.reset()

  def reset(self):
    """ Reset scan members. """
    self.points = np.zeros((0, 3), dtype=np.float32)        # [m, 3]: x, y, z
    self.remissions = np.zeros((0, 1), dtype=np.float32)    # [m ,1]: remission
    self.ring_index = np.zeros((0, 1), dtype=np.int32)      # [m ,1]: ring index

    # projected range image - [H,W] range (-1 is no data)
    self.proj_range = np.full((self.proj_H, self.proj_W), -1,
                              dtype=np.float32)

    # unprojected range (list of depths for each point)
    self.unproj_range = np.zeros((0, 1), dtype=np.float32)

    # projected point cloud xyz - [H,W,3] xyz coord (-1 is no data)
    self.proj_xyz = np.full((self.proj_H, self.proj_W, 3), -1,
                            dtype=np.float32)

    # projected remission - [H,W] intensity (-1 is no data)
    self.proj_remission = np.full((self.proj_H, self.proj_W), -1,
                                  dtype=np.float32)

    # projected index (for each pixel, what I am in the pointcloud)
    # [H,W] index (-1 is no data)
    self.proj_idx = np.full((self.proj_H, self.proj_W), -1,
                            dtype=np.int32)

    # for each point, where it is in the range image
    self.proj_x = np.zeros((0, 1), dtype=np.float32)        # [m, 1]: x
    self.proj_y = np.zeros((0, 1), dtype=np.float32)        # [m, 1]: y

    # mask containing for each pixel, if it contains a point or not
    self.proj_mask = np.zeros((self.proj_H, self.proj_W),
                              dtype=np.int32)       # [H,W] mask

  def size(self):
    """ Return the size of the point cloud. """
    return self.points.shape[0]

  def __len__(self):
    return self.size()

  def open_scan(self, filename):
    """ Open raw scan and fill in attributes
    """
    # reset just in case there was an open structure
    self.reset()

    # check filename is string
    if not isinstance(filename, str):
      raise TypeError("Filename should be string type, "
                      "but was {type}".format(type=str(type(filename))))

    # check extension is a laserscan
    if not any(filename.endswith(ext) for ext in self.EXTENSIONS_SCAN):
      raise RuntimeError("Filename extension is not valid scan file.")

    # if all goes well, open pointcloud
    pc = LidarSegPointCloud(filename) # open .bin file
    scan = pc.points                  # extract the points in scan 
    scan = scan.reshape((-1, 5))      # [N, 5], where N is the number of points

    # put in attribute
    points = scan[:, 0:3]    # get xyz
    remissions = scan[:, 3]  # get remission
    ring_index = scan[:, 4].astype(np.int32)  # get ring index
    self.set_points(points, remissions, ring_index)

  def set_points(self, points, remissions=None, ring_index=None):
    """ Set scan attributes (instead of opening from file)
    """
    # reset just in case there was an open structure
    self.reset()

    # check scan makes sense
    if not isinstance(points, np.ndarray):
      raise TypeError("Scan should be numpy array")

    # check remission makes sense
    if remissions is not None and not isinstance(remissions, np.ndarray):
      raise TypeError("Remissions should be numpy array")

    # check ring index makes sense
    if ring_index is not None and not isinstance(remissions, np.ndarray):
      raise TypeError("Remissions should be numpy array")

    # put in attribute
    self.points = points    # get xyz
    if remissions is not None:
      self.remissions = remissions  # get remission
    else:
      self.remissions = np.zeros((points.shape[0]), dtype=np.float32)

    if ring_index is not None:
      self.ring_index = ring_index
    else:
      self.ring_index = np.zeros((points.shape[0]), dtype=np.int32)

    # if projection is wanted, then do it and fill in the structure
    if self.project:
      if ring_index is not None and self.use_ring_projection:
        self.do_range_projection_ring()
      elif self.proj_fov_up is not None and self.proj_fov_down is not None and self.use_ring_projection is False:
        self.do_range_projection()
      else:
        raise NotImplementedError


  def do_range_projection_ring(self):
    """
    Range projection based on ring index. Does not use FOV UP - FOV DOWN
    """
    # get scan components
    scan_x = self.points[:, 0]
    scan_y = self.points[:, 1]
    scan_z = self.points[:, 2]

    ring_index = self.ring_index

    # get depth of all points
    depth = np.linalg.norm(self.points, 2, axis=1)

    # get angles of all points
    yaw = -np.arctan2(scan_y, scan_x)
    proj_x = 0.5 * (yaw / np.pi + 1.0)          # in [0.0, 1.0]

    proj_x *= self.proj_W                       # in [0.0, W]

    # round and clamp for use as index
    proj_x = np.floor(proj_x)
    proj_x = np.minimum(self.proj_W - 1, proj_x)
    proj_x = np.maximum(0, proj_x).astype(np.int32)   # in [0,W-1]
    self.proj_x = np.copy(proj_x)  # store a copy in orig order

    proj_y = (self.proj_H - 1) - ring_index

    self.proj_range[proj_y, proj_x] = depth
    self.proj_xyz[proj_y, proj_x] = self.points
    self.proj_remission[proj_y, proj_x] = self.remissions
    self.proj_idx[proj_y, proj_x] = np.arange(depth.shape[0])
    self.proj_mask = (self.proj_idx > 0).astype(np.float32)


  def do_range_projection(self):
    """ Project a pointcloud into a spherical projection image.projection.
        Function takes no arguments because it can be also called externally
        if the value of the constructor was not set (in case you change your
        mind about wanting the projection)
    """
    # laser parameters
    fov_up = self.proj_fov_up / 180.0 * np.pi      # field of view up in rad
    fov_down = self.proj_fov_down / 180.0 * np.pi  # field of view down in rad
    fov = abs(fov_down) + abs(fov_up)  # get field of view total in rad

    # get depth of all points
    depth = np.linalg.norm(self.points, 2, axis=1)

    # get scan components
    scan_x = self.points[:, 0]
    scan_y = self.points[:, 1]
    scan_z = self.points[:, 2]

    # get angles of all points
    yaw = -np.arctan2(scan_y, scan_x)
    pitch = np.arcsin(scan_z / depth)

    # get projections in image coords
    proj_x = 0.5 * (yaw / np.pi + 1.0)          # in [0.0, 1.0]
    proj_y = 1.0 - (pitch + abs(fov_down)) / fov        # in [0.0, 1.0]

    # scale to image size using angular resolution
    proj_x *= self.proj_W                              # in [0.0, W]
    proj_y *= self.proj_H                              # in [0.0, H]

    # round and clamp for use as index
    proj_x = np.floor(proj_x)
    proj_x = np.minimum(self.proj_W - 1, proj_x)
    proj_x = np.maximum(0, proj_x).astype(np.int32)   # in [0,W-1]
    self.proj_x = np.copy(proj_x)  # store a copy in orig order

    proj_y = np.floor(proj_y)
    proj_y = np.minimum(self.proj_H - 1, proj_y)
    proj_y = np.maximum(0, proj_y).astype(np.int32)   # in [0,H-1]
    self.proj_y = np.copy(proj_y)  # stope a copy in original order

    # copy of depth in original order
    self.unproj_range = np.copy(depth)

    # order in decreasing depth
    indices = np.arange(depth.shape[0])
    order = np.argsort(depth)[::-1]
    depth = depth[order]
    indices = indices[order]
    points = self.points[order]
    remission = self.remissions[order]
    proj_y = proj_y[order]
    proj_x = proj_x[order]

    # assing to images
    self.proj_range[proj_y, proj_x] = depth
    self.proj_xyz[proj_y, proj_x] = points
    self.proj_remission[proj_y, proj_x] = remission
    self.proj_idx[proj_y, proj_x] = indices
    self.proj_mask = (self.proj_idx > 0).astype(np.float32)


class SemLaserScan(LaserScan):
  """Class that contains LaserScan with x,y,z,r,sem_label,sem_color_label,inst_label,inst_color_label"""
  EXTENSIONS_LABEL = ['.bin']

  def __init__(self, nclasses, sem_color_dict=None, project=False, H=64, W=1024,
               fov_up=3.0, fov_down=-25.0, use_ring_projection=False):
    super(SemLaserScan, self).__init__(project, H, W, fov_up, fov_down, use_ring_projection)
    self.reset()
    self.nclasses = nclasses         # number of classes

    # make semantic colors
    max_sem_key = 0
    for key, data in sem_color_dict.items():
      if key + 1 > max_sem_key:
        max_sem_key = key + 1
    self.sem_color_lut = np.zeros((max_sem_key + 100, 3), dtype=np.float32)
    for key, value in sem_color_dict.items():
      self.sem_color_lut[key] = np.array(value, np.float32) / 255.0


  def reset(self):
    """ Reset scan members. """
    super(SemLaserScan, self).reset()

    # semantic labels
    self.sem_label = np.zeros((0, 1), dtype=np.uint32)         # [m, 1]: label
    self.sem_label_color = np.zeros((0, 3), dtype=np.float32)  # [m ,3]: color

    # projection color with semantic labels
    self.proj_sem_label = np.zeros((self.proj_H, self.proj_W),
                                   dtype=np.int32)              # [H,W]  label
    self.proj_sem_color = np.zeros((self.proj_H, self.proj_W, 3),
                                   dtype=np.float)              # [H,W,3] color

  def open_label(self, filename):
    """ Open raw scan and fill in attributes
    """
    # check filename is string
    if not isinstance(filename, str):
      raise TypeError("Filename should be string type, "
                      "but was {type}".format(type=str(type(filename))))

    # check extension is a laserscan
    if not any(filename.endswith(ext) for ext in self.EXTENSIONS_LABEL):
      raise RuntimeError("Filename extension is not valid label file.")

    # if all goes well, open label
    label = load_bin_file(filename) # loads a .bin file containing the lidarseg
    label = label.reshape((-1))

    # set it
    self.set_label(label)

  def set_label(self, label):
    """ Set points for label not from file but from np
    """
    # check label makes sense
    if not isinstance(label, np.ndarray):
      raise TypeError("Label should be numpy array")

    # only fill in attribute if the right size
    if label.shape[0] == self.points.shape[0]:
      self.sem_label = label 
    else:
      print("Points shape: ", self.points.shape)
      print("Label shape: ", label.shape)
      raise ValueError("Scan and Label don't contain same number of points")

    if self.project:
      self.do_label_projection()

  def colorize(self):
    """ Colorize pointcloud with the color of each semantic label
    """
    self.sem_label_color = self.sem_color_lut[self.sem_label]
    self.sem_label_color = self.sem_label_color.reshape((-1, 3))

  def do_label_projection(self):
    # only map colors to labels that exist
    mask = self.proj_idx >= 0

    # semantics
    self.proj_sem_label[mask] = self.sem_label[self.proj_idx[mask]]
    self.proj_sem_color[mask] = self.sem_color_lut[self.sem_label[self.proj_idx[mask]]]

