# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: cartographer/mapping/proto/local_trajectory_builder_options_3d.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from cartographer.mapping.proto import motion_filter_options_pb2 as cartographer_dot_mapping_dot_proto_dot_motion__filter__options__pb2
from cartographer.mapping.proto import pose_extrapolator_options_pb2 as cartographer_dot_mapping_dot_proto_dot_pose__extrapolator__options__pb2
from cartographer.mapping.proto.scan_matching import ceres_scan_matcher_options_3d_pb2 as cartographer_dot_mapping_dot_proto_dot_scan__matching_dot_ceres__scan__matcher__options__3d__pb2
from cartographer.mapping.proto.scan_matching import real_time_correlative_scan_matcher_options_pb2 as cartographer_dot_mapping_dot_proto_dot_scan__matching_dot_real__time__correlative__scan__matcher__options__pb2
from cartographer.mapping.proto import submaps_options_3d_pb2 as cartographer_dot_mapping_dot_proto_dot_submaps__options__3d__pb2
from cartographer.sensor.proto import adaptive_voxel_filter_options_pb2 as cartographer_dot_sensor_dot_proto_dot_adaptive__voxel__filter__options__pb2
from cartographer.sensor.proto import sensor_pb2 as cartographer_dot_sensor_dot_proto_dot_sensor__pb2
from cartographer.transform.proto import timestamped_transform_pb2 as cartographer_dot_transform_dot_proto_dot_timestamped__transform__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\nDcartographer/mapping/proto/local_trajectory_builder_options_3d.proto\x12\x1a\x63\x61rtographer.mapping.proto\x1a\x36\x63\x61rtographer/mapping/proto/motion_filter_options.proto\x1a:cartographer/mapping/proto/pose_extrapolator_options.proto\x1aLcartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_3d.proto\x1aYcartographer/mapping/proto/scan_matching/real_time_correlative_scan_matcher_options.proto\x1a\x33\x63\x61rtographer/mapping/proto/submaps_options_3d.proto\x1a=cartographer/sensor/proto/adaptive_voxel_filter_options.proto\x1a&cartographer/sensor/proto/sensor.proto\x1a\x38\x63\x61rtographer/transform/proto/timestamped_transform.proto\"\xd5\x08\n\x1fLocalTrajectoryBuilderOptions3D\x12\x11\n\tmin_range\x18\x01 \x01(\x02\x12\x11\n\tmax_range\x18\x02 \x01(\x02\x12\"\n\x1anum_accumulated_range_data\x18\x03 \x01(\x05\x12\x19\n\x11voxel_filter_size\x18\x04 \x01(\x02\x12l\n-high_resolution_adaptive_voxel_filter_options\x18\x05 \x01(\x0b\x32\x35.cartographer.sensor.proto.AdaptiveVoxelFilterOptions\x12k\n,low_resolution_adaptive_voxel_filter_options\x18\x0c \x01(\x0b\x32\x35.cartographer.sensor.proto.AdaptiveVoxelFilterOptions\x12,\n$use_online_correlative_scan_matching\x18\r \x01(\x08\x12\x83\x01\n*real_time_correlative_scan_matcher_options\x18\x0e \x01(\x0b\x32O.cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions\x12g\n\x1a\x63\x65res_scan_matcher_options\x18\x06 \x01(\x0b\x32\x43.cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions3D\x12N\n\x15motion_filter_options\x18\x07 \x01(\x0b\x32/.cartographer.mapping.proto.MotionFilterOptions\x12!\n\x19imu_gravity_time_constant\x18\x0f \x01(\x01\x12!\n\x19rotational_histogram_size\x18\x11 \x01(\x05\x12V\n\x19pose_extrapolator_options\x18\x12 \x01(\x0b\x32\x33.cartographer.mapping.proto.PoseExtrapolatorOptions\x12I\n\rinitial_poses\x18\x13 \x03(\x0b\x32\x32.cartographer.transform.proto.TimestampedTransform\x12<\n\x10initial_imu_data\x18\x14 \x03(\x0b\x32\".cartographer.sensor.proto.ImuData\x12\x45\n\x0fsubmaps_options\x18\x08 \x01(\x0b\x32,.cartographer.mapping.proto.SubmapsOptions3D\x12\x17\n\x0fuse_intensities\x18\x15 \x01(\x08\x62\x06proto3')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'cartographer.mapping.proto.local_trajectory_builder_options_3d_pb2', _globals)
if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _globals['_LOCALTRAJECTORYBUILDEROPTIONS3D']._serialized_start=600
  _globals['_LOCALTRAJECTORYBUILDEROPTIONS3D']._serialized_end=1709
# @@protoc_insertion_point(module_scope)