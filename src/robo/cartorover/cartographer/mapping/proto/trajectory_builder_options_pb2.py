# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: cartographer/mapping/proto/trajectory_builder_options.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from cartographer.transform.proto import transform_pb2 as cartographer_dot_transform_dot_proto_dot_transform__pb2
from cartographer.mapping.proto import motion_filter_options_pb2 as cartographer_dot_mapping_dot_proto_dot_motion__filter__options__pb2
from cartographer.mapping.proto import local_trajectory_builder_options_2d_pb2 as cartographer_dot_mapping_dot_proto_dot_local__trajectory__builder__options__2d__pb2
from cartographer.mapping.proto import local_trajectory_builder_options_3d_pb2 as cartographer_dot_mapping_dot_proto_dot_local__trajectory__builder__options__3d__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n;cartographer/mapping/proto/trajectory_builder_options.proto\x12\x1a\x63\x61rtographer.mapping.proto\x1a,cartographer/transform/proto/transform.proto\x1a\x36\x63\x61rtographer/mapping/proto/motion_filter_options.proto\x1a\x44\x63\x61rtographer/mapping/proto/local_trajectory_builder_options_2d.proto\x1a\x44\x63\x61rtographer/mapping/proto/local_trajectory_builder_options_3d.proto\"\x82\x01\n\x15InitialTrajectoryPose\x12<\n\rrelative_pose\x18\x01 \x01(\x0b\x32%.cartographer.transform.proto.Rigid3d\x12\x18\n\x10to_trajectory_id\x18\x02 \x01(\x05\x12\x11\n\ttimestamp\x18\x03 \x01(\x03\"\xa6\x05\n\x18TrajectoryBuilderOptions\x12\x62\n\x1dtrajectory_builder_2d_options\x18\x01 \x01(\x0b\x32;.cartographer.mapping.proto.LocalTrajectoryBuilderOptions2D\x12\x62\n\x1dtrajectory_builder_3d_options\x18\x02 \x01(\x0b\x32;.cartographer.mapping.proto.LocalTrajectoryBuilderOptions3D\x12R\n\x17initial_trajectory_pose\x18\x04 \x01(\x0b\x32\x31.cartographer.mapping.proto.InitialTrajectoryPose\x12\x1d\n\x11pure_localization\x18\x03 \x01(\x08\x42\x02\x18\x01\x12v\n\x19pure_localization_trimmer\x18\x06 \x01(\x0b\x32S.cartographer.mapping.proto.TrajectoryBuilderOptions.PureLocalizationTrimmerOptions\x12\x1b\n\x13\x63ollate_fixed_frame\x18\x07 \x01(\x08\x12\x19\n\x11\x63ollate_landmarks\x18\x08 \x01(\x08\x12Z\n!pose_graph_odometry_motion_filter\x18\t \x01(\x0b\x32/.cartographer.mapping.proto.MotionFilterOptions\x1a=\n\x1ePureLocalizationTrimmerOptions\x12\x1b\n\x13max_submaps_to_keep\x18\x01 \x01(\x05J\x04\x08\x05\x10\x06\"\xc0\x01\n\x08SensorId\x12=\n\x04type\x18\x01 \x01(\x0e\x32/.cartographer.mapping.proto.SensorId.SensorType\x12\n\n\x02id\x18\x02 \x01(\t\"i\n\nSensorType\x12\t\n\x05RANGE\x10\x00\x12\x07\n\x03IMU\x10\x01\x12\x0c\n\x08ODOMETRY\x10\x02\x12\x14\n\x10\x46IXED_FRAME_POSE\x10\x03\x12\x0c\n\x08LANDMARK\x10\x04\x12\x15\n\x11LOCAL_SLAM_RESULT\x10\x05\"\xba\x01\n%TrajectoryBuilderOptionsWithSensorIds\x12\x37\n\tsensor_id\x18\x01 \x03(\x0b\x32$.cartographer.mapping.proto.SensorId\x12X\n\x1atrajectory_builder_options\x18\x02 \x01(\x0b\x32\x34.cartographer.mapping.proto.TrajectoryBuilderOptions\"\x81\x01\n\x1b\x41llTrajectoryBuilderOptions\x12\x62\n\x17options_with_sensor_ids\x18\x01 \x03(\x0b\x32\x41.cartographer.mapping.proto.TrajectoryBuilderOptionsWithSensorIdsb\x06proto3')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'cartographer.mapping.proto.trajectory_builder_options_pb2', _globals)
if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _TRAJECTORYBUILDEROPTIONS.fields_by_name['pure_localization']._options = None
  _TRAJECTORYBUILDEROPTIONS.fields_by_name['pure_localization']._serialized_options = b'\030\001'
  _globals['_INITIALTRAJECTORYPOSE']._serialized_start=334
  _globals['_INITIALTRAJECTORYPOSE']._serialized_end=464
  _globals['_TRAJECTORYBUILDEROPTIONS']._serialized_start=467
  _globals['_TRAJECTORYBUILDEROPTIONS']._serialized_end=1145
  _globals['_TRAJECTORYBUILDEROPTIONS_PURELOCALIZATIONTRIMMEROPTIONS']._serialized_start=1078
  _globals['_TRAJECTORYBUILDEROPTIONS_PURELOCALIZATIONTRIMMEROPTIONS']._serialized_end=1139
  _globals['_SENSORID']._serialized_start=1148
  _globals['_SENSORID']._serialized_end=1340
  _globals['_SENSORID_SENSORTYPE']._serialized_start=1235
  _globals['_SENSORID_SENSORTYPE']._serialized_end=1340
  _globals['_TRAJECTORYBUILDEROPTIONSWITHSENSORIDS']._serialized_start=1343
  _globals['_TRAJECTORYBUILDEROPTIONSWITHSENSORIDS']._serialized_end=1529
  _globals['_ALLTRAJECTORYBUILDEROPTIONS']._serialized_start=1532
  _globals['_ALLTRAJECTORYBUILDEROPTIONS']._serialized_end=1661
# @@protoc_insertion_point(module_scope)