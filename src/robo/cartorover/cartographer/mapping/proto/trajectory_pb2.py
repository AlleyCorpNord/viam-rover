# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: cartographer/mapping/proto/trajectory.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from cartographer.transform.proto import transform_pb2 as cartographer_dot_transform_dot_proto_dot_transform__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n+cartographer/mapping/proto/trajectory.proto\x12\x1a\x63\x61rtographer.mapping.proto\x1a,cartographer/transform/proto/transform.proto\"\xd6\x02\n\nTrajectory\x12\x15\n\rtrajectory_id\x18\x03 \x01(\x05\x12\x39\n\x04node\x18\x01 \x03(\x0b\x32+.cartographer.mapping.proto.Trajectory.Node\x12=\n\x06submap\x18\x02 \x03(\x0b\x32-.cartographer.mapping.proto.Trajectory.Submap\x1a\x62\n\x04Node\x12\x12\n\nnode_index\x18\x07 \x01(\x05\x12\x11\n\ttimestamp\x18\x01 \x01(\x03\x12\x33\n\x04pose\x18\x05 \x01(\x0b\x32%.cartographer.transform.proto.Rigid3d\x1aS\n\x06Submap\x12\x14\n\x0csubmap_index\x18\x02 \x01(\x05\x12\x33\n\x04pose\x18\x01 \x01(\x0b\x32%.cartographer.transform.proto.Rigid3dB\x16\x42\x14TrajectoryOuterClassb\x06proto3')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'cartographer.mapping.proto.trajectory_pb2', _globals)
if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  DESCRIPTOR._serialized_options = b'B\024TrajectoryOuterClass'
  _globals['_TRAJECTORY']._serialized_start=122
  _globals['_TRAJECTORY']._serialized_end=464
  _globals['_TRAJECTORY_NODE']._serialized_start=281
  _globals['_TRAJECTORY_NODE']._serialized_end=379
  _globals['_TRAJECTORY_SUBMAP']._serialized_start=381
  _globals['_TRAJECTORY_SUBMAP']._serialized_end=464
# @@protoc_insertion_point(module_scope)
