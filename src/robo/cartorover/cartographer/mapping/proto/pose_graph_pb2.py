# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: cartographer/mapping/proto/pose_graph.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from cartographer.mapping.proto import trajectory_pb2 as cartographer_dot_mapping_dot_proto_dot_trajectory__pb2
from cartographer.transform.proto import transform_pb2 as cartographer_dot_transform_dot_proto_dot_transform__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n+cartographer/mapping/proto/pose_graph.proto\x12\x1a\x63\x61rtographer.mapping.proto\x1a+cartographer/mapping/proto/trajectory.proto\x1a,cartographer/transform/proto/transform.proto\"7\n\x08SubmapId\x12\x15\n\rtrajectory_id\x18\x01 \x01(\x05\x12\x14\n\x0csubmap_index\x18\x02 \x01(\x05\"3\n\x06NodeId\x12\x15\n\rtrajectory_id\x18\x01 \x01(\x05\x12\x12\n\nnode_index\x18\x02 \x01(\x05\"\x98\x05\n\tPoseGraph\x12\x44\n\nconstraint\x18\x02 \x03(\x0b\x32\x30.cartographer.mapping.proto.PoseGraph.Constraint\x12:\n\ntrajectory\x18\x04 \x03(\x0b\x32&.cartographer.mapping.proto.Trajectory\x12J\n\x0elandmark_poses\x18\x05 \x03(\x0b\x32\x32.cartographer.mapping.proto.PoseGraph.LandmarkPose\x1a\xdb\x02\n\nConstraint\x12\x37\n\tsubmap_id\x18\x01 \x01(\x0b\x32$.cartographer.mapping.proto.SubmapId\x12\x33\n\x07node_id\x18\x02 \x01(\x0b\x32\".cartographer.mapping.proto.NodeId\x12<\n\rrelative_pose\x18\x03 \x01(\x0b\x32%.cartographer.transform.proto.Rigid3d\x12\x1a\n\x12translation_weight\x18\x06 \x01(\x01\x12\x17\n\x0frotation_weight\x18\x07 \x01(\x01\x12\x41\n\x03tag\x18\x05 \x01(\x0e\x32\x34.cartographer.mapping.proto.PoseGraph.Constraint.Tag\")\n\x03Tag\x12\x10\n\x0cINTRA_SUBMAP\x10\x00\x12\x10\n\x0cINTER_SUBMAP\x10\x01\x1a_\n\x0cLandmarkPose\x12\x13\n\x0blandmark_id\x18\x01 \x01(\t\x12:\n\x0bglobal_pose\x18\x02 \x01(\x0b\x32%.cartographer.transform.proto.Rigid3db\x06proto3')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'cartographer.mapping.proto.pose_graph_pb2', _globals)
if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _globals['_SUBMAPID']._serialized_start=166
  _globals['_SUBMAPID']._serialized_end=221
  _globals['_NODEID']._serialized_start=223
  _globals['_NODEID']._serialized_end=274
  _globals['_POSEGRAPH']._serialized_start=277
  _globals['_POSEGRAPH']._serialized_end=941
  _globals['_POSEGRAPH_CONSTRAINT']._serialized_start=497
  _globals['_POSEGRAPH_CONSTRAINT']._serialized_end=844
  _globals['_POSEGRAPH_CONSTRAINT_TAG']._serialized_start=803
  _globals['_POSEGRAPH_CONSTRAINT_TAG']._serialized_end=844
  _globals['_POSEGRAPH_LANDMARKPOSE']._serialized_start=846
  _globals['_POSEGRAPH_LANDMARKPOSE']._serialized_end=941
# @@protoc_insertion_point(module_scope)
