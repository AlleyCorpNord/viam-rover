# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: cartographer/mapping/proto/submaps_options_2d.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from cartographer.mapping.proto import grid_2d_options_pb2 as cartographer_dot_mapping_dot_proto_dot_grid__2d__options__pb2
from cartographer.mapping.proto import range_data_inserter_options_pb2 as cartographer_dot_mapping_dot_proto_dot_range__data__inserter__options__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n3cartographer/mapping/proto/submaps_options_2d.proto\x12\x1a\x63\x61rtographer.mapping.proto\x1a\x30\x63\x61rtographer/mapping/proto/grid_2d_options.proto\x1a<cartographer/mapping/proto/range_data_inserter_options.proto\"\xc9\x01\n\x10SubmapsOptions2D\x12\x16\n\x0enum_range_data\x18\x01 \x01(\x05\x12\x42\n\x0fgrid_options_2d\x18\x02 \x01(\x0b\x32).cartographer.mapping.proto.GridOptions2D\x12Y\n\x1brange_data_inserter_options\x18\x03 \x01(\x0b\x32\x34.cartographer.mapping.proto.RangeDataInserterOptionsb\x06proto3')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'cartographer.mapping.proto.submaps_options_2d_pb2', _globals)
if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _globals['_SUBMAPSOPTIONS2D']._serialized_start=196
  _globals['_SUBMAPSOPTIONS2D']._serialized_end=397
# @@protoc_insertion_point(module_scope)
