# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: cartographer/mapping/proto/range_data_inserter_options.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from cartographer.mapping.proto import probability_grid_range_data_inserter_options_2d_pb2 as cartographer_dot_mapping_dot_proto_dot_probability__grid__range__data__inserter__options__2d__pb2
from cartographer.mapping.proto import tsdf_range_data_inserter_options_2d_pb2 as cartographer_dot_mapping_dot_proto_dot_tsdf__range__data__inserter__options__2d__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n<cartographer/mapping/proto/range_data_inserter_options.proto\x12\x1a\x63\x61rtographer.mapping.proto\x1aPcartographer/mapping/proto/probability_grid_range_data_inserter_options_2d.proto\x1a\x44\x63\x61rtographer/mapping/proto/tsdf_range_data_inserter_options_2d.proto\"\xd8\x03\n\x18RangeDataInserterOptions\x12l\n\x18range_data_inserter_type\x18\x01 \x01(\x0e\x32J.cartographer.mapping.proto.RangeDataInserterOptions.RangeDataInserterType\x12~\n/probability_grid_range_data_inserter_options_2d\x18\x02 \x01(\x0b\x32\x45.cartographer.mapping.proto.ProbabilityGridRangeDataInserterOptions2D\x12g\n#tsdf_range_data_inserter_options_2d\x18\x03 \x01(\x0b\x32:.cartographer.mapping.proto.TSDFRangeDataInserterOptions2D\"e\n\x15RangeDataInserterType\x12\x14\n\x10INVALID_INSERTER\x10\x00\x12 \n\x1cPROBABILITY_GRID_INSERTER_2D\x10\x01\x12\x14\n\x10TSDF_INSERTER_2D\x10\x02\x62\x06proto3')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'cartographer.mapping.proto.range_data_inserter_options_pb2', _globals)
if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _globals['_RANGEDATAINSERTEROPTIONS']._serialized_start=245
  _globals['_RANGEDATAINSERTEROPTIONS']._serialized_end=717
  _globals['_RANGEDATAINSERTEROPTIONS_RANGEDATAINSERTERTYPE']._serialized_start=616
  _globals['_RANGEDATAINSERTEROPTIONS_RANGEDATAINSERTERTYPE']._serialized_end=717
# @@protoc_insertion_point(module_scope)
