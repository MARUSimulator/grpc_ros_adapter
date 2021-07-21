# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: acoustic_transmission.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='acoustic_transmission.proto',
  package='acoustic_transmission',
  syntax='proto3',
  serialized_options=_b('\n\033io.grpc.examples.navigationB\nNavigationP\001\242\002\003HLW'),
  serialized_pb=_b('\n\x1b\x61\x63oustic_transmission.proto\x12\x15\x61\x63oustic_transmission\"!\n\x0e\x43ommandRequest\x12\x0f\n\x07\x61\x64\x64ress\x18\x01 \x01(\t\"\x92\x01\n\x0f\x41\x63ousticPayload\x12\x0f\n\x07\x61\x64\x64ress\x18\x01 \x01(\t\x12\x38\n\x07payload\x18\x02 \x01(\x0b\x32\'.acoustic_transmission.NanomodemPayload\x12\x34\n\x05range\x18\x03 \x01(\x0b\x32%.acoustic_transmission.NanomodemRange\"]\n\x10\x41\x63ousticResponse\x12\x0f\n\x07success\x18\x01 \x01(\x08\x12\x38\n\x07request\x18\x02 \x01(\x0b\x32\'.acoustic_transmission.NanomodemRequest\"\x84\x02\n\x10NanomodemRequest\x12>\n\x08req_type\x18\x01 \x01(\x0e\x32,.acoustic_transmission.NanomodemRequest.Type\x12\x11\n\tscheduled\x18\x02 \x01(\x08\x12\x0b\n\x03msg\x18\x03 \x01(\t\x12\n\n\x02id\x18\x04 \x01(\r\"\x83\x01\n\x04Type\x12\n\n\x06\x42RDCST\x10\x00\x12\n\n\x06UNICST\x10\x01\x12\n\n\x06PINGID\x10\x02\x12\n\n\x06VOLTID\x10\x03\x12\n\n\x06\x43HNGID\x10\x04\x12\n\n\x06STATUS\x10\x05\x12\x0c\n\x08UNISTACK\x10\x06\x12\x0b\n\x07TESTMSG\x10\x07\x12\x0b\n\x07\x45\x43HOMSG\x10\x08\x12\x0b\n\x07QUALITY\x10\t\"\x92\x01\n\x10NanomodemPayload\x12>\n\x08msg_type\x18\x01 \x01(\x0e\x32,.acoustic_transmission.NanomodemPayload.Type\x12\x0b\n\x03msg\x18\x02 \x01(\t\x12\x11\n\tsender_id\x18\x03 \x01(\r\"\x1e\n\x04Type\x12\n\n\x06\x42RDCST\x10\x00\x12\n\n\x06UNICST\x10\x01\"<\n\x0eNanomodemRange\x12\r\n\x05range\x18\x01 \x01(\r\x12\x0f\n\x07range_m\x18\x02 \x01(\x02\x12\n\n\x02id\x18\x03 \x01(\r2\xe3\x01\n\tAcoustics\x12h\n\x12GetAcousticRequest\x12%.acoustic_transmission.CommandRequest\x1a\'.acoustic_transmission.AcousticResponse\"\x00\x30\x01\x12l\n\x15ReturnAcousticPayload\x12&.acoustic_transmission.AcousticPayload\x1a\'.acoustic_transmission.AcousticResponse\"\x00\x30\x01\x42\x31\n\x1bio.grpc.examples.navigationB\nNavigationP\x01\xa2\x02\x03HLWb\x06proto3')
)



_NANOMODEMREQUEST_TYPE = _descriptor.EnumDescriptor(
  name='Type',
  full_name='acoustic_transmission.NanomodemRequest.Type',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='BRDCST', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='UNICST', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PINGID', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='VOLTID', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='CHNGID', index=4, number=4,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='STATUS', index=5, number=5,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='UNISTACK', index=6, number=6,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TESTMSG', index=7, number=7,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ECHOMSG', index=8, number=8,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='QUALITY', index=9, number=9,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=463,
  serialized_end=594,
)
_sym_db.RegisterEnumDescriptor(_NANOMODEMREQUEST_TYPE)

_NANOMODEMPAYLOAD_TYPE = _descriptor.EnumDescriptor(
  name='Type',
  full_name='acoustic_transmission.NanomodemPayload.Type',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='BRDCST', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='UNICST', index=1, number=1,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=463,
  serialized_end=493,
)
_sym_db.RegisterEnumDescriptor(_NANOMODEMPAYLOAD_TYPE)


_COMMANDREQUEST = _descriptor.Descriptor(
  name='CommandRequest',
  full_name='acoustic_transmission.CommandRequest',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='address', full_name='acoustic_transmission.CommandRequest.address', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=54,
  serialized_end=87,
)


_ACOUSTICPAYLOAD = _descriptor.Descriptor(
  name='AcousticPayload',
  full_name='acoustic_transmission.AcousticPayload',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='address', full_name='acoustic_transmission.AcousticPayload.address', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='payload', full_name='acoustic_transmission.AcousticPayload.payload', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='range', full_name='acoustic_transmission.AcousticPayload.range', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=90,
  serialized_end=236,
)


_ACOUSTICRESPONSE = _descriptor.Descriptor(
  name='AcousticResponse',
  full_name='acoustic_transmission.AcousticResponse',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='success', full_name='acoustic_transmission.AcousticResponse.success', index=0,
      number=1, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='request', full_name='acoustic_transmission.AcousticResponse.request', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=238,
  serialized_end=331,
)


_NANOMODEMREQUEST = _descriptor.Descriptor(
  name='NanomodemRequest',
  full_name='acoustic_transmission.NanomodemRequest',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='req_type', full_name='acoustic_transmission.NanomodemRequest.req_type', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='scheduled', full_name='acoustic_transmission.NanomodemRequest.scheduled', index=1,
      number=2, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='msg', full_name='acoustic_transmission.NanomodemRequest.msg', index=2,
      number=3, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='id', full_name='acoustic_transmission.NanomodemRequest.id', index=3,
      number=4, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _NANOMODEMREQUEST_TYPE,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=334,
  serialized_end=594,
)


_NANOMODEMPAYLOAD = _descriptor.Descriptor(
  name='NanomodemPayload',
  full_name='acoustic_transmission.NanomodemPayload',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='msg_type', full_name='acoustic_transmission.NanomodemPayload.msg_type', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='msg', full_name='acoustic_transmission.NanomodemPayload.msg', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sender_id', full_name='acoustic_transmission.NanomodemPayload.sender_id', index=2,
      number=3, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _NANOMODEMPAYLOAD_TYPE,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=597,
  serialized_end=743,
)


_NANOMODEMRANGE = _descriptor.Descriptor(
  name='NanomodemRange',
  full_name='acoustic_transmission.NanomodemRange',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='range', full_name='acoustic_transmission.NanomodemRange.range', index=0,
      number=1, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='range_m', full_name='acoustic_transmission.NanomodemRange.range_m', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='id', full_name='acoustic_transmission.NanomodemRange.id', index=2,
      number=3, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=745,
  serialized_end=805,
)

_ACOUSTICPAYLOAD.fields_by_name['payload'].message_type = _NANOMODEMPAYLOAD
_ACOUSTICPAYLOAD.fields_by_name['range'].message_type = _NANOMODEMRANGE
_ACOUSTICRESPONSE.fields_by_name['request'].message_type = _NANOMODEMREQUEST
_NANOMODEMREQUEST.fields_by_name['req_type'].enum_type = _NANOMODEMREQUEST_TYPE
_NANOMODEMREQUEST_TYPE.containing_type = _NANOMODEMREQUEST
_NANOMODEMPAYLOAD.fields_by_name['msg_type'].enum_type = _NANOMODEMPAYLOAD_TYPE
_NANOMODEMPAYLOAD_TYPE.containing_type = _NANOMODEMPAYLOAD
DESCRIPTOR.message_types_by_name['CommandRequest'] = _COMMANDREQUEST
DESCRIPTOR.message_types_by_name['AcousticPayload'] = _ACOUSTICPAYLOAD
DESCRIPTOR.message_types_by_name['AcousticResponse'] = _ACOUSTICRESPONSE
DESCRIPTOR.message_types_by_name['NanomodemRequest'] = _NANOMODEMREQUEST
DESCRIPTOR.message_types_by_name['NanomodemPayload'] = _NANOMODEMPAYLOAD
DESCRIPTOR.message_types_by_name['NanomodemRange'] = _NANOMODEMRANGE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

CommandRequest = _reflection.GeneratedProtocolMessageType('CommandRequest', (_message.Message,), dict(
  DESCRIPTOR = _COMMANDREQUEST,
  __module__ = 'acoustic_transmission_pb2'
  # @@protoc_insertion_point(class_scope:acoustic_transmission.CommandRequest)
  ))
_sym_db.RegisterMessage(CommandRequest)

AcousticPayload = _reflection.GeneratedProtocolMessageType('AcousticPayload', (_message.Message,), dict(
  DESCRIPTOR = _ACOUSTICPAYLOAD,
  __module__ = 'acoustic_transmission_pb2'
  # @@protoc_insertion_point(class_scope:acoustic_transmission.AcousticPayload)
  ))
_sym_db.RegisterMessage(AcousticPayload)

AcousticResponse = _reflection.GeneratedProtocolMessageType('AcousticResponse', (_message.Message,), dict(
  DESCRIPTOR = _ACOUSTICRESPONSE,
  __module__ = 'acoustic_transmission_pb2'
  # @@protoc_insertion_point(class_scope:acoustic_transmission.AcousticResponse)
  ))
_sym_db.RegisterMessage(AcousticResponse)

NanomodemRequest = _reflection.GeneratedProtocolMessageType('NanomodemRequest', (_message.Message,), dict(
  DESCRIPTOR = _NANOMODEMREQUEST,
  __module__ = 'acoustic_transmission_pb2'
  # @@protoc_insertion_point(class_scope:acoustic_transmission.NanomodemRequest)
  ))
_sym_db.RegisterMessage(NanomodemRequest)

NanomodemPayload = _reflection.GeneratedProtocolMessageType('NanomodemPayload', (_message.Message,), dict(
  DESCRIPTOR = _NANOMODEMPAYLOAD,
  __module__ = 'acoustic_transmission_pb2'
  # @@protoc_insertion_point(class_scope:acoustic_transmission.NanomodemPayload)
  ))
_sym_db.RegisterMessage(NanomodemPayload)

NanomodemRange = _reflection.GeneratedProtocolMessageType('NanomodemRange', (_message.Message,), dict(
  DESCRIPTOR = _NANOMODEMRANGE,
  __module__ = 'acoustic_transmission_pb2'
  # @@protoc_insertion_point(class_scope:acoustic_transmission.NanomodemRange)
  ))
_sym_db.RegisterMessage(NanomodemRange)


DESCRIPTOR._options = None

_ACOUSTICS = _descriptor.ServiceDescriptor(
  name='Acoustics',
  full_name='acoustic_transmission.Acoustics',
  file=DESCRIPTOR,
  index=0,
  serialized_options=None,
  serialized_start=808,
  serialized_end=1035,
  methods=[
  _descriptor.MethodDescriptor(
    name='GetAcousticRequest',
    full_name='acoustic_transmission.Acoustics.GetAcousticRequest',
    index=0,
    containing_service=None,
    input_type=_COMMANDREQUEST,
    output_type=_ACOUSTICRESPONSE,
    serialized_options=None,
  ),
  _descriptor.MethodDescriptor(
    name='ReturnAcousticPayload',
    full_name='acoustic_transmission.Acoustics.ReturnAcousticPayload',
    index=1,
    containing_service=None,
    input_type=_ACOUSTICPAYLOAD,
    output_type=_ACOUSTICRESPONSE,
    serialized_options=None,
  ),
])
_sym_db.RegisterServiceDescriptor(_ACOUSTICS)

DESCRIPTOR.services_by_name['Acoustics'] = _ACOUSTICS

# @@protoc_insertion_point(module_scope)
