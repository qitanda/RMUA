// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: onnx/onnx-data_onnx2trt_onnx.proto

#ifndef PROTOBUF_INCLUDED_onnx_2fonnx_2ddata_5fonnx2trt_5fonnx_2eproto
#define PROTOBUF_INCLUDED_onnx_2fonnx_2ddata_5fonnx2trt_5fonnx_2eproto

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3006001
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3006001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/inlined_string_field.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
#include "onnx/onnx_onnx2trt_onnx-ml.pb.h"
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_onnx_2fonnx_2ddata_5fonnx2trt_5fonnx_2eproto ONNX_API

namespace protobuf_onnx_2fonnx_2ddata_5fonnx2trt_5fonnx_2eproto {
// Internal implementation detail -- do not use these members.
struct ONNX_API TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[2];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void ONNX_API AddDescriptors();
}  // namespace protobuf_onnx_2fonnx_2ddata_5fonnx2trt_5fonnx_2eproto
namespace onnx2trt_onnx {
class MapProto;
class MapProtoDefaultTypeInternal;
ONNX_API extern MapProtoDefaultTypeInternal _MapProto_default_instance_;
class SequenceProto;
class SequenceProtoDefaultTypeInternal;
ONNX_API extern SequenceProtoDefaultTypeInternal _SequenceProto_default_instance_;
}  // namespace onnx2trt_onnx
namespace google {
namespace protobuf {
template<> ONNX_API ::onnx2trt_onnx::MapProto* Arena::CreateMaybeMessage<::onnx2trt_onnx::MapProto>(Arena*);
template<> ONNX_API ::onnx2trt_onnx::SequenceProto* Arena::CreateMaybeMessage<::onnx2trt_onnx::SequenceProto>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace onnx2trt_onnx {

enum SequenceProto_DataType {
  SequenceProto_DataType_UNDEFINED = 0,
  SequenceProto_DataType_TENSOR = 1,
  SequenceProto_DataType_SPARSE_TENSOR = 2,
  SequenceProto_DataType_SEQUENCE = 3,
  SequenceProto_DataType_MAP = 4
};
ONNX_API bool SequenceProto_DataType_IsValid(int value);
const SequenceProto_DataType SequenceProto_DataType_DataType_MIN = SequenceProto_DataType_UNDEFINED;
const SequenceProto_DataType SequenceProto_DataType_DataType_MAX = SequenceProto_DataType_MAP;
const int SequenceProto_DataType_DataType_ARRAYSIZE = SequenceProto_DataType_DataType_MAX + 1;

ONNX_API const ::google::protobuf::EnumDescriptor* SequenceProto_DataType_descriptor();
inline const ::std::string& SequenceProto_DataType_Name(SequenceProto_DataType value) {
  return ::google::protobuf::internal::NameOfEnum(
    SequenceProto_DataType_descriptor(), value);
}
inline bool SequenceProto_DataType_Parse(
    const ::std::string& name, SequenceProto_DataType* value) {
  return ::google::protobuf::internal::ParseNamedEnum<SequenceProto_DataType>(
    SequenceProto_DataType_descriptor(), name, value);
}
// ===================================================================

class ONNX_API SequenceProto : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:onnx2trt_onnx.SequenceProto) */ {
 public:
  SequenceProto();
  virtual ~SequenceProto();

  SequenceProto(const SequenceProto& from);

  inline SequenceProto& operator=(const SequenceProto& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  SequenceProto(SequenceProto&& from) noexcept
    : SequenceProto() {
    *this = ::std::move(from);
  }

  inline SequenceProto& operator=(SequenceProto&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const SequenceProto& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const SequenceProto* internal_default_instance() {
    return reinterpret_cast<const SequenceProto*>(
               &_SequenceProto_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(SequenceProto* other);
  friend void swap(SequenceProto& a, SequenceProto& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline SequenceProto* New() const final {
    return CreateMaybeMessage<SequenceProto>(NULL);
  }

  SequenceProto* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<SequenceProto>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const SequenceProto& from);
  void MergeFrom(const SequenceProto& from);
  void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(SequenceProto* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  typedef SequenceProto_DataType DataType;
  static const DataType UNDEFINED =
    SequenceProto_DataType_UNDEFINED;
  static const DataType TENSOR =
    SequenceProto_DataType_TENSOR;
  static const DataType SPARSE_TENSOR =
    SequenceProto_DataType_SPARSE_TENSOR;
  static const DataType SEQUENCE =
    SequenceProto_DataType_SEQUENCE;
  static const DataType MAP =
    SequenceProto_DataType_MAP;
  static inline bool DataType_IsValid(int value) {
    return SequenceProto_DataType_IsValid(value);
  }
  static const DataType DataType_MIN =
    SequenceProto_DataType_DataType_MIN;
  static const DataType DataType_MAX =
    SequenceProto_DataType_DataType_MAX;
  static const int DataType_ARRAYSIZE =
    SequenceProto_DataType_DataType_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  DataType_descriptor() {
    return SequenceProto_DataType_descriptor();
  }
  static inline const ::std::string& DataType_Name(DataType value) {
    return SequenceProto_DataType_Name(value);
  }
  static inline bool DataType_Parse(const ::std::string& name,
      DataType* value) {
    return SequenceProto_DataType_Parse(name, value);
  }

  // accessors -------------------------------------------------------

  // repeated .onnx2trt_onnx.TensorProto tensor_values = 3;
  int tensor_values_size() const;
  void clear_tensor_values();
  static const int kTensorValuesFieldNumber = 3;
  ::onnx2trt_onnx::TensorProto* mutable_tensor_values(int index);
  ::google::protobuf::RepeatedPtrField< ::onnx2trt_onnx::TensorProto >*
      mutable_tensor_values();
  const ::onnx2trt_onnx::TensorProto& tensor_values(int index) const;
  ::onnx2trt_onnx::TensorProto* add_tensor_values();
  const ::google::protobuf::RepeatedPtrField< ::onnx2trt_onnx::TensorProto >&
      tensor_values() const;

  // repeated .onnx2trt_onnx.SparseTensorProto sparse_tensor_values = 4;
  int sparse_tensor_values_size() const;
  void clear_sparse_tensor_values();
  static const int kSparseTensorValuesFieldNumber = 4;
  ::onnx2trt_onnx::SparseTensorProto* mutable_sparse_tensor_values(int index);
  ::google::protobuf::RepeatedPtrField< ::onnx2trt_onnx::SparseTensorProto >*
      mutable_sparse_tensor_values();
  const ::onnx2trt_onnx::SparseTensorProto& sparse_tensor_values(int index) const;
  ::onnx2trt_onnx::SparseTensorProto* add_sparse_tensor_values();
  const ::google::protobuf::RepeatedPtrField< ::onnx2trt_onnx::SparseTensorProto >&
      sparse_tensor_values() const;

  // repeated .onnx2trt_onnx.SequenceProto sequence_values = 5;
  int sequence_values_size() const;
  void clear_sequence_values();
  static const int kSequenceValuesFieldNumber = 5;
  ::onnx2trt_onnx::SequenceProto* mutable_sequence_values(int index);
  ::google::protobuf::RepeatedPtrField< ::onnx2trt_onnx::SequenceProto >*
      mutable_sequence_values();
  const ::onnx2trt_onnx::SequenceProto& sequence_values(int index) const;
  ::onnx2trt_onnx::SequenceProto* add_sequence_values();
  const ::google::protobuf::RepeatedPtrField< ::onnx2trt_onnx::SequenceProto >&
      sequence_values() const;

  // repeated .onnx2trt_onnx.MapProto map_values = 6;
  int map_values_size() const;
  void clear_map_values();
  static const int kMapValuesFieldNumber = 6;
  ::onnx2trt_onnx::MapProto* mutable_map_values(int index);
  ::google::protobuf::RepeatedPtrField< ::onnx2trt_onnx::MapProto >*
      mutable_map_values();
  const ::onnx2trt_onnx::MapProto& map_values(int index) const;
  ::onnx2trt_onnx::MapProto* add_map_values();
  const ::google::protobuf::RepeatedPtrField< ::onnx2trt_onnx::MapProto >&
      map_values() const;

  // optional string name = 1;
  bool has_name() const;
  void clear_name();
  static const int kNameFieldNumber = 1;
  const ::std::string& name() const;
  void set_name(const ::std::string& value);
  #if LANG_CXX11
  void set_name(::std::string&& value);
  #endif
  void set_name(const char* value);
  void set_name(const char* value, size_t size);
  ::std::string* mutable_name();
  ::std::string* release_name();
  void set_allocated_name(::std::string* name);

  // optional int32 elem_type = 2;
  bool has_elem_type() const;
  void clear_elem_type();
  static const int kElemTypeFieldNumber = 2;
  ::google::protobuf::int32 elem_type() const;
  void set_elem_type(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:onnx2trt_onnx.SequenceProto)
 private:
  void set_has_name();
  void clear_has_name();
  void set_has_elem_type();
  void clear_has_elem_type();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::RepeatedPtrField< ::onnx2trt_onnx::TensorProto > tensor_values_;
  ::google::protobuf::RepeatedPtrField< ::onnx2trt_onnx::SparseTensorProto > sparse_tensor_values_;
  ::google::protobuf::RepeatedPtrField< ::onnx2trt_onnx::SequenceProto > sequence_values_;
  ::google::protobuf::RepeatedPtrField< ::onnx2trt_onnx::MapProto > map_values_;
  ::google::protobuf::internal::ArenaStringPtr name_;
  ::google::protobuf::int32 elem_type_;
  friend struct ::protobuf_onnx_2fonnx_2ddata_5fonnx2trt_5fonnx_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class ONNX_API MapProto : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:onnx2trt_onnx.MapProto) */ {
 public:
  MapProto();
  virtual ~MapProto();

  MapProto(const MapProto& from);

  inline MapProto& operator=(const MapProto& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  MapProto(MapProto&& from) noexcept
    : MapProto() {
    *this = ::std::move(from);
  }

  inline MapProto& operator=(MapProto&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const MapProto& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const MapProto* internal_default_instance() {
    return reinterpret_cast<const MapProto*>(
               &_MapProto_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  void Swap(MapProto* other);
  friend void swap(MapProto& a, MapProto& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline MapProto* New() const final {
    return CreateMaybeMessage<MapProto>(NULL);
  }

  MapProto* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<MapProto>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const MapProto& from);
  void MergeFrom(const MapProto& from);
  void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(MapProto* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // repeated int64 keys = 3;
  int keys_size() const;
  void clear_keys();
  static const int kKeysFieldNumber = 3;
  ::google::protobuf::int64 keys(int index) const;
  void set_keys(int index, ::google::protobuf::int64 value);
  void add_keys(::google::protobuf::int64 value);
  const ::google::protobuf::RepeatedField< ::google::protobuf::int64 >&
      keys() const;
  ::google::protobuf::RepeatedField< ::google::protobuf::int64 >*
      mutable_keys();

  // repeated bytes string_keys = 4;
  int string_keys_size() const;
  void clear_string_keys();
  static const int kStringKeysFieldNumber = 4;
  const ::std::string& string_keys(int index) const;
  ::std::string* mutable_string_keys(int index);
  void set_string_keys(int index, const ::std::string& value);
  #if LANG_CXX11
  void set_string_keys(int index, ::std::string&& value);
  #endif
  void set_string_keys(int index, const char* value);
  void set_string_keys(int index, const void* value, size_t size);
  ::std::string* add_string_keys();
  void add_string_keys(const ::std::string& value);
  #if LANG_CXX11
  void add_string_keys(::std::string&& value);
  #endif
  void add_string_keys(const char* value);
  void add_string_keys(const void* value, size_t size);
  const ::google::protobuf::RepeatedPtrField< ::std::string>& string_keys() const;
  ::google::protobuf::RepeatedPtrField< ::std::string>* mutable_string_keys();

  // optional string name = 1;
  bool has_name() const;
  void clear_name();
  static const int kNameFieldNumber = 1;
  const ::std::string& name() const;
  void set_name(const ::std::string& value);
  #if LANG_CXX11
  void set_name(::std::string&& value);
  #endif
  void set_name(const char* value);
  void set_name(const char* value, size_t size);
  ::std::string* mutable_name();
  ::std::string* release_name();
  void set_allocated_name(::std::string* name);

  // optional .onnx2trt_onnx.SequenceProto values = 5;
  bool has_values() const;
  void clear_values();
  static const int kValuesFieldNumber = 5;
  private:
  const ::onnx2trt_onnx::SequenceProto& _internal_values() const;
  public:
  const ::onnx2trt_onnx::SequenceProto& values() const;
  ::onnx2trt_onnx::SequenceProto* release_values();
  ::onnx2trt_onnx::SequenceProto* mutable_values();
  void set_allocated_values(::onnx2trt_onnx::SequenceProto* values);

  // optional int32 key_type = 2;
  bool has_key_type() const;
  void clear_key_type();
  static const int kKeyTypeFieldNumber = 2;
  ::google::protobuf::int32 key_type() const;
  void set_key_type(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:onnx2trt_onnx.MapProto)
 private:
  void set_has_name();
  void clear_has_name();
  void set_has_key_type();
  void clear_has_key_type();
  void set_has_values();
  void clear_has_values();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::RepeatedField< ::google::protobuf::int64 > keys_;
  ::google::protobuf::RepeatedPtrField< ::std::string> string_keys_;
  ::google::protobuf::internal::ArenaStringPtr name_;
  ::onnx2trt_onnx::SequenceProto* values_;
  ::google::protobuf::int32 key_type_;
  friend struct ::protobuf_onnx_2fonnx_2ddata_5fonnx2trt_5fonnx_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// SequenceProto

// optional string name = 1;
inline bool SequenceProto::has_name() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void SequenceProto::set_has_name() {
  _has_bits_[0] |= 0x00000001u;
}
inline void SequenceProto::clear_has_name() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void SequenceProto::clear_name() {
  name_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_name();
}
inline const ::std::string& SequenceProto::name() const {
  // @@protoc_insertion_point(field_get:onnx2trt_onnx.SequenceProto.name)
  return name_.GetNoArena();
}
inline void SequenceProto::set_name(const ::std::string& value) {
  set_has_name();
  name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:onnx2trt_onnx.SequenceProto.name)
}
#if LANG_CXX11
inline void SequenceProto::set_name(::std::string&& value) {
  set_has_name();
  name_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:onnx2trt_onnx.SequenceProto.name)
}
#endif
inline void SequenceProto::set_name(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_name();
  name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:onnx2trt_onnx.SequenceProto.name)
}
inline void SequenceProto::set_name(const char* value, size_t size) {
  set_has_name();
  name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:onnx2trt_onnx.SequenceProto.name)
}
inline ::std::string* SequenceProto::mutable_name() {
  set_has_name();
  // @@protoc_insertion_point(field_mutable:onnx2trt_onnx.SequenceProto.name)
  return name_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* SequenceProto::release_name() {
  // @@protoc_insertion_point(field_release:onnx2trt_onnx.SequenceProto.name)
  if (!has_name()) {
    return NULL;
  }
  clear_has_name();
  return name_.ReleaseNonDefaultNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void SequenceProto::set_allocated_name(::std::string* name) {
  if (name != NULL) {
    set_has_name();
  } else {
    clear_has_name();
  }
  name_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), name);
  // @@protoc_insertion_point(field_set_allocated:onnx2trt_onnx.SequenceProto.name)
}

// optional int32 elem_type = 2;
inline bool SequenceProto::has_elem_type() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void SequenceProto::set_has_elem_type() {
  _has_bits_[0] |= 0x00000002u;
}
inline void SequenceProto::clear_has_elem_type() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void SequenceProto::clear_elem_type() {
  elem_type_ = 0;
  clear_has_elem_type();
}
inline ::google::protobuf::int32 SequenceProto::elem_type() const {
  // @@protoc_insertion_point(field_get:onnx2trt_onnx.SequenceProto.elem_type)
  return elem_type_;
}
inline void SequenceProto::set_elem_type(::google::protobuf::int32 value) {
  set_has_elem_type();
  elem_type_ = value;
  // @@protoc_insertion_point(field_set:onnx2trt_onnx.SequenceProto.elem_type)
}

// repeated .onnx2trt_onnx.TensorProto tensor_values = 3;
inline int SequenceProto::tensor_values_size() const {
  return tensor_values_.size();
}
inline ::onnx2trt_onnx::TensorProto* SequenceProto::mutable_tensor_values(int index) {
  // @@protoc_insertion_point(field_mutable:onnx2trt_onnx.SequenceProto.tensor_values)
  return tensor_values_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::onnx2trt_onnx::TensorProto >*
SequenceProto::mutable_tensor_values() {
  // @@protoc_insertion_point(field_mutable_list:onnx2trt_onnx.SequenceProto.tensor_values)
  return &tensor_values_;
}
inline const ::onnx2trt_onnx::TensorProto& SequenceProto::tensor_values(int index) const {
  // @@protoc_insertion_point(field_get:onnx2trt_onnx.SequenceProto.tensor_values)
  return tensor_values_.Get(index);
}
inline ::onnx2trt_onnx::TensorProto* SequenceProto::add_tensor_values() {
  // @@protoc_insertion_point(field_add:onnx2trt_onnx.SequenceProto.tensor_values)
  return tensor_values_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::onnx2trt_onnx::TensorProto >&
SequenceProto::tensor_values() const {
  // @@protoc_insertion_point(field_list:onnx2trt_onnx.SequenceProto.tensor_values)
  return tensor_values_;
}

// repeated .onnx2trt_onnx.SparseTensorProto sparse_tensor_values = 4;
inline int SequenceProto::sparse_tensor_values_size() const {
  return sparse_tensor_values_.size();
}
inline ::onnx2trt_onnx::SparseTensorProto* SequenceProto::mutable_sparse_tensor_values(int index) {
  // @@protoc_insertion_point(field_mutable:onnx2trt_onnx.SequenceProto.sparse_tensor_values)
  return sparse_tensor_values_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::onnx2trt_onnx::SparseTensorProto >*
SequenceProto::mutable_sparse_tensor_values() {
  // @@protoc_insertion_point(field_mutable_list:onnx2trt_onnx.SequenceProto.sparse_tensor_values)
  return &sparse_tensor_values_;
}
inline const ::onnx2trt_onnx::SparseTensorProto& SequenceProto::sparse_tensor_values(int index) const {
  // @@protoc_insertion_point(field_get:onnx2trt_onnx.SequenceProto.sparse_tensor_values)
  return sparse_tensor_values_.Get(index);
}
inline ::onnx2trt_onnx::SparseTensorProto* SequenceProto::add_sparse_tensor_values() {
  // @@protoc_insertion_point(field_add:onnx2trt_onnx.SequenceProto.sparse_tensor_values)
  return sparse_tensor_values_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::onnx2trt_onnx::SparseTensorProto >&
SequenceProto::sparse_tensor_values() const {
  // @@protoc_insertion_point(field_list:onnx2trt_onnx.SequenceProto.sparse_tensor_values)
  return sparse_tensor_values_;
}

// repeated .onnx2trt_onnx.SequenceProto sequence_values = 5;
inline int SequenceProto::sequence_values_size() const {
  return sequence_values_.size();
}
inline void SequenceProto::clear_sequence_values() {
  sequence_values_.Clear();
}
inline ::onnx2trt_onnx::SequenceProto* SequenceProto::mutable_sequence_values(int index) {
  // @@protoc_insertion_point(field_mutable:onnx2trt_onnx.SequenceProto.sequence_values)
  return sequence_values_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::onnx2trt_onnx::SequenceProto >*
SequenceProto::mutable_sequence_values() {
  // @@protoc_insertion_point(field_mutable_list:onnx2trt_onnx.SequenceProto.sequence_values)
  return &sequence_values_;
}
inline const ::onnx2trt_onnx::SequenceProto& SequenceProto::sequence_values(int index) const {
  // @@protoc_insertion_point(field_get:onnx2trt_onnx.SequenceProto.sequence_values)
  return sequence_values_.Get(index);
}
inline ::onnx2trt_onnx::SequenceProto* SequenceProto::add_sequence_values() {
  // @@protoc_insertion_point(field_add:onnx2trt_onnx.SequenceProto.sequence_values)
  return sequence_values_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::onnx2trt_onnx::SequenceProto >&
SequenceProto::sequence_values() const {
  // @@protoc_insertion_point(field_list:onnx2trt_onnx.SequenceProto.sequence_values)
  return sequence_values_;
}

// repeated .onnx2trt_onnx.MapProto map_values = 6;
inline int SequenceProto::map_values_size() const {
  return map_values_.size();
}
inline void SequenceProto::clear_map_values() {
  map_values_.Clear();
}
inline ::onnx2trt_onnx::MapProto* SequenceProto::mutable_map_values(int index) {
  // @@protoc_insertion_point(field_mutable:onnx2trt_onnx.SequenceProto.map_values)
  return map_values_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::onnx2trt_onnx::MapProto >*
SequenceProto::mutable_map_values() {
  // @@protoc_insertion_point(field_mutable_list:onnx2trt_onnx.SequenceProto.map_values)
  return &map_values_;
}
inline const ::onnx2trt_onnx::MapProto& SequenceProto::map_values(int index) const {
  // @@protoc_insertion_point(field_get:onnx2trt_onnx.SequenceProto.map_values)
  return map_values_.Get(index);
}
inline ::onnx2trt_onnx::MapProto* SequenceProto::add_map_values() {
  // @@protoc_insertion_point(field_add:onnx2trt_onnx.SequenceProto.map_values)
  return map_values_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::onnx2trt_onnx::MapProto >&
SequenceProto::map_values() const {
  // @@protoc_insertion_point(field_list:onnx2trt_onnx.SequenceProto.map_values)
  return map_values_;
}

// -------------------------------------------------------------------

// MapProto

// optional string name = 1;
inline bool MapProto::has_name() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void MapProto::set_has_name() {
  _has_bits_[0] |= 0x00000001u;
}
inline void MapProto::clear_has_name() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void MapProto::clear_name() {
  name_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_name();
}
inline const ::std::string& MapProto::name() const {
  // @@protoc_insertion_point(field_get:onnx2trt_onnx.MapProto.name)
  return name_.GetNoArena();
}
inline void MapProto::set_name(const ::std::string& value) {
  set_has_name();
  name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:onnx2trt_onnx.MapProto.name)
}
#if LANG_CXX11
inline void MapProto::set_name(::std::string&& value) {
  set_has_name();
  name_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:onnx2trt_onnx.MapProto.name)
}
#endif
inline void MapProto::set_name(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_name();
  name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:onnx2trt_onnx.MapProto.name)
}
inline void MapProto::set_name(const char* value, size_t size) {
  set_has_name();
  name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:onnx2trt_onnx.MapProto.name)
}
inline ::std::string* MapProto::mutable_name() {
  set_has_name();
  // @@protoc_insertion_point(field_mutable:onnx2trt_onnx.MapProto.name)
  return name_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* MapProto::release_name() {
  // @@protoc_insertion_point(field_release:onnx2trt_onnx.MapProto.name)
  if (!has_name()) {
    return NULL;
  }
  clear_has_name();
  return name_.ReleaseNonDefaultNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void MapProto::set_allocated_name(::std::string* name) {
  if (name != NULL) {
    set_has_name();
  } else {
    clear_has_name();
  }
  name_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), name);
  // @@protoc_insertion_point(field_set_allocated:onnx2trt_onnx.MapProto.name)
}

// optional int32 key_type = 2;
inline bool MapProto::has_key_type() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void MapProto::set_has_key_type() {
  _has_bits_[0] |= 0x00000004u;
}
inline void MapProto::clear_has_key_type() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void MapProto::clear_key_type() {
  key_type_ = 0;
  clear_has_key_type();
}
inline ::google::protobuf::int32 MapProto::key_type() const {
  // @@protoc_insertion_point(field_get:onnx2trt_onnx.MapProto.key_type)
  return key_type_;
}
inline void MapProto::set_key_type(::google::protobuf::int32 value) {
  set_has_key_type();
  key_type_ = value;
  // @@protoc_insertion_point(field_set:onnx2trt_onnx.MapProto.key_type)
}

// repeated int64 keys = 3;
inline int MapProto::keys_size() const {
  return keys_.size();
}
inline void MapProto::clear_keys() {
  keys_.Clear();
}
inline ::google::protobuf::int64 MapProto::keys(int index) const {
  // @@protoc_insertion_point(field_get:onnx2trt_onnx.MapProto.keys)
  return keys_.Get(index);
}
inline void MapProto::set_keys(int index, ::google::protobuf::int64 value) {
  keys_.Set(index, value);
  // @@protoc_insertion_point(field_set:onnx2trt_onnx.MapProto.keys)
}
inline void MapProto::add_keys(::google::protobuf::int64 value) {
  keys_.Add(value);
  // @@protoc_insertion_point(field_add:onnx2trt_onnx.MapProto.keys)
}
inline const ::google::protobuf::RepeatedField< ::google::protobuf::int64 >&
MapProto::keys() const {
  // @@protoc_insertion_point(field_list:onnx2trt_onnx.MapProto.keys)
  return keys_;
}
inline ::google::protobuf::RepeatedField< ::google::protobuf::int64 >*
MapProto::mutable_keys() {
  // @@protoc_insertion_point(field_mutable_list:onnx2trt_onnx.MapProto.keys)
  return &keys_;
}

// repeated bytes string_keys = 4;
inline int MapProto::string_keys_size() const {
  return string_keys_.size();
}
inline void MapProto::clear_string_keys() {
  string_keys_.Clear();
}
inline const ::std::string& MapProto::string_keys(int index) const {
  // @@protoc_insertion_point(field_get:onnx2trt_onnx.MapProto.string_keys)
  return string_keys_.Get(index);
}
inline ::std::string* MapProto::mutable_string_keys(int index) {
  // @@protoc_insertion_point(field_mutable:onnx2trt_onnx.MapProto.string_keys)
  return string_keys_.Mutable(index);
}
inline void MapProto::set_string_keys(int index, const ::std::string& value) {
  // @@protoc_insertion_point(field_set:onnx2trt_onnx.MapProto.string_keys)
  string_keys_.Mutable(index)->assign(value);
}
#if LANG_CXX11
inline void MapProto::set_string_keys(int index, ::std::string&& value) {
  // @@protoc_insertion_point(field_set:onnx2trt_onnx.MapProto.string_keys)
  string_keys_.Mutable(index)->assign(std::move(value));
}
#endif
inline void MapProto::set_string_keys(int index, const char* value) {
  GOOGLE_DCHECK(value != NULL);
  string_keys_.Mutable(index)->assign(value);
  // @@protoc_insertion_point(field_set_char:onnx2trt_onnx.MapProto.string_keys)
}
inline void MapProto::set_string_keys(int index, const void* value, size_t size) {
  string_keys_.Mutable(index)->assign(
    reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:onnx2trt_onnx.MapProto.string_keys)
}
inline ::std::string* MapProto::add_string_keys() {
  // @@protoc_insertion_point(field_add_mutable:onnx2trt_onnx.MapProto.string_keys)
  return string_keys_.Add();
}
inline void MapProto::add_string_keys(const ::std::string& value) {
  string_keys_.Add()->assign(value);
  // @@protoc_insertion_point(field_add:onnx2trt_onnx.MapProto.string_keys)
}
#if LANG_CXX11
inline void MapProto::add_string_keys(::std::string&& value) {
  string_keys_.Add(std::move(value));
  // @@protoc_insertion_point(field_add:onnx2trt_onnx.MapProto.string_keys)
}
#endif
inline void MapProto::add_string_keys(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  string_keys_.Add()->assign(value);
  // @@protoc_insertion_point(field_add_char:onnx2trt_onnx.MapProto.string_keys)
}
inline void MapProto::add_string_keys(const void* value, size_t size) {
  string_keys_.Add()->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_add_pointer:onnx2trt_onnx.MapProto.string_keys)
}
inline const ::google::protobuf::RepeatedPtrField< ::std::string>&
MapProto::string_keys() const {
  // @@protoc_insertion_point(field_list:onnx2trt_onnx.MapProto.string_keys)
  return string_keys_;
}
inline ::google::protobuf::RepeatedPtrField< ::std::string>*
MapProto::mutable_string_keys() {
  // @@protoc_insertion_point(field_mutable_list:onnx2trt_onnx.MapProto.string_keys)
  return &string_keys_;
}

// optional .onnx2trt_onnx.SequenceProto values = 5;
inline bool MapProto::has_values() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void MapProto::set_has_values() {
  _has_bits_[0] |= 0x00000002u;
}
inline void MapProto::clear_has_values() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void MapProto::clear_values() {
  if (values_ != NULL) values_->Clear();
  clear_has_values();
}
inline const ::onnx2trt_onnx::SequenceProto& MapProto::_internal_values() const {
  return *values_;
}
inline const ::onnx2trt_onnx::SequenceProto& MapProto::values() const {
  const ::onnx2trt_onnx::SequenceProto* p = values_;
  // @@protoc_insertion_point(field_get:onnx2trt_onnx.MapProto.values)
  return p != NULL ? *p : *reinterpret_cast<const ::onnx2trt_onnx::SequenceProto*>(
      &::onnx2trt_onnx::_SequenceProto_default_instance_);
}
inline ::onnx2trt_onnx::SequenceProto* MapProto::release_values() {
  // @@protoc_insertion_point(field_release:onnx2trt_onnx.MapProto.values)
  clear_has_values();
  ::onnx2trt_onnx::SequenceProto* temp = values_;
  values_ = NULL;
  return temp;
}
inline ::onnx2trt_onnx::SequenceProto* MapProto::mutable_values() {
  set_has_values();
  if (values_ == NULL) {
    auto* p = CreateMaybeMessage<::onnx2trt_onnx::SequenceProto>(GetArenaNoVirtual());
    values_ = p;
  }
  // @@protoc_insertion_point(field_mutable:onnx2trt_onnx.MapProto.values)
  return values_;
}
inline void MapProto::set_allocated_values(::onnx2trt_onnx::SequenceProto* values) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete values_;
  }
  if (values) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      values = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, values, submessage_arena);
    }
    set_has_values();
  } else {
    clear_has_values();
  }
  values_ = values;
  // @@protoc_insertion_point(field_set_allocated:onnx2trt_onnx.MapProto.values)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace onnx2trt_onnx

namespace google {
namespace protobuf {

template <> struct is_proto_enum< ::onnx2trt_onnx::SequenceProto_DataType> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::onnx2trt_onnx::SequenceProto_DataType>() {
  return ::onnx2trt_onnx::SequenceProto_DataType_descriptor();
}

}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_onnx_2fonnx_2ddata_5fonnx2trt_5fonnx_2eproto
