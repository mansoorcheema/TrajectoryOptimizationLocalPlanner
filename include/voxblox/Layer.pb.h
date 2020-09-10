// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: Layer.proto

#ifndef PROTOBUF_Layer_2eproto__INCLUDED
#define PROTOBUF_Layer_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 2006000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 2006001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)

namespace voxblox {

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_Layer_2eproto();
void protobuf_AssignDesc_Layer_2eproto();
void protobuf_ShutdownFile_Layer_2eproto();

class LayerProto;

// ===================================================================

class LayerProto : public ::google::protobuf::Message {
 public:
  LayerProto();
  virtual ~LayerProto();

  LayerProto(const LayerProto& from);

  inline LayerProto& operator=(const LayerProto& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const LayerProto& default_instance();

  void Swap(LayerProto* other);

  // implements Message ----------------------------------------------

  LayerProto* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const LayerProto& from);
  void MergeFrom(const LayerProto& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:
  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional double voxel_size = 1;
  inline bool has_voxel_size() const;
  inline void clear_voxel_size();
  static const int kVoxelSizeFieldNumber = 1;
  inline double voxel_size() const;
  inline void set_voxel_size(double value);

  // optional uint32 voxels_per_side = 2;
  inline bool has_voxels_per_side() const;
  inline void clear_voxels_per_side();
  static const int kVoxelsPerSideFieldNumber = 2;
  inline ::google::protobuf::uint32 voxels_per_side() const;
  inline void set_voxels_per_side(::google::protobuf::uint32 value);

  // optional string type = 3;
  inline bool has_type() const;
  inline void clear_type();
  static const int kTypeFieldNumber = 3;
  inline const ::std::string& type() const;
  inline void set_type(const ::std::string& value);
  inline void set_type(const char* value);
  inline void set_type(const char* value, size_t size);
  inline ::std::string* mutable_type();
  inline ::std::string* release_type();
  inline void set_allocated_type(::std::string* type);

  // @@protoc_insertion_point(class_scope:voxblox.LayerProto)
 private:
  inline void set_has_voxel_size();
  inline void clear_has_voxel_size();
  inline void set_has_voxels_per_side();
  inline void clear_has_voxels_per_side();
  inline void set_has_type();
  inline void clear_has_type();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  double voxel_size_;
  ::std::string* type_;
  ::google::protobuf::uint32 voxels_per_side_;
  friend void  protobuf_AddDesc_Layer_2eproto();
  friend void protobuf_AssignDesc_Layer_2eproto();
  friend void protobuf_ShutdownFile_Layer_2eproto();

  void InitAsDefaultInstance();
  static LayerProto* default_instance_;
};
// ===================================================================


// ===================================================================

// LayerProto

// optional double voxel_size = 1;
inline bool LayerProto::has_voxel_size() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void LayerProto::set_has_voxel_size() {
  _has_bits_[0] |= 0x00000001u;
}
inline void LayerProto::clear_has_voxel_size() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void LayerProto::clear_voxel_size() {
  voxel_size_ = 0;
  clear_has_voxel_size();
}
inline double LayerProto::voxel_size() const {
  // @@protoc_insertion_point(field_get:voxblox.LayerProto.voxel_size)
  return voxel_size_;
}
inline void LayerProto::set_voxel_size(double value) {
  set_has_voxel_size();
  voxel_size_ = value;
  // @@protoc_insertion_point(field_set:voxblox.LayerProto.voxel_size)
}

// optional uint32 voxels_per_side = 2;
inline bool LayerProto::has_voxels_per_side() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void LayerProto::set_has_voxels_per_side() {
  _has_bits_[0] |= 0x00000002u;
}
inline void LayerProto::clear_has_voxels_per_side() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void LayerProto::clear_voxels_per_side() {
  voxels_per_side_ = 0u;
  clear_has_voxels_per_side();
}
inline ::google::protobuf::uint32 LayerProto::voxels_per_side() const {
  // @@protoc_insertion_point(field_get:voxblox.LayerProto.voxels_per_side)
  return voxels_per_side_;
}
inline void LayerProto::set_voxels_per_side(::google::protobuf::uint32 value) {
  set_has_voxels_per_side();
  voxels_per_side_ = value;
  // @@protoc_insertion_point(field_set:voxblox.LayerProto.voxels_per_side)
}

// optional string type = 3;
inline bool LayerProto::has_type() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void LayerProto::set_has_type() {
  _has_bits_[0] |= 0x00000004u;
}
inline void LayerProto::clear_has_type() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void LayerProto::clear_type() {
  if (type_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    type_->clear();
  }
  clear_has_type();
}
inline const ::std::string& LayerProto::type() const {
  // @@protoc_insertion_point(field_get:voxblox.LayerProto.type)
  return *type_;
}
inline void LayerProto::set_type(const ::std::string& value) {
  set_has_type();
  if (type_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    type_ = new ::std::string;
  }
  type_->assign(value);
  // @@protoc_insertion_point(field_set:voxblox.LayerProto.type)
}
inline void LayerProto::set_type(const char* value) {
  set_has_type();
  if (type_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    type_ = new ::std::string;
  }
  type_->assign(value);
  // @@protoc_insertion_point(field_set_char:voxblox.LayerProto.type)
}
inline void LayerProto::set_type(const char* value, size_t size) {
  set_has_type();
  if (type_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    type_ = new ::std::string;
  }
  type_->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:voxblox.LayerProto.type)
}
inline ::std::string* LayerProto::mutable_type() {
  set_has_type();
  if (type_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    type_ = new ::std::string;
  }
  // @@protoc_insertion_point(field_mutable:voxblox.LayerProto.type)
  return type_;
}
inline ::std::string* LayerProto::release_type() {
  clear_has_type();
  if (type_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    return NULL;
  } else {
    ::std::string* temp = type_;
    type_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
    return temp;
  }
}
inline void LayerProto::set_allocated_type(::std::string* type) {
  if (type_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    delete type_;
  }
  if (type) {
    set_has_type();
    type_ = type;
  } else {
    clear_has_type();
    type_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  }
  // @@protoc_insertion_point(field_set_allocated:voxblox.LayerProto.type)
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace voxblox

#ifndef SWIG
namespace google {
namespace protobuf {


}  // namespace google
}  // namespace protobuf
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_Layer_2eproto__INCLUDED