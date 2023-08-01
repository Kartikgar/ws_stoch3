// Auto-generated. Do not edit!

// (in-package pronto_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class VisualOdometryUpdate {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.curr_timestamp = null;
      this.prev_timestamp = null;
      this.relative_transform = null;
      this.covariance = null;
      this.estimate_status = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('curr_timestamp')) {
        this.curr_timestamp = initObj.curr_timestamp
      }
      else {
        this.curr_timestamp = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('prev_timestamp')) {
        this.prev_timestamp = initObj.prev_timestamp
      }
      else {
        this.prev_timestamp = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('relative_transform')) {
        this.relative_transform = initObj.relative_transform
      }
      else {
        this.relative_transform = new geometry_msgs.msg.Transform();
      }
      if (initObj.hasOwnProperty('covariance')) {
        this.covariance = initObj.covariance
      }
      else {
        this.covariance = new Array(36).fill(0);
      }
      if (initObj.hasOwnProperty('estimate_status')) {
        this.estimate_status = initObj.estimate_status
      }
      else {
        this.estimate_status = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VisualOdometryUpdate
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [curr_timestamp]
    bufferOffset = _serializer.time(obj.curr_timestamp, buffer, bufferOffset);
    // Serialize message field [prev_timestamp]
    bufferOffset = _serializer.time(obj.prev_timestamp, buffer, bufferOffset);
    // Serialize message field [relative_transform]
    bufferOffset = geometry_msgs.msg.Transform.serialize(obj.relative_transform, buffer, bufferOffset);
    // Check that the constant length array field [covariance] has the right length
    if (obj.covariance.length !== 36) {
      throw new Error('Unable to serialize array field covariance - length must be 36')
    }
    // Serialize message field [covariance]
    bufferOffset = _arraySerializer.float64(obj.covariance, buffer, bufferOffset, 36);
    // Serialize message field [estimate_status]
    bufferOffset = _serializer.uint8(obj.estimate_status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VisualOdometryUpdate
    let len;
    let data = new VisualOdometryUpdate(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [curr_timestamp]
    data.curr_timestamp = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [prev_timestamp]
    data.prev_timestamp = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [relative_transform]
    data.relative_transform = geometry_msgs.msg.Transform.deserialize(buffer, bufferOffset);
    // Deserialize message field [covariance]
    data.covariance = _arrayDeserializer.float64(buffer, bufferOffset, 36)
    // Deserialize message field [estimate_status]
    data.estimate_status = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 361;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pronto_msgs/VisualOdometryUpdate';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '86e4e9402396b9789cfeaadd317551b4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    time curr_timestamp
    time prev_timestamp
    geometry_msgs/Transform relative_transform
    float64[36] covariance
    uint8 estimate_status
    
    uint8 NO_DATA = 0
    uint8 ESTIMATE_VALID = 1
    uint8 ESTIMATE_INSUFFICIENT_FEATURES = 2
    uint8 ESTIMATE_DEGENERATE = 3
    uint8 ESTIMATE_REPROJECTION_ERROR = 4
    
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Transform
    # This represents the transform between two coordinate frames in free space.
    
    Vector3 translation
    Quaternion rotation
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new VisualOdometryUpdate(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.curr_timestamp !== undefined) {
      resolved.curr_timestamp = msg.curr_timestamp;
    }
    else {
      resolved.curr_timestamp = {secs: 0, nsecs: 0}
    }

    if (msg.prev_timestamp !== undefined) {
      resolved.prev_timestamp = msg.prev_timestamp;
    }
    else {
      resolved.prev_timestamp = {secs: 0, nsecs: 0}
    }

    if (msg.relative_transform !== undefined) {
      resolved.relative_transform = geometry_msgs.msg.Transform.Resolve(msg.relative_transform)
    }
    else {
      resolved.relative_transform = new geometry_msgs.msg.Transform()
    }

    if (msg.covariance !== undefined) {
      resolved.covariance = msg.covariance;
    }
    else {
      resolved.covariance = new Array(36).fill(0)
    }

    if (msg.estimate_status !== undefined) {
      resolved.estimate_status = msg.estimate_status;
    }
    else {
      resolved.estimate_status = 0
    }

    return resolved;
    }
};

// Constants for message
VisualOdometryUpdate.Constants = {
  NO_DATA: 0,
  ESTIMATE_VALID: 1,
  ESTIMATE_INSUFFICIENT_FEATURES: 2,
  ESTIMATE_DEGENERATE: 3,
  ESTIMATE_REPROJECTION_ERROR: 4,
}

module.exports = VisualOdometryUpdate;
