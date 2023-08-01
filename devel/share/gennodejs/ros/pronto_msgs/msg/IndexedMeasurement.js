// Auto-generated. Do not edit!

// (in-package pronto_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class IndexedMeasurement {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.utime = null;
      this.state_utime = null;
      this.z_effective = null;
      this.z_indices = null;
      this.R_effective = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('utime')) {
        this.utime = initObj.utime
      }
      else {
        this.utime = 0;
      }
      if (initObj.hasOwnProperty('state_utime')) {
        this.state_utime = initObj.state_utime
      }
      else {
        this.state_utime = 0;
      }
      if (initObj.hasOwnProperty('z_effective')) {
        this.z_effective = initObj.z_effective
      }
      else {
        this.z_effective = [];
      }
      if (initObj.hasOwnProperty('z_indices')) {
        this.z_indices = initObj.z_indices
      }
      else {
        this.z_indices = [];
      }
      if (initObj.hasOwnProperty('R_effective')) {
        this.R_effective = initObj.R_effective
      }
      else {
        this.R_effective = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type IndexedMeasurement
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [utime]
    bufferOffset = _serializer.uint64(obj.utime, buffer, bufferOffset);
    // Serialize message field [state_utime]
    bufferOffset = _serializer.uint64(obj.state_utime, buffer, bufferOffset);
    // Serialize message field [z_effective]
    bufferOffset = _arraySerializer.float64(obj.z_effective, buffer, bufferOffset, null);
    // Serialize message field [z_indices]
    bufferOffset = _arraySerializer.int32(obj.z_indices, buffer, bufferOffset, null);
    // Serialize message field [R_effective]
    bufferOffset = _arraySerializer.float64(obj.R_effective, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IndexedMeasurement
    let len;
    let data = new IndexedMeasurement(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [utime]
    data.utime = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [state_utime]
    data.state_utime = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [z_effective]
    data.z_effective = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [z_indices]
    data.z_indices = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [R_effective]
    data.R_effective = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 8 * object.z_effective.length;
    length += 4 * object.z_indices.length;
    length += 8 * object.R_effective.length;
    return length + 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pronto_msgs/IndexedMeasurement';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5a9a7b4bda2dae60a94faa7c688f6e20';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    uint64 utime
    uint64 state_utime
    float64[] z_effective
    int32[] z_indices
    float64[] R_effective
    
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new IndexedMeasurement(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.utime !== undefined) {
      resolved.utime = msg.utime;
    }
    else {
      resolved.utime = 0
    }

    if (msg.state_utime !== undefined) {
      resolved.state_utime = msg.state_utime;
    }
    else {
      resolved.state_utime = 0
    }

    if (msg.z_effective !== undefined) {
      resolved.z_effective = msg.z_effective;
    }
    else {
      resolved.z_effective = []
    }

    if (msg.z_indices !== undefined) {
      resolved.z_indices = msg.z_indices;
    }
    else {
      resolved.z_indices = []
    }

    if (msg.R_effective !== undefined) {
      resolved.R_effective = msg.R_effective;
    }
    else {
      resolved.R_effective = []
    }

    return resolved;
    }
};

module.exports = IndexedMeasurement;
