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

class QuadrupedStance {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.lf = null;
      this.rf = null;
      this.lh = null;
      this.rh = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('lf')) {
        this.lf = initObj.lf
      }
      else {
        this.lf = 0.0;
      }
      if (initObj.hasOwnProperty('rf')) {
        this.rf = initObj.rf
      }
      else {
        this.rf = 0.0;
      }
      if (initObj.hasOwnProperty('lh')) {
        this.lh = initObj.lh
      }
      else {
        this.lh = 0.0;
      }
      if (initObj.hasOwnProperty('rh')) {
        this.rh = initObj.rh
      }
      else {
        this.rh = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type QuadrupedStance
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [lf]
    bufferOffset = _serializer.float32(obj.lf, buffer, bufferOffset);
    // Serialize message field [rf]
    bufferOffset = _serializer.float32(obj.rf, buffer, bufferOffset);
    // Serialize message field [lh]
    bufferOffset = _serializer.float32(obj.lh, buffer, bufferOffset);
    // Serialize message field [rh]
    bufferOffset = _serializer.float32(obj.rh, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type QuadrupedStance
    let len;
    let data = new QuadrupedStance(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [lf]
    data.lf = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rf]
    data.rf = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [lh]
    data.lh = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rh]
    data.rh = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pronto_msgs/QuadrupedStance';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6e165cd35d6f3eecca48a2a9f715fd5f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float32 lf
    float32 rf
    float32 lh
    float32 rh
    
    
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
    const resolved = new QuadrupedStance(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.lf !== undefined) {
      resolved.lf = msg.lf;
    }
    else {
      resolved.lf = 0.0
    }

    if (msg.rf !== undefined) {
      resolved.rf = msg.rf;
    }
    else {
      resolved.rf = 0.0
    }

    if (msg.lh !== undefined) {
      resolved.lh = msg.lh;
    }
    else {
      resolved.lh = 0.0
    }

    if (msg.rh !== undefined) {
      resolved.rh = msg.rh;
    }
    else {
      resolved.rh = 0.0
    }

    return resolved;
    }
};

module.exports = QuadrupedStance;
