// Auto-generated. Do not edit!

// (in-package stoch3_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class QuadrupedLegFeedback {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.name = null;
      this.foot_position = null;
      this.foot_velocity = null;
      this.foot_force = null;
      this.foot_pos_error = null;
      this.error_status = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = [];
      }
      if (initObj.hasOwnProperty('foot_position')) {
        this.foot_position = initObj.foot_position
      }
      else {
        this.foot_position = [];
      }
      if (initObj.hasOwnProperty('foot_velocity')) {
        this.foot_velocity = initObj.foot_velocity
      }
      else {
        this.foot_velocity = [];
      }
      if (initObj.hasOwnProperty('foot_force')) {
        this.foot_force = initObj.foot_force
      }
      else {
        this.foot_force = [];
      }
      if (initObj.hasOwnProperty('foot_pos_error')) {
        this.foot_pos_error = initObj.foot_pos_error
      }
      else {
        this.foot_pos_error = [];
      }
      if (initObj.hasOwnProperty('error_status')) {
        this.error_status = initObj.error_status
      }
      else {
        this.error_status = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type QuadrupedLegFeedback
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [name]
    bufferOffset = _arraySerializer.string(obj.name, buffer, bufferOffset, null);
    // Serialize message field [foot_position]
    bufferOffset = _arraySerializer.float64(obj.foot_position, buffer, bufferOffset, null);
    // Serialize message field [foot_velocity]
    bufferOffset = _arraySerializer.float64(obj.foot_velocity, buffer, bufferOffset, null);
    // Serialize message field [foot_force]
    bufferOffset = _arraySerializer.float64(obj.foot_force, buffer, bufferOffset, null);
    // Serialize message field [foot_pos_error]
    bufferOffset = _arraySerializer.float64(obj.foot_pos_error, buffer, bufferOffset, null);
    // Serialize message field [error_status]
    bufferOffset = _serializer.bool(obj.error_status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type QuadrupedLegFeedback
    let len;
    let data = new QuadrupedLegFeedback(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [name]
    data.name = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [foot_position]
    data.foot_position = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [foot_velocity]
    data.foot_velocity = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [foot_force]
    data.foot_force = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [foot_pos_error]
    data.foot_pos_error = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [error_status]
    data.error_status = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.name.forEach((val) => {
      length += 4 + val.length;
    });
    length += 8 * object.foot_position.length;
    length += 8 * object.foot_velocity.length;
    length += 8 * object.foot_force.length;
    length += 8 * object.foot_pos_error.length;
    return length + 21;
  }

  static datatype() {
    // Returns string type for a message object
    return 'stoch3_msgs/QuadrupedLegFeedback';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7475d2278216c40ff50e977798003eb3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header    header
    string[]  name
    float64[] foot_position
    float64[] foot_velocity
    float64[] foot_force
    float64[] foot_pos_error
    bool error_status
    
    
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
    const resolved = new QuadrupedLegFeedback(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = []
    }

    if (msg.foot_position !== undefined) {
      resolved.foot_position = msg.foot_position;
    }
    else {
      resolved.foot_position = []
    }

    if (msg.foot_velocity !== undefined) {
      resolved.foot_velocity = msg.foot_velocity;
    }
    else {
      resolved.foot_velocity = []
    }

    if (msg.foot_force !== undefined) {
      resolved.foot_force = msg.foot_force;
    }
    else {
      resolved.foot_force = []
    }

    if (msg.foot_pos_error !== undefined) {
      resolved.foot_pos_error = msg.foot_pos_error;
    }
    else {
      resolved.foot_pos_error = []
    }

    if (msg.error_status !== undefined) {
      resolved.error_status = msg.error_status;
    }
    else {
      resolved.error_status = false
    }

    return resolved;
    }
};

module.exports = QuadrupedLegFeedback;
