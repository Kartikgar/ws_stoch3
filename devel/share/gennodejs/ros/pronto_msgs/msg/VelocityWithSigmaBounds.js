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

class VelocityWithSigmaBounds {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.velocity_plus_one_sigma = null;
      this.velocity_minus_one_sigma = null;
      this.plus_one_sigma = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('velocity_plus_one_sigma')) {
        this.velocity_plus_one_sigma = initObj.velocity_plus_one_sigma
      }
      else {
        this.velocity_plus_one_sigma = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('velocity_minus_one_sigma')) {
        this.velocity_minus_one_sigma = initObj.velocity_minus_one_sigma
      }
      else {
        this.velocity_minus_one_sigma = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('plus_one_sigma')) {
        this.plus_one_sigma = initObj.plus_one_sigma
      }
      else {
        this.plus_one_sigma = new geometry_msgs.msg.Vector3();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VelocityWithSigmaBounds
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [velocity_plus_one_sigma]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.velocity_plus_one_sigma, buffer, bufferOffset);
    // Serialize message field [velocity_minus_one_sigma]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.velocity_minus_one_sigma, buffer, bufferOffset);
    // Serialize message field [plus_one_sigma]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.plus_one_sigma, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VelocityWithSigmaBounds
    let len;
    let data = new VelocityWithSigmaBounds(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [velocity_plus_one_sigma]
    data.velocity_plus_one_sigma = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [velocity_minus_one_sigma]
    data.velocity_minus_one_sigma = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [plus_one_sigma]
    data.plus_one_sigma = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 72;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pronto_msgs/VelocityWithSigmaBounds';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4f91adbed2b2172ae0f78e162a5bc17d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # convenience message encoding the linear velocity
    # one sigma bounds (plus, minus) and the sigma alone
    
    Header header
    geometry_msgs/Vector3 velocity_plus_one_sigma
    geometry_msgs/Vector3 velocity_minus_one_sigma
    geometry_msgs/Vector3 plus_one_sigma
    
    
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new VelocityWithSigmaBounds(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.velocity_plus_one_sigma !== undefined) {
      resolved.velocity_plus_one_sigma = geometry_msgs.msg.Vector3.Resolve(msg.velocity_plus_one_sigma)
    }
    else {
      resolved.velocity_plus_one_sigma = new geometry_msgs.msg.Vector3()
    }

    if (msg.velocity_minus_one_sigma !== undefined) {
      resolved.velocity_minus_one_sigma = geometry_msgs.msg.Vector3.Resolve(msg.velocity_minus_one_sigma)
    }
    else {
      resolved.velocity_minus_one_sigma = new geometry_msgs.msg.Vector3()
    }

    if (msg.plus_one_sigma !== undefined) {
      resolved.plus_one_sigma = geometry_msgs.msg.Vector3.Resolve(msg.plus_one_sigma)
    }
    else {
      resolved.plus_one_sigma = new geometry_msgs.msg.Vector3()
    }

    return resolved;
    }
};

module.exports = VelocityWithSigmaBounds;
