// Auto-generated. Do not edit!

// (in-package stoch3_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let LegCommand = require('./LegCommand.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class QuadrupedLegCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.fl = null;
      this.fr = null;
      this.bl = null;
      this.br = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('fl')) {
        this.fl = initObj.fl
      }
      else {
        this.fl = new LegCommand();
      }
      if (initObj.hasOwnProperty('fr')) {
        this.fr = initObj.fr
      }
      else {
        this.fr = new LegCommand();
      }
      if (initObj.hasOwnProperty('bl')) {
        this.bl = initObj.bl
      }
      else {
        this.bl = new LegCommand();
      }
      if (initObj.hasOwnProperty('br')) {
        this.br = initObj.br
      }
      else {
        this.br = new LegCommand();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type QuadrupedLegCommand
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [fl]
    bufferOffset = LegCommand.serialize(obj.fl, buffer, bufferOffset);
    // Serialize message field [fr]
    bufferOffset = LegCommand.serialize(obj.fr, buffer, bufferOffset);
    // Serialize message field [bl]
    bufferOffset = LegCommand.serialize(obj.bl, buffer, bufferOffset);
    // Serialize message field [br]
    bufferOffset = LegCommand.serialize(obj.br, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type QuadrupedLegCommand
    let len;
    let data = new QuadrupedLegCommand(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [fl]
    data.fl = LegCommand.deserialize(buffer, bufferOffset);
    // Deserialize message field [fr]
    data.fr = LegCommand.deserialize(buffer, bufferOffset);
    // Deserialize message field [bl]
    data.bl = LegCommand.deserialize(buffer, bufferOffset);
    // Deserialize message field [br]
    data.br = LegCommand.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 352;
  }

  static datatype() {
    // Returns string type for a message object
    return 'stoch3_msgs/QuadrupedLegCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bfe7d96dc958adfec0e70eda22cb9e0a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header    header
    stoch3_msgs/LegCommand fl
    stoch3_msgs/LegCommand fr
    stoch3_msgs/LegCommand bl
    stoch3_msgs/LegCommand br
    
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
    MSG: stoch3_msgs/LegCommand
    geometry_msgs/Vector3 position   # Position of foot
    geometry_msgs/Vector3 velocity   # Velocity of foot
    geometry_msgs/Vector3 force      # Force exerted by foot on environment
    float64 kp_scale
    float64 kd_scale
    
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
    const resolved = new QuadrupedLegCommand(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.fl !== undefined) {
      resolved.fl = LegCommand.Resolve(msg.fl)
    }
    else {
      resolved.fl = new LegCommand()
    }

    if (msg.fr !== undefined) {
      resolved.fr = LegCommand.Resolve(msg.fr)
    }
    else {
      resolved.fr = new LegCommand()
    }

    if (msg.bl !== undefined) {
      resolved.bl = LegCommand.Resolve(msg.bl)
    }
    else {
      resolved.bl = new LegCommand()
    }

    if (msg.br !== undefined) {
      resolved.br = LegCommand.Resolve(msg.br)
    }
    else {
      resolved.br = new LegCommand()
    }

    return resolved;
    }
};

module.exports = QuadrupedLegCommand;
