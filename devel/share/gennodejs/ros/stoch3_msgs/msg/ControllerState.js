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

class ControllerState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.joint_names = null;
      this.act_position = null;
      this.cmd_position = null;
      this.position_error = null;
      this.shifts = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('joint_names')) {
        this.joint_names = initObj.joint_names
      }
      else {
        this.joint_names = [];
      }
      if (initObj.hasOwnProperty('act_position')) {
        this.act_position = initObj.act_position
      }
      else {
        this.act_position = [];
      }
      if (initObj.hasOwnProperty('cmd_position')) {
        this.cmd_position = initObj.cmd_position
      }
      else {
        this.cmd_position = [];
      }
      if (initObj.hasOwnProperty('position_error')) {
        this.position_error = initObj.position_error
      }
      else {
        this.position_error = [];
      }
      if (initObj.hasOwnProperty('shifts')) {
        this.shifts = initObj.shifts
      }
      else {
        this.shifts = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControllerState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [joint_names]
    bufferOffset = _arraySerializer.string(obj.joint_names, buffer, bufferOffset, null);
    // Serialize message field [act_position]
    bufferOffset = _arraySerializer.float64(obj.act_position, buffer, bufferOffset, null);
    // Serialize message field [cmd_position]
    bufferOffset = _arraySerializer.float64(obj.cmd_position, buffer, bufferOffset, null);
    // Serialize message field [position_error]
    bufferOffset = _arraySerializer.float64(obj.position_error, buffer, bufferOffset, null);
    // Serialize message field [shifts]
    bufferOffset = _arraySerializer.float64(obj.shifts, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControllerState
    let len;
    let data = new ControllerState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [joint_names]
    data.joint_names = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [act_position]
    data.act_position = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [cmd_position]
    data.cmd_position = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [position_error]
    data.position_error = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [shifts]
    data.shifts = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.joint_names.forEach((val) => {
      length += 4 + val.length;
    });
    length += 8 * object.act_position.length;
    length += 8 * object.cmd_position.length;
    length += 8 * object.position_error.length;
    length += 8 * object.shifts.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'stoch3_msgs/ControllerState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd913fd7c30d2891f22fc139634144c49';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    string[] joint_names
    float64[] act_position # Actual position
    float64[] cmd_position # Commanded position
    float64[] position_error # Position error
    float64[] shifts # leg shifts from linear policy
    
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
    const resolved = new ControllerState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.joint_names !== undefined) {
      resolved.joint_names = msg.joint_names;
    }
    else {
      resolved.joint_names = []
    }

    if (msg.act_position !== undefined) {
      resolved.act_position = msg.act_position;
    }
    else {
      resolved.act_position = []
    }

    if (msg.cmd_position !== undefined) {
      resolved.cmd_position = msg.cmd_position;
    }
    else {
      resolved.cmd_position = []
    }

    if (msg.position_error !== undefined) {
      resolved.position_error = msg.position_error;
    }
    else {
      resolved.position_error = []
    }

    if (msg.shifts !== undefined) {
      resolved.shifts = msg.shifts;
    }
    else {
      resolved.shifts = []
    }

    return resolved;
    }
};

module.exports = ControllerState;
