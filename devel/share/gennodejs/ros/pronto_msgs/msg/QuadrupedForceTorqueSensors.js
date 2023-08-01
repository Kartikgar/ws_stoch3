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

class QuadrupedForceTorqueSensors {
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
        this.lf = new geometry_msgs.msg.Wrench();
      }
      if (initObj.hasOwnProperty('rf')) {
        this.rf = initObj.rf
      }
      else {
        this.rf = new geometry_msgs.msg.Wrench();
      }
      if (initObj.hasOwnProperty('lh')) {
        this.lh = initObj.lh
      }
      else {
        this.lh = new geometry_msgs.msg.Wrench();
      }
      if (initObj.hasOwnProperty('rh')) {
        this.rh = initObj.rh
      }
      else {
        this.rh = new geometry_msgs.msg.Wrench();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type QuadrupedForceTorqueSensors
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [lf]
    bufferOffset = geometry_msgs.msg.Wrench.serialize(obj.lf, buffer, bufferOffset);
    // Serialize message field [rf]
    bufferOffset = geometry_msgs.msg.Wrench.serialize(obj.rf, buffer, bufferOffset);
    // Serialize message field [lh]
    bufferOffset = geometry_msgs.msg.Wrench.serialize(obj.lh, buffer, bufferOffset);
    // Serialize message field [rh]
    bufferOffset = geometry_msgs.msg.Wrench.serialize(obj.rh, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type QuadrupedForceTorqueSensors
    let len;
    let data = new QuadrupedForceTorqueSensors(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [lf]
    data.lf = geometry_msgs.msg.Wrench.deserialize(buffer, bufferOffset);
    // Deserialize message field [rf]
    data.rf = geometry_msgs.msg.Wrench.deserialize(buffer, bufferOffset);
    // Deserialize message field [lh]
    data.lh = geometry_msgs.msg.Wrench.deserialize(buffer, bufferOffset);
    // Deserialize message field [rh]
    data.rh = geometry_msgs.msg.Wrench.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 192;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pronto_msgs/QuadrupedForceTorqueSensors';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '493101afbbcce5b658c33739603c5c3e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Atlas force torque sensors for the wrists and ankles
    Header header
    
    geometry_msgs/Wrench lf
    geometry_msgs/Wrench rf
    geometry_msgs/Wrench lh
    geometry_msgs/Wrench rh
    
    
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
    MSG: geometry_msgs/Wrench
    # This represents force in free space, separated into
    # its linear and angular parts.
    Vector3  force
    Vector3  torque
    
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
    const resolved = new QuadrupedForceTorqueSensors(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.lf !== undefined) {
      resolved.lf = geometry_msgs.msg.Wrench.Resolve(msg.lf)
    }
    else {
      resolved.lf = new geometry_msgs.msg.Wrench()
    }

    if (msg.rf !== undefined) {
      resolved.rf = geometry_msgs.msg.Wrench.Resolve(msg.rf)
    }
    else {
      resolved.rf = new geometry_msgs.msg.Wrench()
    }

    if (msg.lh !== undefined) {
      resolved.lh = geometry_msgs.msg.Wrench.Resolve(msg.lh)
    }
    else {
      resolved.lh = new geometry_msgs.msg.Wrench()
    }

    if (msg.rh !== undefined) {
      resolved.rh = geometry_msgs.msg.Wrench.Resolve(msg.rh)
    }
    else {
      resolved.rh = new geometry_msgs.msg.Wrench()
    }

    return resolved;
    }
};

module.exports = QuadrupedForceTorqueSensors;
