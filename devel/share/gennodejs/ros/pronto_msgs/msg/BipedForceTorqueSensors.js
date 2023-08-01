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

class BipedForceTorqueSensors {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.l_foot = null;
      this.r_foot = null;
      this.l_hand = null;
      this.r_hand = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('l_foot')) {
        this.l_foot = initObj.l_foot
      }
      else {
        this.l_foot = new geometry_msgs.msg.Wrench();
      }
      if (initObj.hasOwnProperty('r_foot')) {
        this.r_foot = initObj.r_foot
      }
      else {
        this.r_foot = new geometry_msgs.msg.Wrench();
      }
      if (initObj.hasOwnProperty('l_hand')) {
        this.l_hand = initObj.l_hand
      }
      else {
        this.l_hand = new geometry_msgs.msg.Wrench();
      }
      if (initObj.hasOwnProperty('r_hand')) {
        this.r_hand = initObj.r_hand
      }
      else {
        this.r_hand = new geometry_msgs.msg.Wrench();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BipedForceTorqueSensors
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [l_foot]
    bufferOffset = geometry_msgs.msg.Wrench.serialize(obj.l_foot, buffer, bufferOffset);
    // Serialize message field [r_foot]
    bufferOffset = geometry_msgs.msg.Wrench.serialize(obj.r_foot, buffer, bufferOffset);
    // Serialize message field [l_hand]
    bufferOffset = geometry_msgs.msg.Wrench.serialize(obj.l_hand, buffer, bufferOffset);
    // Serialize message field [r_hand]
    bufferOffset = geometry_msgs.msg.Wrench.serialize(obj.r_hand, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BipedForceTorqueSensors
    let len;
    let data = new BipedForceTorqueSensors(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [l_foot]
    data.l_foot = geometry_msgs.msg.Wrench.deserialize(buffer, bufferOffset);
    // Deserialize message field [r_foot]
    data.r_foot = geometry_msgs.msg.Wrench.deserialize(buffer, bufferOffset);
    // Deserialize message field [l_hand]
    data.l_hand = geometry_msgs.msg.Wrench.deserialize(buffer, bufferOffset);
    // Deserialize message field [r_hand]
    data.r_hand = geometry_msgs.msg.Wrench.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 192;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pronto_msgs/BipedForceTorqueSensors';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '608f43cae98f77f5fb6cfbbbfa4c9253';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Atlas force torque sensors for the wrists and ankles
    Header header
    
    geometry_msgs/Wrench l_foot
    geometry_msgs/Wrench r_foot
    geometry_msgs/Wrench l_hand
    geometry_msgs/Wrench r_hand
    
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
    const resolved = new BipedForceTorqueSensors(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.l_foot !== undefined) {
      resolved.l_foot = geometry_msgs.msg.Wrench.Resolve(msg.l_foot)
    }
    else {
      resolved.l_foot = new geometry_msgs.msg.Wrench()
    }

    if (msg.r_foot !== undefined) {
      resolved.r_foot = geometry_msgs.msg.Wrench.Resolve(msg.r_foot)
    }
    else {
      resolved.r_foot = new geometry_msgs.msg.Wrench()
    }

    if (msg.l_hand !== undefined) {
      resolved.l_hand = geometry_msgs.msg.Wrench.Resolve(msg.l_hand)
    }
    else {
      resolved.l_hand = new geometry_msgs.msg.Wrench()
    }

    if (msg.r_hand !== undefined) {
      resolved.r_hand = geometry_msgs.msg.Wrench.Resolve(msg.r_hand)
    }
    else {
      resolved.r_hand = new geometry_msgs.msg.Wrench()
    }

    return resolved;
    }
};

module.exports = BipedForceTorqueSensors;
