// Auto-generated. Do not edit!

// (in-package stoch3_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class GaitRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.step_frequency = null;
      this.swing_height = null;
      this.stance_height = null;
      this.max_vel = null;
    }
    else {
      if (initObj.hasOwnProperty('step_frequency')) {
        this.step_frequency = initObj.step_frequency
      }
      else {
        this.step_frequency = 0.0;
      }
      if (initObj.hasOwnProperty('swing_height')) {
        this.swing_height = initObj.swing_height
      }
      else {
        this.swing_height = 0.0;
      }
      if (initObj.hasOwnProperty('stance_height')) {
        this.stance_height = initObj.stance_height
      }
      else {
        this.stance_height = 0.0;
      }
      if (initObj.hasOwnProperty('max_vel')) {
        this.max_vel = initObj.max_vel
      }
      else {
        this.max_vel = new geometry_msgs.msg.Twist();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GaitRequest
    // Serialize message field [step_frequency]
    bufferOffset = _serializer.float64(obj.step_frequency, buffer, bufferOffset);
    // Serialize message field [swing_height]
    bufferOffset = _serializer.float64(obj.swing_height, buffer, bufferOffset);
    // Serialize message field [stance_height]
    bufferOffset = _serializer.float64(obj.stance_height, buffer, bufferOffset);
    // Serialize message field [max_vel]
    bufferOffset = geometry_msgs.msg.Twist.serialize(obj.max_vel, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GaitRequest
    let len;
    let data = new GaitRequest(null);
    // Deserialize message field [step_frequency]
    data.step_frequency = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [swing_height]
    data.swing_height = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [stance_height]
    data.stance_height = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [max_vel]
    data.max_vel = geometry_msgs.msg.Twist.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 72;
  }

  static datatype() {
    // Returns string type for a service object
    return 'stoch3_msgs/GaitRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ed56ce313d28e93dd09cffbe6ef6c08d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 step_frequency
    float64 swing_height
    float64 stance_height
    geometry_msgs/Twist max_vel
    
    ================================================================================
    MSG: geometry_msgs/Twist
    # This expresses velocity in free space broken into its linear and angular parts.
    Vector3  linear
    Vector3  angular
    
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
    const resolved = new GaitRequest(null);
    if (msg.step_frequency !== undefined) {
      resolved.step_frequency = msg.step_frequency;
    }
    else {
      resolved.step_frequency = 0.0
    }

    if (msg.swing_height !== undefined) {
      resolved.swing_height = msg.swing_height;
    }
    else {
      resolved.swing_height = 0.0
    }

    if (msg.stance_height !== undefined) {
      resolved.stance_height = msg.stance_height;
    }
    else {
      resolved.stance_height = 0.0
    }

    if (msg.max_vel !== undefined) {
      resolved.max_vel = geometry_msgs.msg.Twist.Resolve(msg.max_vel)
    }
    else {
      resolved.max_vel = new geometry_msgs.msg.Twist()
    }

    return resolved;
    }
};

class GaitResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ok = null;
    }
    else {
      if (initObj.hasOwnProperty('ok')) {
        this.ok = initObj.ok
      }
      else {
        this.ok = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GaitResponse
    // Serialize message field [ok]
    bufferOffset = _serializer.bool(obj.ok, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GaitResponse
    let len;
    let data = new GaitResponse(null);
    // Deserialize message field [ok]
    data.ok = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'stoch3_msgs/GaitResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6f6da3883749771fac40d6deb24a8c02';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool ok
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GaitResponse(null);
    if (msg.ok !== undefined) {
      resolved.ok = msg.ok;
    }
    else {
      resolved.ok = false
    }

    return resolved;
    }
};

module.exports = {
  Request: GaitRequest,
  Response: GaitResponse,
  md5sum() { return 'e17f1b3fe4a600b5b71bcddef43f4905'; },
  datatype() { return 'stoch3_msgs/Gait'; }
};
