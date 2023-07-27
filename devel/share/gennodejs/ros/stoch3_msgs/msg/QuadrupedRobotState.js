// Auto-generated. Do not edit!

// (in-package stoch3_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let LegState = require('./LegState.js');
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class QuadrupedRobotState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.pose = null;
      this.twist = null;
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
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('twist')) {
        this.twist = initObj.twist
      }
      else {
        this.twist = new geometry_msgs.msg.Twist();
      }
      if (initObj.hasOwnProperty('fl')) {
        this.fl = initObj.fl
      }
      else {
        this.fl = new LegState();
      }
      if (initObj.hasOwnProperty('fr')) {
        this.fr = initObj.fr
      }
      else {
        this.fr = new LegState();
      }
      if (initObj.hasOwnProperty('bl')) {
        this.bl = initObj.bl
      }
      else {
        this.bl = new LegState();
      }
      if (initObj.hasOwnProperty('br')) {
        this.br = initObj.br
      }
      else {
        this.br = new LegState();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type QuadrupedRobotState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.pose, buffer, bufferOffset);
    // Serialize message field [twist]
    bufferOffset = geometry_msgs.msg.Twist.serialize(obj.twist, buffer, bufferOffset);
    // Serialize message field [fl]
    bufferOffset = LegState.serialize(obj.fl, buffer, bufferOffset);
    // Serialize message field [fr]
    bufferOffset = LegState.serialize(obj.fr, buffer, bufferOffset);
    // Serialize message field [bl]
    bufferOffset = LegState.serialize(obj.bl, buffer, bufferOffset);
    // Serialize message field [br]
    bufferOffset = LegState.serialize(obj.br, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type QuadrupedRobotState
    let len;
    let data = new QuadrupedRobotState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [twist]
    data.twist = geometry_msgs.msg.Twist.deserialize(buffer, bufferOffset);
    // Deserialize message field [fl]
    data.fl = LegState.deserialize(buffer, bufferOffset);
    // Deserialize message field [fr]
    data.fr = LegState.deserialize(buffer, bufferOffset);
    // Deserialize message field [bl]
    data.bl = LegState.deserialize(buffer, bufferOffset);
    // Deserialize message field [br]
    data.br = LegState.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += LegState.getMessageSize(object.fl);
    length += LegState.getMessageSize(object.fr);
    length += LegState.getMessageSize(object.bl);
    length += LegState.getMessageSize(object.br);
    return length + 104;
  }

  static datatype() {
    // Returns string type for a message object
    return 'stoch3_msgs/QuadrupedRobotState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c0b1dcdf6adea2ab6ab0d9d2dab7e7de';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    geometry_msgs/Pose pose
    geometry_msgs/Twist twist
    stoch3_msgs/LegState fl
    stoch3_msgs/LegState fr
    stoch3_msgs/LegState bl
    stoch3_msgs/LegState br
    
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
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
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
    ================================================================================
    MSG: stoch3_msgs/LegState
    string name
    geometry_msgs/Vector3 position
    geometry_msgs/Vector3 velocity
    geometry_msgs/Vector3 force
    float64 support_probability        # [0, 1] , probability that the leg is a support leg.
                                       # Limit the value to range [0, 1] if it is outside the range.
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new QuadrupedRobotState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.Pose.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.Pose()
    }

    if (msg.twist !== undefined) {
      resolved.twist = geometry_msgs.msg.Twist.Resolve(msg.twist)
    }
    else {
      resolved.twist = new geometry_msgs.msg.Twist()
    }

    if (msg.fl !== undefined) {
      resolved.fl = LegState.Resolve(msg.fl)
    }
    else {
      resolved.fl = new LegState()
    }

    if (msg.fr !== undefined) {
      resolved.fr = LegState.Resolve(msg.fr)
    }
    else {
      resolved.fr = new LegState()
    }

    if (msg.bl !== undefined) {
      resolved.bl = LegState.Resolve(msg.bl)
    }
    else {
      resolved.bl = new LegState()
    }

    if (msg.br !== undefined) {
      resolved.br = LegState.Resolve(msg.br)
    }
    else {
      resolved.br = new LegState()
    }

    return resolved;
    }
};

module.exports = QuadrupedRobotState;
