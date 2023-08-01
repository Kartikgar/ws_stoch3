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

class ControllerFootContact {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.num_right_foot_contacts = null;
      this.right_foot_contacts = null;
      this.num_left_foot_contacts = null;
      this.left_foot_contacts = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('num_right_foot_contacts')) {
        this.num_right_foot_contacts = initObj.num_right_foot_contacts
      }
      else {
        this.num_right_foot_contacts = 0;
      }
      if (initObj.hasOwnProperty('right_foot_contacts')) {
        this.right_foot_contacts = initObj.right_foot_contacts
      }
      else {
        this.right_foot_contacts = [];
      }
      if (initObj.hasOwnProperty('num_left_foot_contacts')) {
        this.num_left_foot_contacts = initObj.num_left_foot_contacts
      }
      else {
        this.num_left_foot_contacts = 0;
      }
      if (initObj.hasOwnProperty('left_foot_contacts')) {
        this.left_foot_contacts = initObj.left_foot_contacts
      }
      else {
        this.left_foot_contacts = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControllerFootContact
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [num_right_foot_contacts]
    bufferOffset = _serializer.int32(obj.num_right_foot_contacts, buffer, bufferOffset);
    // Serialize message field [right_foot_contacts]
    bufferOffset = _arraySerializer.float64(obj.right_foot_contacts, buffer, bufferOffset, null);
    // Serialize message field [num_left_foot_contacts]
    bufferOffset = _serializer.int32(obj.num_left_foot_contacts, buffer, bufferOffset);
    // Serialize message field [left_foot_contacts]
    bufferOffset = _arraySerializer.float64(obj.left_foot_contacts, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControllerFootContact
    let len;
    let data = new ControllerFootContact(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [num_right_foot_contacts]
    data.num_right_foot_contacts = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [right_foot_contacts]
    data.right_foot_contacts = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [num_left_foot_contacts]
    data.num_left_foot_contacts = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [left_foot_contacts]
    data.left_foot_contacts = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 8 * object.right_foot_contacts.length;
    length += 8 * object.left_foot_contacts.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pronto_msgs/ControllerFootContact';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ac4086dd5e9ba23a62d9963e591cc896';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    int32 num_right_foot_contacts
    
    # Porting from LCM definition:
    # double right_foot_contacts[3][num_right_foot_contacts]
    float64[] right_foot_contacts
    
    int32 num_left_foot_contacts
    
    # Porting from LCM definition:
    # double left_foot_contacts[3][num_left_foot_contacts];
    float64[] left_foot_contacts
    
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
    const resolved = new ControllerFootContact(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.num_right_foot_contacts !== undefined) {
      resolved.num_right_foot_contacts = msg.num_right_foot_contacts;
    }
    else {
      resolved.num_right_foot_contacts = 0
    }

    if (msg.right_foot_contacts !== undefined) {
      resolved.right_foot_contacts = msg.right_foot_contacts;
    }
    else {
      resolved.right_foot_contacts = []
    }

    if (msg.num_left_foot_contacts !== undefined) {
      resolved.num_left_foot_contacts = msg.num_left_foot_contacts;
    }
    else {
      resolved.num_left_foot_contacts = 0
    }

    if (msg.left_foot_contacts !== undefined) {
      resolved.left_foot_contacts = msg.left_foot_contacts;
    }
    else {
      resolved.left_foot_contacts = []
    }

    return resolved;
    }
};

module.exports = ControllerFootContact;
