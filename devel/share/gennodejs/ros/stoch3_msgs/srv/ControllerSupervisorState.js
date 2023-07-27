// Auto-generated. Do not edit!

// (in-package stoch3_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class ControllerSupervisorStateRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControllerSupervisorStateRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControllerSupervisorStateRequest
    let len;
    let data = new ControllerSupervisorStateRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'stoch3_msgs/ControllerSupervisorStateRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControllerSupervisorStateRequest(null);
    return resolved;
    }
};

class ControllerSupervisorStateResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.controller_name = null;
      this.seq_num = null;
    }
    else {
      if (initObj.hasOwnProperty('controller_name')) {
        this.controller_name = initObj.controller_name
      }
      else {
        this.controller_name = '';
      }
      if (initObj.hasOwnProperty('seq_num')) {
        this.seq_num = initObj.seq_num
      }
      else {
        this.seq_num = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControllerSupervisorStateResponse
    // Serialize message field [controller_name]
    bufferOffset = _serializer.string(obj.controller_name, buffer, bufferOffset);
    // Serialize message field [seq_num]
    bufferOffset = _serializer.int64(obj.seq_num, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControllerSupervisorStateResponse
    let len;
    let data = new ControllerSupervisorStateResponse(null);
    // Deserialize message field [controller_name]
    data.controller_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [seq_num]
    data.seq_num = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.controller_name.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'stoch3_msgs/ControllerSupervisorStateResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '61f9a8ff34ea0f4986898104d7e20d25';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string controller_name # Name of the current active controller
    int64  seq_num         # Sequence numer of the current active controller
                           # (i.e, number of times the controller update function called since it became active)
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControllerSupervisorStateResponse(null);
    if (msg.controller_name !== undefined) {
      resolved.controller_name = msg.controller_name;
    }
    else {
      resolved.controller_name = ''
    }

    if (msg.seq_num !== undefined) {
      resolved.seq_num = msg.seq_num;
    }
    else {
      resolved.seq_num = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: ControllerSupervisorStateRequest,
  Response: ControllerSupervisorStateResponse,
  md5sum() { return '61f9a8ff34ea0f4986898104d7e20d25'; },
  datatype() { return 'stoch3_msgs/ControllerSupervisorState'; }
};
