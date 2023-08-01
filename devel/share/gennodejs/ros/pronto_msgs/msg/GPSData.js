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

class GPSData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.utime = null;
      this.gps_lock = null;
      this.longitude = null;
      this.latitude = null;
      this.elev = null;
      this.horizontal_accuracy = null;
      this.vertical_accuracy = null;
      this.num_satellites = null;
      this.speed = null;
      this.heading = null;
      this.xyz_pos = null;
      this.gps_time = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('utime')) {
        this.utime = initObj.utime
      }
      else {
        this.utime = 0;
      }
      if (initObj.hasOwnProperty('gps_lock')) {
        this.gps_lock = initObj.gps_lock
      }
      else {
        this.gps_lock = 0;
      }
      if (initObj.hasOwnProperty('longitude')) {
        this.longitude = initObj.longitude
      }
      else {
        this.longitude = 0.0;
      }
      if (initObj.hasOwnProperty('latitude')) {
        this.latitude = initObj.latitude
      }
      else {
        this.latitude = 0.0;
      }
      if (initObj.hasOwnProperty('elev')) {
        this.elev = initObj.elev
      }
      else {
        this.elev = 0.0;
      }
      if (initObj.hasOwnProperty('horizontal_accuracy')) {
        this.horizontal_accuracy = initObj.horizontal_accuracy
      }
      else {
        this.horizontal_accuracy = 0.0;
      }
      if (initObj.hasOwnProperty('vertical_accuracy')) {
        this.vertical_accuracy = initObj.vertical_accuracy
      }
      else {
        this.vertical_accuracy = 0.0;
      }
      if (initObj.hasOwnProperty('num_satellites')) {
        this.num_satellites = initObj.num_satellites
      }
      else {
        this.num_satellites = 0;
      }
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0.0;
      }
      if (initObj.hasOwnProperty('heading')) {
        this.heading = initObj.heading
      }
      else {
        this.heading = 0.0;
      }
      if (initObj.hasOwnProperty('xyz_pos')) {
        this.xyz_pos = initObj.xyz_pos
      }
      else {
        this.xyz_pos = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('gps_time')) {
        this.gps_time = initObj.gps_time
      }
      else {
        this.gps_time = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GPSData
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [utime]
    bufferOffset = _serializer.uint64(obj.utime, buffer, bufferOffset);
    // Serialize message field [gps_lock]
    bufferOffset = _serializer.int32(obj.gps_lock, buffer, bufferOffset);
    // Serialize message field [longitude]
    bufferOffset = _serializer.float64(obj.longitude, buffer, bufferOffset);
    // Serialize message field [latitude]
    bufferOffset = _serializer.float64(obj.latitude, buffer, bufferOffset);
    // Serialize message field [elev]
    bufferOffset = _serializer.float64(obj.elev, buffer, bufferOffset);
    // Serialize message field [horizontal_accuracy]
    bufferOffset = _serializer.float64(obj.horizontal_accuracy, buffer, bufferOffset);
    // Serialize message field [vertical_accuracy]
    bufferOffset = _serializer.float64(obj.vertical_accuracy, buffer, bufferOffset);
    // Serialize message field [num_satellites]
    bufferOffset = _serializer.uint32(obj.num_satellites, buffer, bufferOffset);
    // Serialize message field [speed]
    bufferOffset = _serializer.float64(obj.speed, buffer, bufferOffset);
    // Serialize message field [heading]
    bufferOffset = _serializer.float64(obj.heading, buffer, bufferOffset);
    // Check that the constant length array field [xyz_pos] has the right length
    if (obj.xyz_pos.length !== 3) {
      throw new Error('Unable to serialize array field xyz_pos - length must be 3')
    }
    // Serialize message field [xyz_pos]
    bufferOffset = _arraySerializer.float64(obj.xyz_pos, buffer, bufferOffset, 3);
    // Serialize message field [gps_time]
    bufferOffset = _serializer.float64(obj.gps_time, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GPSData
    let len;
    let data = new GPSData(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [utime]
    data.utime = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [gps_lock]
    data.gps_lock = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [longitude]
    data.longitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [latitude]
    data.latitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [elev]
    data.elev = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [horizontal_accuracy]
    data.horizontal_accuracy = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [vertical_accuracy]
    data.vertical_accuracy = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [num_satellites]
    data.num_satellites = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [speed]
    data.speed = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [heading]
    data.heading = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [xyz_pos]
    data.xyz_pos = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [gps_time]
    data.gps_time = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 104;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pronto_msgs/GPSData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '79674b2d1f8526159c0504accff3f259';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    uint64 utime
    int32 gps_lock
    float64 longitude
    float64 latitude
    float64 elev
    float64 horizontal_accuracy
    float64 vertical_accuracy
    uint32 num_satellites
    float64 speed
    float64 heading
    float64[3] xyz_pos
    float64 gps_time
    
    
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
    const resolved = new GPSData(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.utime !== undefined) {
      resolved.utime = msg.utime;
    }
    else {
      resolved.utime = 0
    }

    if (msg.gps_lock !== undefined) {
      resolved.gps_lock = msg.gps_lock;
    }
    else {
      resolved.gps_lock = 0
    }

    if (msg.longitude !== undefined) {
      resolved.longitude = msg.longitude;
    }
    else {
      resolved.longitude = 0.0
    }

    if (msg.latitude !== undefined) {
      resolved.latitude = msg.latitude;
    }
    else {
      resolved.latitude = 0.0
    }

    if (msg.elev !== undefined) {
      resolved.elev = msg.elev;
    }
    else {
      resolved.elev = 0.0
    }

    if (msg.horizontal_accuracy !== undefined) {
      resolved.horizontal_accuracy = msg.horizontal_accuracy;
    }
    else {
      resolved.horizontal_accuracy = 0.0
    }

    if (msg.vertical_accuracy !== undefined) {
      resolved.vertical_accuracy = msg.vertical_accuracy;
    }
    else {
      resolved.vertical_accuracy = 0.0
    }

    if (msg.num_satellites !== undefined) {
      resolved.num_satellites = msg.num_satellites;
    }
    else {
      resolved.num_satellites = 0
    }

    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0.0
    }

    if (msg.heading !== undefined) {
      resolved.heading = msg.heading;
    }
    else {
      resolved.heading = 0.0
    }

    if (msg.xyz_pos !== undefined) {
      resolved.xyz_pos = msg.xyz_pos;
    }
    else {
      resolved.xyz_pos = new Array(3).fill(0)
    }

    if (msg.gps_time !== undefined) {
      resolved.gps_time = msg.gps_time;
    }
    else {
      resolved.gps_time = 0.0
    }

    return resolved;
    }
};

module.exports = GPSData;
