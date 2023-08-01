// Auto-generated. Do not edit!

// (in-package fovis_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Stats {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.num_matches = null;
      this.num_inliers = null;
      this.num_detected_keypoints = null;
      this.num_reprojection_failures = null;
      this.num_keypoints = null;
      this.mean_reprojection_error = null;
      this.fast_threshold = null;
      this.status = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('num_matches')) {
        this.num_matches = initObj.num_matches
      }
      else {
        this.num_matches = 0;
      }
      if (initObj.hasOwnProperty('num_inliers')) {
        this.num_inliers = initObj.num_inliers
      }
      else {
        this.num_inliers = 0;
      }
      if (initObj.hasOwnProperty('num_detected_keypoints')) {
        this.num_detected_keypoints = initObj.num_detected_keypoints
      }
      else {
        this.num_detected_keypoints = 0;
      }
      if (initObj.hasOwnProperty('num_reprojection_failures')) {
        this.num_reprojection_failures = initObj.num_reprojection_failures
      }
      else {
        this.num_reprojection_failures = 0;
      }
      if (initObj.hasOwnProperty('num_keypoints')) {
        this.num_keypoints = initObj.num_keypoints
      }
      else {
        this.num_keypoints = 0;
      }
      if (initObj.hasOwnProperty('mean_reprojection_error')) {
        this.mean_reprojection_error = initObj.mean_reprojection_error
      }
      else {
        this.mean_reprojection_error = 0.0;
      }
      if (initObj.hasOwnProperty('fast_threshold')) {
        this.fast_threshold = initObj.fast_threshold
      }
      else {
        this.fast_threshold = 0.0;
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Stats
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [num_matches]
    bufferOffset = _serializer.int32(obj.num_matches, buffer, bufferOffset);
    // Serialize message field [num_inliers]
    bufferOffset = _serializer.int32(obj.num_inliers, buffer, bufferOffset);
    // Serialize message field [num_detected_keypoints]
    bufferOffset = _serializer.int32(obj.num_detected_keypoints, buffer, bufferOffset);
    // Serialize message field [num_reprojection_failures]
    bufferOffset = _serializer.int32(obj.num_reprojection_failures, buffer, bufferOffset);
    // Serialize message field [num_keypoints]
    bufferOffset = _serializer.int32(obj.num_keypoints, buffer, bufferOffset);
    // Serialize message field [mean_reprojection_error]
    bufferOffset = _serializer.float64(obj.mean_reprojection_error, buffer, bufferOffset);
    // Serialize message field [fast_threshold]
    bufferOffset = _serializer.float64(obj.fast_threshold, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = _serializer.uint8(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Stats
    let len;
    let data = new Stats(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [num_matches]
    data.num_matches = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [num_inliers]
    data.num_inliers = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [num_detected_keypoints]
    data.num_detected_keypoints = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [num_reprojection_failures]
    data.num_reprojection_failures = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [num_keypoints]
    data.num_keypoints = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [mean_reprojection_error]
    data.mean_reprojection_error = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [fast_threshold]
    data.fast_threshold = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 37;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fovis_msgs/Stats';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dbaf83c8b3afd2954f468abc4e36c223';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # state of fovis
    # same as fovis::stats_t lcm message
    Header header
    
    int32 num_matches
    int32 num_inliers
    int32 num_detected_keypoints
    int32 num_reprojection_failures
    
    # keypoints after pruning
    int32 num_keypoints
    
    float64 mean_reprojection_error
    float64 fast_threshold
    
    uint8 STATUS_CODE_NO_DATA = 0
    uint8 STATUS_CODE_SUCCESS = 1
    uint8 STATUS_CODE_INSUFFICIENT_INLIERS = 2
    uint8 STATUS_CODE_OPTIMIZATION_FAILURE = 3
    uint8 STATUS_CODE_REPROJECTION_ERROR = 4
    
    uint8 status
    
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
    const resolved = new Stats(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.num_matches !== undefined) {
      resolved.num_matches = msg.num_matches;
    }
    else {
      resolved.num_matches = 0
    }

    if (msg.num_inliers !== undefined) {
      resolved.num_inliers = msg.num_inliers;
    }
    else {
      resolved.num_inliers = 0
    }

    if (msg.num_detected_keypoints !== undefined) {
      resolved.num_detected_keypoints = msg.num_detected_keypoints;
    }
    else {
      resolved.num_detected_keypoints = 0
    }

    if (msg.num_reprojection_failures !== undefined) {
      resolved.num_reprojection_failures = msg.num_reprojection_failures;
    }
    else {
      resolved.num_reprojection_failures = 0
    }

    if (msg.num_keypoints !== undefined) {
      resolved.num_keypoints = msg.num_keypoints;
    }
    else {
      resolved.num_keypoints = 0
    }

    if (msg.mean_reprojection_error !== undefined) {
      resolved.mean_reprojection_error = msg.mean_reprojection_error;
    }
    else {
      resolved.mean_reprojection_error = 0.0
    }

    if (msg.fast_threshold !== undefined) {
      resolved.fast_threshold = msg.fast_threshold;
    }
    else {
      resolved.fast_threshold = 0.0
    }

    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = 0
    }

    return resolved;
    }
};

// Constants for message
Stats.Constants = {
  STATUS_CODE_NO_DATA: 0,
  STATUS_CODE_SUCCESS: 1,
  STATUS_CODE_INSUFFICIENT_INLIERS: 2,
  STATUS_CODE_OPTIMIZATION_FAILURE: 3,
  STATUS_CODE_REPROJECTION_ERROR: 4,
}

module.exports = Stats;
