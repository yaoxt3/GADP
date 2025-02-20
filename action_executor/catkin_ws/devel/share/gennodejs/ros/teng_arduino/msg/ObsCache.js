// Auto-generated. Do not edit!

// (in-package teng_arduino.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ObsCache {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.stamp = null;
      this.pose_1 = null;
      this.pose_2 = null;
      this.point_cloud_1 = null;
      this.point_cloud_2 = null;
    }
    else {
      if (initObj.hasOwnProperty('stamp')) {
        this.stamp = initObj.stamp
      }
      else {
        this.stamp = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('pose_1')) {
        this.pose_1 = initObj.pose_1
      }
      else {
        this.pose_1 = [];
      }
      if (initObj.hasOwnProperty('pose_2')) {
        this.pose_2 = initObj.pose_2
      }
      else {
        this.pose_2 = [];
      }
      if (initObj.hasOwnProperty('point_cloud_1')) {
        this.point_cloud_1 = initObj.point_cloud_1
      }
      else {
        this.point_cloud_1 = [];
      }
      if (initObj.hasOwnProperty('point_cloud_2')) {
        this.point_cloud_2 = initObj.point_cloud_2
      }
      else {
        this.point_cloud_2 = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ObsCache
    // Serialize message field [stamp]
    bufferOffset = _serializer.time(obj.stamp, buffer, bufferOffset);
    // Serialize message field [pose_1]
    bufferOffset = _arraySerializer.float32(obj.pose_1, buffer, bufferOffset, null);
    // Serialize message field [pose_2]
    bufferOffset = _arraySerializer.float32(obj.pose_2, buffer, bufferOffset, null);
    // Serialize message field [point_cloud_1]
    bufferOffset = _arraySerializer.float32(obj.point_cloud_1, buffer, bufferOffset, null);
    // Serialize message field [point_cloud_2]
    bufferOffset = _arraySerializer.float32(obj.point_cloud_2, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ObsCache
    let len;
    let data = new ObsCache(null);
    // Deserialize message field [stamp]
    data.stamp = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [pose_1]
    data.pose_1 = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [pose_2]
    data.pose_2 = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [point_cloud_1]
    data.point_cloud_1 = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [point_cloud_2]
    data.point_cloud_2 = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.pose_1.length;
    length += 4 * object.pose_2.length;
    length += 4 * object.point_cloud_1.length;
    length += 4 * object.point_cloud_2.length;
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'teng_arduino/ObsCache';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0afcc9dc37d685388aeb48cae3618cb4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    time stamp
    float32[] pose_1
    float32[] pose_2
    float32[] point_cloud_1
    float32[] point_cloud_2
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ObsCache(null);
    if (msg.stamp !== undefined) {
      resolved.stamp = msg.stamp;
    }
    else {
      resolved.stamp = {secs: 0, nsecs: 0}
    }

    if (msg.pose_1 !== undefined) {
      resolved.pose_1 = msg.pose_1;
    }
    else {
      resolved.pose_1 = []
    }

    if (msg.pose_2 !== undefined) {
      resolved.pose_2 = msg.pose_2;
    }
    else {
      resolved.pose_2 = []
    }

    if (msg.point_cloud_1 !== undefined) {
      resolved.point_cloud_1 = msg.point_cloud_1;
    }
    else {
      resolved.point_cloud_1 = []
    }

    if (msg.point_cloud_2 !== undefined) {
      resolved.point_cloud_2 = msg.point_cloud_2;
    }
    else {
      resolved.point_cloud_2 = []
    }

    return resolved;
    }
};

module.exports = ObsCache;
