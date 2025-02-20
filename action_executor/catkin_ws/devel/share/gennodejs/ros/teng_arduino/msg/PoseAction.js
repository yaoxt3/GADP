// Auto-generated. Do not edit!

// (in-package teng_arduino.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class PoseAction {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pose_data = null;
      this.action_data = null;
    }
    else {
      if (initObj.hasOwnProperty('pose_data')) {
        this.pose_data = initObj.pose_data
      }
      else {
        this.pose_data = new std_msgs.msg.Float32MultiArray();
      }
      if (initObj.hasOwnProperty('action_data')) {
        this.action_data = initObj.action_data
      }
      else {
        this.action_data = new std_msgs.msg.Float32MultiArray();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PoseAction
    // Serialize message field [pose_data]
    bufferOffset = std_msgs.msg.Float32MultiArray.serialize(obj.pose_data, buffer, bufferOffset);
    // Serialize message field [action_data]
    bufferOffset = std_msgs.msg.Float32MultiArray.serialize(obj.action_data, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PoseAction
    let len;
    let data = new PoseAction(null);
    // Deserialize message field [pose_data]
    data.pose_data = std_msgs.msg.Float32MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [action_data]
    data.action_data = std_msgs.msg.Float32MultiArray.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Float32MultiArray.getMessageSize(object.pose_data);
    length += std_msgs.msg.Float32MultiArray.getMessageSize(object.action_data);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'teng_arduino/PoseAction';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c51c985cde497333deb62314cbacaf1b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Float32MultiArray pose_data
    std_msgs/Float32MultiArray action_data
    
    ================================================================================
    MSG: std_msgs/Float32MultiArray
    # Please look at the MultiArrayLayout message definition for
    # documentation on all multiarrays.
    
    MultiArrayLayout  layout        # specification of data layout
    float32[]         data          # array of data
    
    
    ================================================================================
    MSG: std_msgs/MultiArrayLayout
    # The multiarray declares a generic multi-dimensional array of a
    # particular data type.  Dimensions are ordered from outer most
    # to inner most.
    
    MultiArrayDimension[] dim # Array of dimension properties
    uint32 data_offset        # padding elements at front of data
    
    # Accessors should ALWAYS be written in terms of dimension stride
    # and specified outer-most dimension first.
    # 
    # multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
    #
    # A standard, 3-channel 640x480 image with interleaved color channels
    # would be specified as:
    #
    # dim[0].label  = "height"
    # dim[0].size   = 480
    # dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
    # dim[1].label  = "width"
    # dim[1].size   = 640
    # dim[1].stride = 3*640 = 1920
    # dim[2].label  = "channel"
    # dim[2].size   = 3
    # dim[2].stride = 3
    #
    # multiarray(i,j,k) refers to the ith row, jth column, and kth channel.
    
    ================================================================================
    MSG: std_msgs/MultiArrayDimension
    string label   # label of given dimension
    uint32 size    # size of given dimension (in type units)
    uint32 stride  # stride of given dimension
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PoseAction(null);
    if (msg.pose_data !== undefined) {
      resolved.pose_data = std_msgs.msg.Float32MultiArray.Resolve(msg.pose_data)
    }
    else {
      resolved.pose_data = new std_msgs.msg.Float32MultiArray()
    }

    if (msg.action_data !== undefined) {
      resolved.action_data = std_msgs.msg.Float32MultiArray.Resolve(msg.action_data)
    }
    else {
      resolved.action_data = new std_msgs.msg.Float32MultiArray()
    }

    return resolved;
    }
};

module.exports = PoseAction;
