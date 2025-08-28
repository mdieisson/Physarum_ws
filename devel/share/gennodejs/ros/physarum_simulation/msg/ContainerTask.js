// Auto-generated. Do not edit!

// (in-package physarum_simulation.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ContainerTask {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.x = null;
      this.y = null;
      this.coleta_x = null;
      this.coleta_y = null;
      this.dest_x = null;
      this.dest_y = null;
      this.status = null;
      this.robot_id = null;
      this.timestamp = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = '';
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('coleta_x')) {
        this.coleta_x = initObj.coleta_x
      }
      else {
        this.coleta_x = 0.0;
      }
      if (initObj.hasOwnProperty('coleta_y')) {
        this.coleta_y = initObj.coleta_y
      }
      else {
        this.coleta_y = 0.0;
      }
      if (initObj.hasOwnProperty('dest_x')) {
        this.dest_x = initObj.dest_x
      }
      else {
        this.dest_x = 0.0;
      }
      if (initObj.hasOwnProperty('dest_y')) {
        this.dest_y = initObj.dest_y
      }
      else {
        this.dest_y = 0.0;
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = '';
      }
      if (initObj.hasOwnProperty('robot_id')) {
        this.robot_id = initObj.robot_id
      }
      else {
        this.robot_id = '';
      }
      if (initObj.hasOwnProperty('timestamp')) {
        this.timestamp = initObj.timestamp
      }
      else {
        this.timestamp = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ContainerTask
    // Serialize message field [id]
    bufferOffset = _serializer.string(obj.id, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.float32(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float32(obj.y, buffer, bufferOffset);
    // Serialize message field [coleta_x]
    bufferOffset = _serializer.float32(obj.coleta_x, buffer, bufferOffset);
    // Serialize message field [coleta_y]
    bufferOffset = _serializer.float32(obj.coleta_y, buffer, bufferOffset);
    // Serialize message field [dest_x]
    bufferOffset = _serializer.float32(obj.dest_x, buffer, bufferOffset);
    // Serialize message field [dest_y]
    bufferOffset = _serializer.float32(obj.dest_y, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = _serializer.string(obj.status, buffer, bufferOffset);
    // Serialize message field [robot_id]
    bufferOffset = _serializer.string(obj.robot_id, buffer, bufferOffset);
    // Serialize message field [timestamp]
    bufferOffset = _serializer.float64(obj.timestamp, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ContainerTask
    let len;
    let data = new ContainerTask(null);
    // Deserialize message field [id]
    data.id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [coleta_x]
    data.coleta_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [coleta_y]
    data.coleta_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [dest_x]
    data.dest_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [dest_y]
    data.dest_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [robot_id]
    data.robot_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [timestamp]
    data.timestamp = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.id);
    length += _getByteLength(object.status);
    length += _getByteLength(object.robot_id);
    return length + 44;
  }

  static datatype() {
    // Returns string type for a message object
    return 'physarum_simulation/ContainerTask';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4ac7e3eef6a945017db3fb1512eda31d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string id
    float32 x
    float32 y
    float32 coleta_x     # novo
    float32 coleta_y     # novo
    float32 dest_x
    float32 dest_y
    string status
    string robot_id
    float64 timestamp
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ContainerTask(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = ''
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.coleta_x !== undefined) {
      resolved.coleta_x = msg.coleta_x;
    }
    else {
      resolved.coleta_x = 0.0
    }

    if (msg.coleta_y !== undefined) {
      resolved.coleta_y = msg.coleta_y;
    }
    else {
      resolved.coleta_y = 0.0
    }

    if (msg.dest_x !== undefined) {
      resolved.dest_x = msg.dest_x;
    }
    else {
      resolved.dest_x = 0.0
    }

    if (msg.dest_y !== undefined) {
      resolved.dest_y = msg.dest_y;
    }
    else {
      resolved.dest_y = 0.0
    }

    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = ''
    }

    if (msg.robot_id !== undefined) {
      resolved.robot_id = msg.robot_id;
    }
    else {
      resolved.robot_id = ''
    }

    if (msg.timestamp !== undefined) {
      resolved.timestamp = msg.timestamp;
    }
    else {
      resolved.timestamp = 0.0
    }

    return resolved;
    }
};

module.exports = ContainerTask;
