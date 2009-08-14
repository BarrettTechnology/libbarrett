; Auto-generated. Do not edit!


(in-package wam_ros-msg)


;//! \htmlinclude Joints.msg.html

(defclass <Joints> (ros-message)
  ((j
    :accessor j-val
    :initarg :j
    :initform ""))
)
(defmethod serialize ((msg <Joints>) ostream)
  "Serializes a message object of type '<Joints>"
  (let ((__ros_str_len (length (slot-value msg 'j))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'j))
)
(defmethod deserialize ((msg <Joints>) istream)
  "Deserializes a message object of type '<Joints>"
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'j) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'j) __ros_str_idx) (code-char (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<Joints>)))
  "Returns string type for a message object of type '<Joints>"
  "wam_ros/Joints")
(defmethod md5sum ((type (eql '<Joints>)))
  "Returns md5sum for a message object of type '<Joints>"
  #x0faeaaa42c2070611310a54ecba6c3ef)
(defmethod message-definition ((type (eql '<Joints>)))
  "Returns full string definition for message of type '<Joints>"
  (format nil "string j~%~%~%"))
(defmethod serialization-length ((msg <Joints>))
  (+ 0
     4 (length (slot-value msg 'j))
))
(defmethod ros-message-to-list ((msg <Joints>))
  "Converts a ROS message object to a list"
  (list '<Joints>
    (cons ':j (ros-message-to-list (j-val msg)))
))
