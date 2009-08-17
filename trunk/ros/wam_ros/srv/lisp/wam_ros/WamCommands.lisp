; Auto-generated. Do not edit!


(in-package wam_ros-srv)


;//! \htmlinclude WamCommands-request.msg.html

(defclass <WamCommands-request> (ros-message)
  ((command
    :accessor command-val
    :initarg :command
    :initform 0))
)
(defmethod serialize ((msg <WamCommands-request>) ostream)
  "Serializes a message object of type '<WamCommands-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'command)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'command)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'command)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'command)) ostream)
  (write-byte (ldb (byte 8 32) (slot-value msg 'command)) ostream)
  (write-byte (ldb (byte 8 40) (slot-value msg 'command)) ostream)
  (write-byte (ldb (byte 8 48) (slot-value msg 'command)) ostream)
  (write-byte (ldb (byte 8 56) (slot-value msg 'command)) ostream)
)
(defmethod deserialize ((msg <WamCommands-request>) istream)
  "Deserializes a message object of type '<WamCommands-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'command)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'command)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'command)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'command)) (read-byte istream))
  (setf (ldb (byte 8 32) (slot-value msg 'command)) (read-byte istream))
  (setf (ldb (byte 8 40) (slot-value msg 'command)) (read-byte istream))
  (setf (ldb (byte 8 48) (slot-value msg 'command)) (read-byte istream))
  (setf (ldb (byte 8 56) (slot-value msg 'command)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<WamCommands-request>)))
  "Returns string type for a service object of type '<WamCommands-request>"
  "wam_ros/WamCommandsRequest")
(defmethod md5sum ((type (eql '<WamCommands-request>)))
  "Returns md5sum for a message object of type '<WamCommands-request>"
  #xa4d35f165bc8241b74b54e4eef915a13)
(defmethod message-definition ((type (eql '<WamCommands-request>)))
  "Returns full string definition for message of type '<WamCommands-request>"
  (format nil "uint64 command~%~%"))
(defmethod serialization-length ((msg <WamCommands-request>))
  (+ 0
     8
))
(defmethod ros-message-to-list ((msg <WamCommands-request>))
  "Converts a ROS message object to a list"
  (list '<WamCommands-request>
    (cons ':command (ros-message-to-list (command-val msg)))
))
;//! \htmlinclude WamCommands-response.msg.html

(defclass <WamCommands-response> (ros-message)
  ((response
    :accessor response-val
    :initarg :response
    :initform ""))
)
(defmethod serialize ((msg <WamCommands-response>) ostream)
  "Serializes a message object of type '<WamCommands-response>"
  (let ((__ros_str_len (length (slot-value msg 'response))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'response))
)
(defmethod deserialize ((msg <WamCommands-response>) istream)
  "Deserializes a message object of type '<WamCommands-response>"
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'response) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'response) __ros_str_idx) (code-char (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<WamCommands-response>)))
  "Returns string type for a service object of type '<WamCommands-response>"
  "wam_ros/WamCommandsResponse")
(defmethod md5sum ((type (eql '<WamCommands-response>)))
  "Returns md5sum for a message object of type '<WamCommands-response>"
  #xa4d35f165bc8241b74b54e4eef915a13)
(defmethod message-definition ((type (eql '<WamCommands-response>)))
  "Returns full string definition for message of type '<WamCommands-response>"
  (format nil "string response~%~%~%"))
(defmethod serialization-length ((msg <WamCommands-response>))
  (+ 0
     4 (length (slot-value msg 'response))
))
(defmethod ros-message-to-list ((msg <WamCommands-response>))
  "Converts a ROS message object to a list"
  (list '<WamCommands-response>
    (cons ':response (ros-message-to-list (response-val msg)))
))
(defmethod service-request-type ((msg (eql 'WamCommands)))
  '<WamCommands-request>)
(defmethod service-response-type ((msg (eql 'WamCommands)))
  '<WamCommands-response>)
(defmethod ros-datatype ((msg (eql 'WamCommands)))
  "Returns string type for a service object of type '<WamCommands>"
  "wam_ros/WamCommands")
