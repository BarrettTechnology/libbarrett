; Auto-generated. Do not edit!


(in-package wam_ros-srv)


;//! \htmlinclude WamCommands-request.msg.html

(defclass <WamCommands-request> (ros-message)
  ((command
    :accessor command-val
    :initarg :command
    :initform 0)
   (desiredJoints
    :accessor desiredJoints-val
    :initarg :desiredJoints
    :initform #()))
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
    (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream)))(slot-value msg 'desiredJoints))
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
  (setf (slot-value msg 'desiredJoints) (make-array 7))
  (let ((vals (slot-value msg 'desiredJoints)))
    (dotimes (i 7)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<WamCommands-request>)))
  "Returns string type for a service object of type '<WamCommands-request>"
  "wam_ros/WamCommandsRequest")
(defmethod md5sum ((type (eql '<WamCommands-request>)))
  "Returns md5sum for a message object of type '<WamCommands-request>"
  #x5487903b3c35697e058b56c00a8836fa)
(defmethod message-definition ((type (eql '<WamCommands-request>)))
  "Returns full string definition for message of type '<WamCommands-request>"
  (format nil "uint64 command~%float64[7] desiredJoints~%~%"))
(defmethod serialization-length ((msg <WamCommands-request>))
  (+ 0
     8
     0 (reduce #'+ (slot-value msg 'desiredJoints) :key #'(lambda (ele) (declare (ignorable ele)) (+ 8)))
))
(defmethod ros-message-to-list ((msg <WamCommands-request>))
  "Converts a ROS message object to a list"
  (list '<WamCommands-request>
    (cons ':command (ros-message-to-list (command-val msg)))
    (cons ':desiredJoints (ros-message-to-list (desiredJoints-val msg)))
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
  #x5487903b3c35697e058b56c00a8836fa)
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
