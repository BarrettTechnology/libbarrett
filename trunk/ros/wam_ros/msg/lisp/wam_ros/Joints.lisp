; Auto-generated. Do not edit!


(in-package wam_ros-msg)


;//! \htmlinclude Joints.msg.html

(defclass <Joints> (ros-message)
  ((j
    :accessor j-val
    :initarg :j
    :initform #()))
)
(defmethod serialize ((msg <Joints>) ostream)
  "Serializes a message object of type '<Joints>"
  (let ((__ros_arr_len (length (slot-value msg 'j))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream)))
    (slot-value msg 'j))
)
(defmethod deserialize ((msg <Joints>) istream)
  "Deserializes a message object of type '<Joints>"
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'j) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'j)))
      (dotimes (i __ros_arr_len)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(defmethod ros-datatype ((msg (eql '<Joints>)))
  "Returns string type for a message object of type '<Joints>"
  "wam_ros/Joints")
(defmethod md5sum ((type (eql '<Joints>)))
  "Returns md5sum for a message object of type '<Joints>"
  #x59542e81b1fd2eaee58892b9055022e8)
(defmethod message-definition ((type (eql '<Joints>)))
  "Returns full string definition for message of type '<Joints>"
  (format nil "float64[] j~%~%~%"))
(defmethod serialization-length ((msg <Joints>))
  (+ 0
     4 (reduce #'+ (slot-value msg 'j) :key #'(lambda (ele) (declare (ignorable ele)) (+ 8)))
))
(defmethod ros-message-to-list ((msg <Joints>))
  "Converts a ROS message object to a list"
  (list '<Joints>
    (cons ':j (ros-message-to-list (j-val msg)))
))
