/* auto-generated by genmsg_cpp from /home/robot/libbarrett/ros/wam_ros/msg/Joints.msg.  Do not edit! */
package ros.pkg.wam_ros.msg;


import java.nio.ByteBuffer;

public  class Joints extends ros.communication.Message
{

  public java.lang.String j;

  public Joints() {
 super();
    j = new java.lang.String();

  }
  public static java.lang.String __s_getDataType() { return "wam_ros/Joints"; }
  public static java.lang.String __s_getMD5Sum() { return "0faeaaa42c2070611310a54ecba6c3ef"; }
  public static java.lang.String __s_getMessageDefinition()
  {
    return 
    "string j\n" + 
    "\n" + 
    "\n" + 
    "";
  }
  public java.lang.String getDataType() { return __s_getDataType(); }
  public java.lang.String getMD5Sum()   { return __s_getMD5Sum(); }
  public java.lang.String getMessageDefinition() { return __s_getMessageDefinition(); }
  public Joints clone() {
    Joints clone = (Joints)super.clone();
    return clone;
  }

  public static java.util.Map<java.lang.String, java.lang.String> fieldTypes() {
         java.util.HashMap<java.lang.String, java.lang.String> m = new java.util.HashMap<java.lang.String, java.lang.String>  ();      m.put("j", "java.lang.String");
     return m;
  }

  public static java.util.Set<java.lang.String> submessageTypes() {
         java.util.HashSet<java.lang.String> s = new java.util.HashSet<java.lang.String>  ();      return s;
  }

  public void setTo(ros.communication.Message __m) {
    if (!(__m instanceof Joints)) throw new RuntimeException("Invalid Type");
    Joints __m2 = (Joints) __m;
    j = __m2.j;
    }

  public int serializationLength() 
  {
    int __l = 0;
    __l += 4 + j.length(); // j
    return __l;
  }
  public void serialize(ByteBuffer bb, int seq) {
    Serialization.writeString(bb, j);
  }
  public void deserialize(ByteBuffer bb)  {
    j = Serialization.readString(bb);
  }
}

