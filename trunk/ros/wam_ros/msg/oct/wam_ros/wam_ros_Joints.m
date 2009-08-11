% Auto-generated.  Do not edit!

% msg = wam_ros_Joints()
%
% Joints message type, fields include:
% double{} j

% //! \htmlinclude Joints.msg.html
function msg = wam_ros_Joints()

msg = [];
msg.j = [];
msg.md5sum_ = @wam_ros_Joints___md5sum;
msg.type_ = @wam_ros_Joints___type;
msg.serializationLength_ = @wam_ros_Joints___serializationLength;
msg.serialize_ = @wam_ros_Joints___serialize;
msg.deserialize_ = @wam_ros_Joints___deserialize;

function x = wam_ros_Joints___md5sum()
x = '59542e81b1fd2eaee58892b9055022e8';

function x = wam_ros_Joints___type()
x = 'wam_ros/Joints';

function l__ = wam_ros_Joints___serializationLength(msg)
l__ =  ...
    + 4 + numel(msg.j) * (8);

function dat__ = wam_ros_Joints___serialize(msg__, seq__, fid__)
global rosoct

c__ = 0;
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
end
fwrite(fid__, numel(msg__.j), 'uint32');
fwrite(fid__, msg__.j(:), 'double');
if( file_created__ )
    fseek(fid__,0,SEEK_SET);
    dat__ = fread(fid__,Inf,'uint8=>uint8');
    fclose(fid__);
end

function msg__ = wam_ros_Joints___deserialize(dat__, fid__)
msg__ = wam_ros_Joints();
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
    fwrite(fid__,dat__,'uint8');
    fseek(fid__,0,SEEK_SET);
end
size__ = double(fread(fid__, 1, 'uint32=>uint32'));
msg__.j = fread(fid__, size__, 'double=>double');
if( file_created__ )
    fclose(fid__);
end
function l__ = wam_ros_Joints___sum_array_length__(x)
if( ~exist('x','var') || isempty(x) )
    l__ = 0;
else
    l__ = sum(x(:));
end

