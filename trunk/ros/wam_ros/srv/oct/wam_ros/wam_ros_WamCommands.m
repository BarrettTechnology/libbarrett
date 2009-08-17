% Auto-generated.  Do not edit!

% [reqmsg,resmsg] = core_testsrv1()
%
function [reqmsg,resmsg] = wam_ros_WamCommands()
if( nargout > 0 )
    reqmsg = wam_ros_Request();
end
if( nargout > 0 )
    resmsg = wam_ros_Response();
end

% Auto-generated.  Do not edit!

% msg = wam_ros_Request()
%
% Request message type, fields include:
% uint64 command

% //! \htmlinclude Request.msg.html
function msg = wam_ros_Request()

msg = [];
msg.create_response_ = @wam_ros_Response;
msg.command = uint64(0);
msg.md5sum_ = @wam_ros_Request___md5sum;
msg.server_md5sum_ = @wam_ros_Request___server_md5sum;
msg.type_ = @wam_ros_Request___type;
msg.serializationLength_ = @wam_ros_Request___serializationLength;
msg.serialize_ = @wam_ros_Request___serialize;
msg.deserialize_ = @wam_ros_Request___deserialize;

function x = wam_ros_Request___md5sum()
x = '';

function x = wam_ros_Request___server_md5sum()
x = 'a4d35f165bc8241b74b54e4eef915a13';

function x = wam_ros_Request___type()
x = 'wam_ros/WamCommandsRequest';

function l__ = wam_ros_Request___serializationLength(msg)
l__ =  ...
    + 8;

function dat__ = wam_ros_Request___serialize(msg__, seq__, fid__)
global rosoct

c__ = 0;
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
end
c__ = c__ + fwrite(fid__, msg__.command, 'uint64');
if( c__ ~= 1 )
    error('some members of msg wam_ros:Request are initialized incorrectly!');
end
if( file_created__ )
    fseek(fid__,0,SEEK_SET);
    dat__ = fread(fid__,Inf,'uint8=>uint8');
    fclose(fid__);
end

function msg__ = wam_ros_Request___deserialize(dat__, fid__)
msg__ = wam_ros_Request();
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
    fwrite(fid__,dat__,'uint8');
    fseek(fid__,0,SEEK_SET);
end
msg__.command = fread(fid__,1,'uint64=>uint64');
if( file_created__ )
    fclose(fid__);
end
function l__ = wam_ros_Request___sum_array_length__(x)
if( ~exist('x','var') || isempty(x) )
    l__ = 0;
else
    l__ = sum(x(:));
end

% msg = wam_ros_Response()
%
% Response message type, fields include:
% string response

% //! \htmlinclude Response.msg.html
function msg = wam_ros_Response()

msg = [];
msg.response = '';
msg.md5sum_ = @wam_ros_Response___md5sum;
msg.server_md5sum_ = @wam_ros_Response___server_md5sum;
msg.type_ = @wam_ros_Response___type;
msg.serializationLength_ = @wam_ros_Response___serializationLength;
msg.serialize_ = @wam_ros_Response___serialize;
msg.deserialize_ = @wam_ros_Response___deserialize;

function x = wam_ros_Response___md5sum()
x = '';

function x = wam_ros_Response___server_md5sum()
x = 'a4d35f165bc8241b74b54e4eef915a13';

function x = wam_ros_Response___type()
x = 'wam_ros/WamCommandsResponse';

function l__ = wam_ros_Response___serializationLength(msg)
l__ =  ...
    + 4 + numel(msg.response);

function dat__ = wam_ros_Response___serialize(msg__, seq__, fid__)
global rosoct

c__ = 0;
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
end
fwrite(fid__, numel(msg__.response), 'uint32');
fwrite(fid__, msg__.response, 'uint8');
if( file_created__ )
    fseek(fid__,0,SEEK_SET);
    dat__ = fread(fid__,Inf,'uint8=>uint8');
    fclose(fid__);
end

function msg__ = wam_ros_Response___deserialize(dat__, fid__)
msg__ = wam_ros_Response();
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
    fwrite(fid__,dat__,'uint8');
    fseek(fid__,0,SEEK_SET);
end
size__ = double(fread(fid__, 1,'uint32=>uint32'));
msg__.response = fread(fid__, size__, '*char')';
if( file_created__ )
    fclose(fid__);
end
function l__ = wam_ros_Response___sum_array_length__(x)
if( ~exist('x','var') || isempty(x) )
    l__ = 0;
else
    l__ = sum(x(:));
end

