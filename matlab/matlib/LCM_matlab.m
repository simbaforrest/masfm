classdef LCM_matlab < handle
  properties
    lc
    ma
    channel_name
    lcm_java_type_name
  end
  methods
    function obj=LCM_matlab(channel_name, lcm_java_type_name, max_buffer_size)
      % channel_name: name of the channel to receive lcm message
      % lcm_java_type_name: name of the java type of lcm message
      % max_buffer_size: max number of Bytes of the queue, default 1MB
      if ~exist('lcm.lcm.LCM', 'class')
        error('matlab cannot find java class lcm.lcm.LCM, is lcm.jar added by javaaddpath?')
      end
      assert(ischar(channel_name));
      assert(ischar(lcm_java_type_name));
      if ~exist(lcm_java_type_name,'class')
        error('matlab cannot find java class %s, is javaaddpath correctly set?', lcm_java_type_name)
      end
      if nargin<3
        max_buffer_size = 1024^2; % 1 MB maximum
      end
      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.ma = lcm.lcm.MessageAggregator();
      obj.ma.setMaxBufferSize(max_buffer_size);
      obj.channel_name = channel_name;
      obj.lcm_java_type_name = lcm_java_type_name;
      obj.lc.subscribe(obj.channel_name, obj.ma);
    end
    
    function ret=askForNext(obj)
      % blocking call, wait until receive a new detection
      % ret <lcm_java_type>: java type lcm_java_type, [] is failed
      msg = obj.ma.getNextMessage();
      if isempty(msg)
        ret=[];
        return;
      end
      ret = javaObject(obj.lcm_java_type_name, msg.data);
    end
    
    function ret=waitForNext(obj, millis_to_wait)
      % non-blocking call, wait for no more than millis_to_wait ms
      % ret <lcm_java_type>: java type lcm_java_type, [] if failed
      % millis_to_wait: milli-seconds to wait for next message
      msg = obj.ma.getNextMessage(millis_to_wait);
      if length(msg)<=0
        ret=[];
        return;
      end
      ret = javaObject(obj.lcm_java_type_name, msg.data);
    end
    
    function delete(obj)
      obj.lc = [];
      obj.ma = [];
    end
  end
end