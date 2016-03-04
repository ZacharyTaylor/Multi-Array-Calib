classdef CustomMsgConsts
    %CustomMsgConsts This class stores all message types
    %   The message types are constant properties, which in turn resolve
    %   to the strings of the actual types.
    
    %   Copyright 2016 The MathWorks, Inc.
    
    properties (Constant)
        ros_vrpn_client_viconEstimator = 'ros_vrpn_client/viconEstimator'
        visensor_msgs_visensor_calibration = 'visensor_msgs/visensor_calibration'
        visensor_msgs_visensor_calibration_service = 'visensor_msgs/visensor_calibration_service'
        visensor_msgs_visensor_calibration_serviceRequest = 'visensor_msgs/visensor_calibration_serviceRequest'
        visensor_msgs_visensor_calibration_serviceResponse = 'visensor_msgs/visensor_calibration_serviceResponse'
        visensor_msgs_visensor_imu = 'visensor_msgs/visensor_imu'
        visensor_msgs_visensor_time_host = 'visensor_msgs/visensor_time_host'
        visensor_msgs_visensor_trigger = 'visensor_msgs/visensor_trigger'
    end
    
    methods (Static, Hidden)
        function messageList = getMessageList
            %getMessageList Generate a cell array with all message types.
            %   The list will be sorted alphabetically.
            
            persistent msgList
            if isempty(msgList)
                msgList = cell(8, 1);
                msgList{1} = 'ros_vrpn_client/viconEstimator';
                msgList{2} = 'visensor_msgs/visensor_calibration';
                msgList{3} = 'visensor_msgs/visensor_calibration_service';
                msgList{4} = 'visensor_msgs/visensor_calibration_serviceRequest';
                msgList{5} = 'visensor_msgs/visensor_calibration_serviceResponse';
                msgList{6} = 'visensor_msgs/visensor_imu';
                msgList{7} = 'visensor_msgs/visensor_time_host';
                msgList{8} = 'visensor_msgs/visensor_trigger';
            end
            
            messageList = msgList;
        end
        
        function serviceList = getServiceList
            %getServiceList Generate a cell array with all service types.
            %   The list will be sorted alphabetically.
            
            persistent svcList
            if isempty(svcList)
                svcList = cell(1, 1);
                svcList{1} = 'visensor_msgs/visensor_calibration_service';
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            serviceList = svcList;
        end
    end
end
