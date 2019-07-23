%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% File: event_histogram.m
% Author: Eric Gronda
% Date: 7/22/19
% Email: eric.gronda@umbc.edu
% Description:
%  The goal of this program is to show that matching events (events that 
%  appear in multiple mirrors, but are the same in real space) occur within
%  a window of micro-seconds of each other.
%
% NOTE:
%   This program needs the data extracted from the rosbag first (see
%   sunfest_2019/src/matlab_ros/rosbag_to_matlab/extract_ros_data.m ).

%-------------------------------------------------------------------
% event_histogram()
% input:    matbag, extracted ros data
%           time_start, (optional) start point for video cropping
%           interval, time (in microsecs) to group each event
% output:   plt, information from the final histogram
function plt = event_histogram( matbag , time_start , interval)
    if nargin < 1
        matbag = "~/sunfest/src/matbag_ws/matlab_test2.mat";
    end
    
    % check arguments
    if nargin < 2
        time_start = 5;
    end
        
    if nargin < 3
        interval = 10e-6;
    end 

    % read in rosbag data, normalize time (note bag is stored in secs)
    bag = load(matbag);
    bag.events(3, :) = (bag.events(3 , :) - bag.events(3 , 1));% / 1000;
    
    % find where to start in array
    for i = 1 : size(bag.events , 2)
        if bag.events(3 , i) >= time_start
            break;
        end
    end
    
    % group events by time interval in time range
    duration = 300e-6; % (sec)
    timer = bag.events(3,i);
    tracker = time_start + interval;
    bins = zeros( (duration * 10^6) / (interval * 10^6) , 1);
    bindex = 1;
    
    while timer < time_start + duration
        timer = bag.events(3, i);

        % out of interval
        while timer >= tracker
            tracker = tracker + interval; 
            bindex = bindex + 1;
        end
        
        if bindex < size(bins , 1)
            bins(bindex , 1) = bins(bindex , 1) + 1;
        end
        i = i + 1;
        
    end    
    
    % set up and display plot
    plt = bar(bins);
    
end
  