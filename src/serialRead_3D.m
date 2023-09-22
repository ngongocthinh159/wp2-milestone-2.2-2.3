%% Preparing the serial communication
% https://www.mathworks.com/help/matlab/import_export/read-streaming-data-from-arduino.html
% Clear the workspace
% Select the correct port and correct Baudrate
% Set the terminator to "CR/LF" (CR = Carriage Return and LF = Line Feed)
clc; clear; close all; format compact; format shortG
s = serialport('COM3', 9600);
s.Timeout = 1; % Set timeout to 1 seconds
warning('off', 'MATLAB:serialport:readline:unsuccessfulRead');

configureTerminator(s, "CR/LF");


% Use this code for real-time data vizualisation (plot angle vs time)
clc; close all; flush(s);

% Create a variable to store user input
global userInput;
userInput = "";
% Prepare the parameters for the animated line

%setup figure for our graph, most important thing is keyPressCallback,
%which listens for keys and sends commands to serial output
screen_property = get(0,'screensize');
h = figure('KeyPressFcn', @keyPressCallback, 'outerposition', ...
    [0, screen_property(4)/2, ...
    screen_property(3)/2, screen_property(4)/2]);
grid on; hold on;

O  = [0,0,0]';
e1 = [0,0,1]';
e2 = [1,0,0]';
e3 = [0,1,0]';
p1 = plot3(  [O(1), e1(1)], ...
    [O(2), e1(2)], ...
    [O(3), e1(3)], ...
    'LineWidth',2,'Color','r');
p2 = plot3(  [O(1), e2(1)], ...
    [O(2), e2(2)], ...
    [O(3), e2(3)], ...
    'LineWidth',2,'Color','g');
p3 = plot3(  [O(1), e3(1)], ...
    [O(2), e3(2)], ...
    [O(3), e3(3)], ...
    'LineWidth',2,'Color','b');

grid on;
view(-30,30); axis equal
xlabel('y','FontSize',16);
ylabel('z','FontSize',16);
zlabel('x','FontSize',16);
xlim([-1,1]);
ylim([-1,1]);
zlim([-1,1]);
%Animation
flush(s);

% Start the serial COM reading and animation
% Break the loop with Ctrl+C
while 1
    % write to serial
    if (~strcmp(userInput, ''))
        writeline(s, userInput);
        userInput = '';
    end

    %read from serial communication
    %do NOT add a semicolon so we can see its outputs via the command window
    try
        string = readline(s);
    catch ME
        if contains(ME.message, 'timeout')
        warning('Failed to read from serial port within 1 second.');
        string = ''; % or handle as needed
        else
            rethrow(ME); % if it's another error, rethrow it
        end
    end

    %scans the string for inputs: %f 
    %double %% means one % in the string
    if (size(string) == 0)
        continue;
    end
    disp(string)
    data = sscanf(string, "time: %d\ndeg_kalman_y: %f\ndeg_kalman_z: %f\nleftOutput: %f\nrightOutput: %f\nP: %f\nI: %f\nD: %f\noverlap: %f\nstartSignal: %f\nloadCell: %f\n");
    %make sure the data is correct i.e. 4 outputs is extracted from string
    if (size(data) < 2) 
        continue;
    end
    %assign the variables
    time = data(1);

    th1 = 0;        %% x
    th2 = data(2);  %% y
    th3 = data(3);  %% z

    R1 = [  cosd(th1), -sind(th1), 0;
        sind(th1),  cosd(th1), 0;
        0, 0, 1];
    R2 = [  1, 0, 0;
        0, cosd(th2), -sind(th2);
        0, sind(th2),  cosd(th2)];
    R3 = [  cosd(th3), 0, sind(th3);
        0, 1, 0;
        -sind(th3), 0, cosd(th3)];
    e1_pr = R3*R2*R1*e1;
    e2_pr = R3*R2*R1*e2;
    e3_pr = R3*R2*R1*e3;

    set(p1, 'XData', [O(1), e1_pr(1)], ...
        'YData', [O(2), e1_pr(2)], ...
        'ZData', [O(3), e1_pr(3)])
    set(p2, 'XData', [O(1), e2_pr(1)], ...
        'YData', [O(2), e2_pr(2)], ...
        'ZData', [O(3), e2_pr(3)])
    set(p3, 'XData', [O(1), e3_pr(1)], ...
        'YData', [O(2), e3_pr(2)], ...
        'ZData', [O(3), e3_pr(3)])
    %%drawnow;
end

function result = isCharacter(input)
    if ischar(input) && length(input) == 1
        result = true;
    elseif isstring(input) && strlength(input) == 1
        result = true;
    else
        result = false;
    end
end

% Command Window KeyPressFcn callback functions
function keyPressCallback(src, event)
    % Use the global variable 'userInput' inside the callback function
    global userInput;
    % Get the pressed key
    key = event.Key;

    % Handle the user input
    if isCharacter(key) || strcmp(key, '\n')
        userInput = key;
    end
end