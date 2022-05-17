clear;
s = serialport("COM15", 115200);
d = 2;
theta = -0.1;
% m = "|" + num2str(d) + ";" + num2str(theta)+"/";
% writeline(s,m);   
% pause(1); % pause 1s
for i=0:400    
    m = "|" + num2str(d) + ";" + num2str(theta)+"/";
    writeline(s,m);%writes the ASCII text data followed by the terminator to the specified serial port.
    j = 0.005; 
    d = d - j;
    n = s.NumBytesAvailable(); %Number of bytes available to be read, returned as a double.
    while s.NumBytesAvailable() == n
    end
    s.readline();%reads ASCII data from the ESP wi-fi connection device.
end 
