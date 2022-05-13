%%
rosshutdown
rosinit;
%%
motorSvcClient = rossvcclient('/dynamixel_workbench/dynamixel_command');
motorCommandMsg= rosmessage(motorSvcClient);
%%
motorCommandMsg.AddrName="Goal_Position";
%%
pause(1)
t=[0 0 0 0 0];
for i=1:length(t)
    motorCommandMsg.Id=i;
    motorCommandMsg.Value=round(mapfun(t(i),-180,180,0,4096));%bits
    call(motorSvcClient,motorCommandMsg);
    pause(1);
end

%%
pause(1)
t=[-20 20 -20 20 0];
for i=1:length(t) 
    motorCommandMsg.Id=i;
    motorCommandMsg.Value=round(mapfun(t(i),-180,180,0,4096));%bits
    call(motorSvcClient,motorCommandMsg);
    pause(1);
end
%%
pause(1)
t=[30 -30 30 -30 0];
for i=1:length(t)
    motorCommandMsg.Id=i;
    motorCommandMsg.Value=round(mapfun(t(i),-180,180,0,4096));%bits
    call(motorSvcClient,motorCommandMsg);
    pause(1);
end
%%
pause(1)
t=[-90 15 -55 17 0];
for i=1:length(t)
    motorCommandMsg.Id=i;
    motorCommandMsg.Value=round(mapfun(t(i),-180,180,0,4096));%bits
    call(motorSvcClient,motorCommandMsg);
end
%%
pause(1)
t=[-90 45 -55 45 10];
for i=1:length(t)
    motorCommandMsg.Id=i;
    motorCommandMsg.Value=round(mapfun(t(i),-180,180,0,4096));%bits
    call(motorSvcClient,motorCommandMsg);
    pause(1);
end
%%
Sub=rossubscriber('/dynamixel_workbench/joint_states');
Sub.LatestMessage.Position;
%%
rosshutdown;