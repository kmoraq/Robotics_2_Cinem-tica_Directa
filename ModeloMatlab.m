clear L
clear l
l(1) = 89.45;
l(2) = 105.95;
l(3) = 100;
l(4) = 107.6;

L(1) = Link('revolute','alpha',pi/2,'a',0,   'd',l(1),'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
L(2) = Link('revolute','alpha',0,'a',l(2),   'd',0,'offset',atan(100/35),   'qlim',[-3*pi/4 3*pi/4]);
L(3) = Link('revolute','alpha',0,'a',l(3),   'd',0,'offset',-atan(100/35),   'qlim',[-3*pi/4 3*pi/4]);
L(4) = Link('revolute','alpha',0,'a',l(4),   'd',0,'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
PhantomX = SerialLink(L,'name','Px');

PhantomX.tool = trotx(pi/2)*troty(pi/2);

% Graficar robot
PhantomX.plot([0 0 0 0],'notiles');
hold on 
PhantomX.teach()