function x=untitled(A,B,C,D)

assignin('base', 'A', A)
assignin('base', 'B', B)
assignin('base', 'C', C)
assignin('base', 'D', D)

%A = 1;
%B = 2;
%C = 1;
%D = 1;
%global x1
%global x2
%global y1
%global y2
%x1 = A;
%x2 = B;
%y1 = C;
%y2 = D;


%save('matlab.mat','A')
%save('matlab.mat','B')
%save('matlab.mat','C')
%save('matlab.mat','D')

%save A A
%save B B
%save C C
%save D D

%load('A.mat','A')
%load('B.mat','B')
%load('C.mat','C')
%load('D.mat','D')
%global kp1
%global Ki1
%global Kp2
%global Kp2

Kp1=double(A);
Ki1=double(B);
Ki2=double(C);
Kp2=double(D);

assignin('base', 'Kp1', Kp1)
assignin('base', 'Ki1', Ki1)
assignin('base', 'Ki2', Ki2)
assignin('base', 'Kp2', Kp2)
%save('matlab.mat','Ki1')
%save('matlab.mat','Kp1')
%save('matlab.mat','Ki2')
%save('matlab.mat','Kp2')

%save Ki1 Ki1
%save Kp1 Kp1
%save Ki2 Ki2
%save Kp2 Kp2

%load('Kp1.mat','Kp1')
%load('Ki1.mat','Ki1')
%load('Kp2.mat','Kp2')
%load('Ki2.mat','Ki2')


out=sim('pvtest1');
%disp(out.ITAE(end,1));
x = double(out.ITAE(end,1));
%end 
