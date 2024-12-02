function x=untitled(A,B,C,D)
Kp1=double(1);
Ki1=double(1);
Kp2=double(1);
Ki2=double(1);
out=sim('pvtest1');
x = out.ITAE(end,1);
end 





