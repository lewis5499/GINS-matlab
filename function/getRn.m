function [resultRn] = getRn(phi)
%UNTITLED4 此处显示有关此函数的摘要
%   此处显示详细说明
e_e2 = 0.00669437999013;
e_a=6.3781370e6 ;%长半轴
resultRn = e_a/sqrt(1-e_e2*(sin(phi))^2);
end

