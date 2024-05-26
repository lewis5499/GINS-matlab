function [resultRm] = getRm(phi)
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明
e_e2 = 0.00669437999013;
e_a=6.3781370e6 ;%长半轴
resultRm = e_a*(1-e_e2)/power(1-e_e2*(sin(phi))^2,1.5);
end

