function[win_n]=winn(navstate)
    param = Param();
    wie_n=[param.wie*cos(navstate.pos(1));
             0;
        -param.wie*sin(navstate.pos(1))];
    wen_n=[navstate.vel(2)/(navstate.Rn+navstate.pos(3));
        -navstate.vel(1)/(navstate.Rm+navstate.pos(3));
        -navstate.vel(2)*tan(navstate.pos(1))/(navstate.Rn+navstate.pos(3))];
    win_n=wie_n+wen_n;
end

