function iszupt = detectZUPT(imudata, imuindex, navstate, dt, realtime)

    %   零速探测有四种方式：
    %   1、加速度幅值检测;2、加速度滑动方差检测
    %   3、角速度幅值检测;4、角速度滑动方差检测
    if realtime
        iszupt = false;
        thisimu = imudata(imuindex, :);
        zuptacc_length = false;
        zuptacc_sigma2 = false;
        zuptgyr_length = false;
        zuptgyr_sigma2 = false;
    
        %% 索引满足条件进行探测
        if imuindex < size(imudata, 1)
            B = navstate.pos(1);
            H = navstate.pos(3);
            g = getGravity(B, H);
            
            thisimu = thisimu / dt;
            %% 1、加速度幅值检测（加速度计的三轴输出矢量应稳定在当地重力加速度值附近）
            if abs(sqrt(power(thisimu(5), 2) + power(thisimu(6), 2) + power(thisimu(7), 2)) - g) < 0.3
                zuptacc_length = true;
            end
            %% 2、加速度滑动方差检测
            if imuindex > 5 && imuindex < size(imudata, 1) - 5
                % 方差计算 
                bo = [false, false, false];
                for k = 1:3
                    sum_a = 0;
                    sigma2_a = 0;
                    for i = imuindex-5 : imuindex +5
                        sum_a = sum_a + imudata(i, k+4);
                    end
                    ave = sum_a/11;
                    for i = imuindex-5 : imuindex +5
                        sigma2_a = sigma2_a + power(imudata(i, k+4)-ave,2); 
                    end
                    sigma2_a = sigma2_a / 11;
                    if sigma2_a < 0.05
                        bo(k) = true;
                    end
                    if(bo(1)&&bo(2)&&bo(3))
                        zuptacc_sigma2 = true;
                    end
                end
            end
            %% 3、角速度幅值检测
            if sqrt(power(thisimu(2), 2) + power(thisimu(3), 2) + power(thisimu(4), 2))<0.05
                zuptgyr_length = true;
            end
            %% 4、角速度平滑检测
            if imuindex > 5 && imuindex < size(imudata, 1) - 5
                % 方差计算         
                bo = [false, false, false];
                for k = 1:3
                    sum_w = 0;
                    sigma2_w = 0;
                    for i = imuindex-5 : imuindex +5
                        sum_w = sum_w + imudata(i, k+1);
                    end
                    ave = sum_w/11;
                    for i = imuindex-5 : imuindex +5
                        sigma2_w = sigma2_w + power(imudata(i, k+1)-ave, 2); 
                    end
                    sigma2_w = 1/11 * sigma2_w;
                    if sigma2_w < 0.05
                        bo(k) = true;
                    end
                    if(bo(1)&&bo(2)&&bo(3))
                        zuptgyr_sigma2 = true;
                    end
                end
            end
            %% 最终联合判断
            if(zuptacc_length && zuptgyr_length && zuptacc_sigma2 && zuptgyr_sigma2)
                iszupt = true;
            end
    
        end

    end    

end