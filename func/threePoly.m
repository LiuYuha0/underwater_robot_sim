function [q_now]=threePoly(q_s, q_f, i, i_1, Ts)

if abs(q_s - q_f)==0
    q_now = q_f;
else
    anglev = 10/180*pi;
    tf = abs(q_f - q_s)/anglev;
    if i <= tf * 200 + 1 + i_1
        a0 = q_s;
        a2 =  3 * (q_f - q_s) / tf^2;
        a3 = -2 * (q_f - q_s) / tf^3;
        t = (i + 1 - i_1) * Ts;
        q_now = a0 + a2 * t^2 + a3 * t^3;
    else
        q_now = q_f;
    end
end

end



