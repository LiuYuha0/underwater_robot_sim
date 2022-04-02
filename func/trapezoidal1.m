function [q,dq,ddq,err] = trapezoidal1(q_i,q_f,dq_c,tf,t)
%
% trapezoidal velocity profile at instant t
% 
%   [q,dq,ddq,err] = trapezoidal1(q_i,q_f,dq_c,t_f,t) 
%
%   input:
%       q_i     dim nx1     initial configuration
%       q_f     dim nx1     final configuration
%       dq_c    dim nx1     cruise velocity
%       tf      dim 1       final time
%       t       dim 1       output time
%   output:
%       q       dim nx1     position at t
%       dq      dim nx1     velocity at t
%       ddq     dim nx1     acceleration at t
%       err     dim 1       =1 solution does not exist, abrupt exit
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

err = 0;

% input as columns
[n,m] = size(q_i);
if m>n
    q_i = q_i';
    q_f = q_f';
    dq_c = dq_c';
end
n = size(q_i,1);
q   = zeros(n,1);
dq  = zeros(n,1);
ddq = zeros(n,1);

% repeat for each of the joint
for i=1:n
    delta = q_f(i) - q_i(i);
    dq_c(i) = sign(delta)*abs(dq_c(i));
    if (delta==0)
        q(i)   = q_i(i);
        dq(i)  = 0;
        ddq(i) = 0;
    else
        % constraint verification for joint i
        dq_r = abs(delta/tf);
        err = (abs(dq_c(i)) <= dq_r)|(abs(dq_c(i)) > 2*dq_r);
        if (err==1)
            fprintf('\n ERROR in trapezoidal1.m: error at input %d\n',i);
            error('check cruise speed');
        else
            % evaluates t_c
            t_c = tf - delta/dq_c(i);
            % evaluates ddq_c
            ddq_c = dq_c(i)/t_c;
            % if on the time slots
            if (t<=t_c)
                q(i)   = q_i(i) + 0.5*ddq_c*t^2;
                dq(i)  = ddq_c*t;
                ddq(i) = ddq_c;
            elseif (t<=(tf-t_c))
                q(i)   = q_i(i) + dq_c(i)*(t-0.5*t_c);
                dq(i)  = dq_c(i);
                ddq(i) = 0;
            elseif (t<=tf)
                q(i)   = q_f(i) - 0.5*ddq_c*(t-tf)^2 ;
                dq(i)  = ddq_c*(tf-t);
                ddq(i) = -ddq_c;
            else
                q(i)   = q_f(i);
                dq(i)  = 0;
                ddq(i) = 0;
            end
        end
    end
end
