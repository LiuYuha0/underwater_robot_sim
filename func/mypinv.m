function out = mypinv(J,W,caller)
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

if nargin==2
    caller = 'x';
end

l = 1e-5;

[n,~]=size(J);

if rank(J)==n
    out = W*J'/(J*W*J');
else
    out = W*J'/(l*eye(n) + J*W*J');
    fprintf('\n WARNING in mypinv.m: task %s singular, damped inverse used',caller);  	
end

