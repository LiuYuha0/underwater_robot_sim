function out=CheckVector(in)
%
% Vectors are columns
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

if all(size(in)>1)
    error('Vector expected')
elseif size(in,1)==1
    out = in';
else
    out = in;
end