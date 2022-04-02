function out=VectorSat(in,max_value)
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

in        = CheckVector(in);
max_value = CheckVector(max_value);

ratios = abs(in)./max_value;

if all(ratios<=1)
    out = in;
else
    alfa = max(ratios);
    out = in/alfa;
end

