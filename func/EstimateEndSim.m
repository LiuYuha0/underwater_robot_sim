function EstimateEndSim(t1, i, npti)
%
% Estimate time lenght of the simulation and print update
%
% EstimateEndSim(t1, i, npti)
%
% input:
%   t1   dim struct     clock() at starting time
%   i    dim 1x1        counter
%   npti dim 1x1    	total cycle number
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

if i==(round(i/25)*25)
    dt = etime(clock(),t1)/60;
    total = dt/i*npti;
    if total>1
        if dt<1
            fprintf('\n step %5d of %5d. estimated time %3.1f [m]',i,npti,total);
        else
            fprintf('\n step %5d of %5d. elapsed time %3.1f [m] estimated time %3.1f [m] remaining %3.1f [m]',i,npti,dt,total,total-dt);
        end
    end
end

