function PrintData(DH,PARAM)
%
% print at screen some model data
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

n = size(DH,1);
fprintf('\n number of link: %d', n)
fprintf('\n robot dry weight : %4.1f [kg]',sum(PARAM.m))
fprintf('\n robot buoyancy   : %4.1f [kg]',PARAM.rho*sum(PARAM.delta))
fprintf('\n robot wet weight : %4.1f [kg]',sum(PARAM.m)-PARAM.rho*sum(PARAM.delta))
fprintf('\n link masses     : ')
for i=1:n
	fprintf('%5.1f ',PARAM.m(i))
end
fprintf(' [kg]')
fprintf('\n link buoyancies : ')
for i=1:n
	fprintf('%5.1f ',PARAM.rho*PARAM.delta(i))
end
fprintf(' [kg]')
fprintf('\n')
