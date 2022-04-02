function PlotTasks(t,sigma_tilde_a,sigma_tilde_b,sigma_tilde_c)
%
% PlotTasks(t,sigma_tilde_a)
% PlotTasks(t,sigma_tilde_a,sigma_tilde_b)
% PlotTasks(t,sigma_tilde_a,sigma_tilde_b,sigma_tilde_c)
%
% input:
%       t               dim nptix1      time vector
%       sigma_tilde_a   dim maxnpti     task a
%       sigma_tilde_b   dim mbxnpti     task b
%       sigma_tilde_c   dim mcxnpti     task c
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

cla

if nargin==2
    plot(t,sigma_tilde_a)
    title('task a')
    xlabel('t [s]'),ylabel('sigma tilde a [-]')
    grid
elseif nargin==3
    subplot(211)
    plot(t,sigma_tilde_a)
    title('task a')
    xlabel('t [s]'),ylabel('sigma tilde a [-]')
    grid
    
    subplot(212)
    plot(t,sigma_tilde_b)
    title('task b')
    xlabel('t [s]'),ylabel('sigma tilde b [-]')
    grid
else
    subplot(311)
    plot(t,sigma_tilde_a)
    title('task a')
    xlabel('t [s]'),ylabel('sigma tilde a [-]')
    grid

    subplot(312)
    plot(t,sigma_tilde_b)
    title('task b')
    xlabel('t [s]'),ylabel('sigma tilde b [-]')
    grid

    subplot(313)
    plot(t,sigma_tilde_c)
    title('task c')
    xlabel('t [s]'),ylabel('sigma tilde c [-]')
    grid
end

