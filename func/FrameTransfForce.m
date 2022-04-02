function [f_a, m_a] = FrameTransfForce(R_b_a, f_b, m_b, r_ab_a)
%
% Force/moment transformation from frame b to frame a
%
% function [f_a, m_a] = FrameTransfForce(R_b_a, f_b, m_b, r_ab_a)
%
% input:
%       R_b_a   dim 3x3     rotation matrix from frame b to frame a
%       f_b     dim 3x1     force expressed in frame b
%       m_b     dim 3x1     moment expressed in frame b
%       r_ab_a  dim 3x1     vector from origin of frame a to origin of
%                           frame b expressed in frame a
%
% output:
%       f_a     dim 3x1     force expressed in frame a
%       m_a     dim 3x1     moment expressed in frame a
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

f_b    = CheckVector(f_b);
m_b    = CheckVector(m_b);
r_ab_a = CheckVector(r_ab_a);

f_a = R_b_a*f_b;
m_a = R_b_a*m_b + cross(r_ab_a,R_b_a*f_b);